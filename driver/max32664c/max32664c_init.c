/*
 * Initialization code for the MAX32664C biometric sensor hub.
 *
 * Copyright (c) 2025, Daniel Kampert
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/sys/byteorder.h>
#include "max32664c.h"

LOG_MODULE_REGISTER(maxim_max32664c_init, CONFIG_SENSOR_LOG_LEVEL);

static uint8_t max86174_itime_code_from_dt(uint8_t dt_idx)
{
	/* FW35 valid codes: 0..3 (14.8, 29.4, 58.7, 117.3us) */
	if (dt_idx > 3) {
		return 0x02; /* safe default: 58.7us */
	}
	return dt_idx;
}

static uint8_t max86174_spsavg_code_from_dt(uint8_t dt_idx)
{
	/* FW35 valid codes: 0..4 (25/50/100/200/400 sps with avg) */
	if (dt_idx > 4) {
		return 0x00; /* safe default: 25sps avg1 */
	}
	return dt_idx;
}

static int max32664c_configure_max86174_fw35(const struct device *dev)
{
	const struct max32664c_config *cfg = dev->config;
	struct max32664c_data *data = dev->data;

	uint8_t itime_code  = max86174_itime_code_from_dt(cfg->min_integration_time_idx);
	uint8_t spsavg_code = max86174_spsavg_code_from_dt(cfg->min_sampling_rate_idx);

	LOG_INF("MAX86174 FW35: configuring ITIME=%u SPS/AVG=%u", itime_code, spsavg_code);

	/* Stop any running algo before AFE config (safe) */
	(void)max32664c_stop_algo(dev);
	k_msleep(20);

	for (uint8_t s = 0; s < MAX86174_MEAS_SLOTS; s++) {
		int ret = max86174_write_itime_slot(dev, s, itime_code);
		if (ret) {
			LOG_ERR("ITIME slot %u failed st=0x%02X (%s)",
				s, data->last_status, max32664c_status_str(data->last_status));
			return ret;
		}
	}
	k_msleep(20);

	for (uint8_t s = 0; s < MAX86174_MEAS_SLOTS; s++) {
		int ret = max86174_write_spsavg_slot(dev, s, spsavg_code);
		if (ret) {
			LOG_ERR("SPS/AVG slot %u failed st=0x%02X (%s)",
				s, data->last_status, max32664c_status_str(data->last_status));
			return ret;
		}
	}
	k_msleep(20);

	/* One-time readback log (very useful while stabilizing) */
	uint8_t rb[9];
	if (max86174_read_itime_all(dev, rb) == 0) {
		LOG_INF("ITIME slots: %02X %02X %02X %02X %02X %02X %02X %02X %02X",
			rb[0], rb[1], rb[2], rb[3], rb[4], rb[5], rb[6], rb[7], rb[8]);
	}
	if (max86174_read_spsavg_all(dev, rb) == 0) {
		LOG_INF("SPS/AVG slots: %02X %02X %02X %02X %02X %02X %02X %02X %02X",
			rb[0], rb[1], rb[2], rb[3], rb[4], rb[5], rb[6], rb[7], rb[8]);
	}

	return 0;
}

/** @brief      Set the SpO2 calibration coefficients.
 *              NOTE: See page 10 of the SpO2 and Heart Rate User Guide for additional information.
 *  @param dev  Pointer to device
 *  @return     0 when successful
 */
int max32664c_set_spo2_coeffs(const struct device *dev)
{
	const struct max32664c_config *config = dev->config;

	uint8_t tx[15];
	uint8_t rx;

	/* Write the calibration coefficients */
	tx[0] = 0x50;
	tx[1] = 0x07;
	tx[2] = 0x00;

	/* PATCH: Use Big Endian (MSB First) for the sensor hub */
	sys_put_be32((uint32_t)config->spo2_calib[0], &tx[3]);  /* Coeff A */
	sys_put_be32((uint32_t)config->spo2_calib[1], &tx[7]);  /* Coeff B */
	sys_put_be32((uint32_t)config->spo2_calib[2], &tx[11]); /* Coeff C */

	return max32664c_i2c_transmit(dev, tx, sizeof(tx), &rx, sizeof(rx),
				      MAX32664C_DEFAULT_CMD_DELAY);
}

/** @brief      Write the default configuration to the sensor hub.
 *  @param dev  Pointer to device
 *  @return     0 when successful
 */
static int max32664c_write_config(const struct device *dev)
{
	uint8_t rx;
	uint8_t tx[5];
	const struct max32664c_config *config = dev->config;
	struct max32664c_data *data = dev->data;

	/* Write the default settings */
	if (config->use_max86174) {
		/* MAX86174 FW35: use per-slot register configuration */
		if (max32664c_configure_max86174_fw35(dev)) {
			LOG_ERR("MAX86174 FW35 config failed: %d", data->last_status);
			return -EINVAL;
		}
	} else {
		/* Legacy AFE: use global min integration time command */
		tx[0] = 0x50;
		tx[1] = 0x07;
		tx[2] = 0x13;
		tx[3] = config->min_integration_time_idx;
		if (max32664c_i2c_transmit(dev, tx, 4, &rx, 1, MAX32664C_DEFAULT_CMD_DELAY)) {
			LOG_ERR("Can not write minimum integration time!");
			return -EINVAL;
		}
	}

	/* FW35 + MAX86174: sampling is configured via per-slot SPS/AVG (0x1B).
	 * The legacy 0x14 command is not valid and causes hub errors.
	 */
	if (!config->use_max86174) {
		tx[0] = 0x50;
		tx[1] = 0x07;
		tx[2] = 0x14;
		tx[3] = config->min_sampling_rate_idx;
		if (max32664c_i2c_transmit(dev, tx, 4, &rx, 1, MAX32664C_DEFAULT_CMD_DELAY)) {
			LOG_ERR("Can not write minimum sampling rate!");
			return -EINVAL;
		}
	} else {
		LOG_INF("MAX86174 FW35: skipping legacy min sampling rate (0x50 0x07 0x14)");
	}

	/* FW35 + MAX86174: max integration time and max sampling rate are not used.
	 * AFE configuration is done via per-slot registers.
	 */
	if (!config->use_max86174) {
		tx[0] = 0x50;
		tx[1] = 0x07;
		tx[2] = 0x15;
		tx[3] = config->max_integration_time_idx;
		if (max32664c_i2c_transmit(dev, tx, 4, &rx, 1, MAX32664C_DEFAULT_CMD_DELAY)) {
			LOG_ERR("Can not write maximum integration time!");
			return -EINVAL;
		}

		tx[0] = 0x50;
		tx[1] = 0x07;
		tx[2] = 0x16;
		tx[3] = config->max_sampling_rate_idx;
		if (max32664c_i2c_transmit(dev, tx, 4, &rx, 1, MAX32664C_DEFAULT_CMD_DELAY)) {
			LOG_ERR("Can not write maximum sampling rate!");
			return -EINVAL;
		}
	} else {
		LOG_INF("MAX86174 FW35: skipping legacy max itime/sampling (0x15/0x16)");
	}

	tx[0] = 0x10;
	tx[1] = 0x02;
	tx[2] = config->report_period;
	if (max32664c_i2c_transmit(dev, tx, 3, &rx, 1, MAX32664C_DEFAULT_CMD_DELAY)) {
		LOG_ERR("Can not set report period!");
		return -EINVAL;
	}

	/* Configure WHRM (0x17) - Skip for MAX86174A (FW 35.x.y) */
	if (!config->use_max86174) {
		tx[0] = 0x50;
		tx[1] = 0x07;
		tx[2] = 0x17;
		tx[3] = config->hr_config[0];
		tx[4] = config->hr_config[1];
		LOG_DBG("Configuring WHRM: 0x%02X%02X", tx[3], tx[4]);
		if (max32664c_i2c_transmit(dev, tx, 5, &rx, 1, MAX32664C_DEFAULT_CMD_DELAY)) {
			LOG_ERR("Can not configure WHRM!");
			return -EINVAL;
		}
	} else {
		LOG_INF("MAX86174 FW35: skipping legacy WHRM config (0x17)");
	}

	/* Configure SpO2 (0x18) - Skip for MAX86174A (FW 35.x.y) */
	if (!config->use_max86174) {
		tx[0] = 0x50;
		tx[1] = 0x07;
		tx[2] = 0x18;
		tx[3] = config->spo2_config[0];
		tx[4] = config->spo2_config[1];
		LOG_DBG("Configuring SpO2: 0x%02X%02X", tx[3], tx[4]);
		if (max32664c_i2c_transmit(dev, tx, 5, &rx, 1, MAX32664C_DEFAULT_CMD_DELAY)) {
			LOG_ERR("Can not configure SpO2!");
			return -EINVAL;
		}
	} else {
		LOG_INF("MAX86174 FW35: skipping legacy SpO2 config (0x18)");
	}

	/* Set the interrupt threshold */
	tx[0] = 0x10;
	tx[1] = 0x01;
	tx[2] = 0x01;
	if (max32664c_i2c_transmit(dev, tx, 3, &rx, 1, MAX32664C_DEFAULT_CMD_DELAY)) {
		LOG_ERR("Can not set interrupt threshold!");
		return -EINVAL;
	}

	if (max32664c_set_spo2_coeffs(dev)) {
		LOG_ERR("Can not set SpO2 calibration coefficients!");
		return -EINVAL;
	}

	data->motion_time = config->motion_time;
	data->motion_threshold = config->motion_threshold;
	memcpy(data->led_current, config->led_current, sizeof(data->led_current));

	return 0;
}

/** @brief      Read the configuration from the sensor hub.
 *  @param dev  Pointer to device
 *  @return     0 when successful
 */
static int max32664c_read_config(const struct device *dev)
{
	uint8_t tx[3];
	uint8_t rx[2];
	struct max32664c_data *data = dev->data;

	tx[0] = 0x11;
	tx[1] = 0x02;
	if (max32664c_i2c_transmit(dev, tx, 2, rx, 2, MAX32664C_DEFAULT_CMD_DELAY)) {
		LOG_ERR("Can not read report period!");
		return -EINVAL;
	}
	data->report_period = rx[1];

	tx[0] = 0x51;
	tx[1] = 0x07;
	tx[2] = 0x13;
	if (max32664c_i2c_transmit(dev, tx, 3, rx, 2, MAX32664C_DEFAULT_CMD_DELAY)) {
		LOG_ERR("Can not read minimum integration time!");
		return -EINVAL;
	}
	data->min_integration_time_idx = rx[1];

	tx[0] = 0x51;
	tx[1] = 0x07;
	tx[2] = 0x14;
	if (max32664c_i2c_transmit(dev, tx, 3, rx, 2, MAX32664C_DEFAULT_CMD_DELAY)) {
		LOG_ERR("Can not read minimum sampling rate!");
		return -EINVAL;
	}
	data->min_sampling_rate_idx = rx[1];

	tx[0] = 0x51;
	tx[1] = 0x07;
	tx[2] = 0x15;
	if (max32664c_i2c_transmit(dev, tx, 3, rx, 2, MAX32664C_DEFAULT_CMD_DELAY)) {
		LOG_ERR("Can not read maximum integration time!");
		return -EINVAL;
	}
	data->max_integration_time_idx = rx[1];

	tx[0] = 0x51;
	tx[1] = 0x07;
	tx[2] = 0x16;
	if (max32664c_i2c_transmit(dev, tx, 3, rx, 2, MAX32664C_DEFAULT_CMD_DELAY)) {
		LOG_ERR("Can not read maximum sampling rate!");
		return -EINVAL;
	}
	data->max_sampling_rate_idx = rx[1];

	return 0;
}

int max32664c_init_hub(const struct device *dev)
{
	struct max32664c_data *data = dev->data;

	LOG_DBG("Initialize sensor hub");

	/* FIX: Increase LED Current to 20mA (0x14).
	 * 10mA (0x0A) is too weak for SpO2 and causes "Low Signal: 9" error.
	 */
	data->led_current[0] = 20; /* Green */
	data->led_current[1] = 20; /* IR */
	data->led_current[2] = 20; /* Red */
	data->led_current[3] = 0;  /* Off */

	/* Write Config */
	int ret = max32664c_write_config(dev);
	if (ret) {
		LOG_ERR("Default cfg failed (continuing)");
	}

	/* Initialize Message Queues (CRITICAL FIX) */
#ifdef CONFIG_MAX32664C_USE_STATIC_MEMORY
	/* Static Memory Init */
	k_msgq_init(&data->raw_report_queue, data->raw_report_queue_buffer,
		    sizeof(struct max32664c_raw_report_t), CONFIG_MAX32664C_QUEUE_SIZE);

	k_msgq_init(&data->scd_report_queue, data->scd_report_queue_buffer,
		    sizeof(struct max32664c_scd_report_t), CONFIG_MAX32664C_QUEUE_SIZE);

#ifdef CONFIG_MAX32664C_USE_EXTENDED_REPORTS
	k_msgq_init(&data->ext_report_queue, data->ext_report_queue_buffer,
		    sizeof(struct max32664c_ext_report_t), CONFIG_MAX32664C_QUEUE_SIZE);
#else
	k_msgq_init(&data->report_queue, data->report_queue_buffer,
		    sizeof(struct max32664c_report_t), CONFIG_MAX32664C_QUEUE_SIZE);
#endif

#else
	/* Dynamic Memory Init */
	LOG_INF("Allocating queues with dynamic memory");
	
	if (k_msgq_alloc_init(&data->raw_report_queue, sizeof(struct max32664c_raw_report_t),
			      CONFIG_MAX32664C_QUEUE_SIZE)) {
		LOG_ERR("Failed to allocate raw_report_queue!");
		return -ENOMEM;
	}

	if (k_msgq_alloc_init(&data->scd_report_queue, sizeof(struct max32664c_scd_report_t),
			      CONFIG_MAX32664C_QUEUE_SIZE)) {
		LOG_ERR("Failed to allocate scd_report_queue!");
		return -ENOMEM;
	}

#ifdef CONFIG_MAX32664C_USE_EXTENDED_REPORTS
	if (k_msgq_alloc_init(&data->ext_report_queue, sizeof(struct max32664c_ext_report_t),
			      CONFIG_MAX32664C_QUEUE_SIZE)) {
		LOG_ERR("Failed to allocate ext_report_queue!");
		return -ENOMEM;
	}
#else
	if (k_msgq_alloc_init(&data->report_queue, sizeof(struct max32664c_report_t),
			      CONFIG_MAX32664C_QUEUE_SIZE)) {
		LOG_ERR("Failed to allocate report_queue!");
		return -ENOMEM;
	}
#endif
#endif /* CONFIG_MAX32664C_USE_STATIC_MEMORY */

	LOG_INF("Message queues initialized successfully");

	/* Create Worker Thread */
	data->is_thread_running = true;
	data->thread_id = k_thread_create(&data->thread, data->thread_stack,
					  K_THREAD_STACK_SIZEOF(data->thread_stack),
					  (k_thread_entry_t)max32664c_worker, (void *)dev, NULL,
					  NULL, K_LOWEST_APPLICATION_THREAD_PRIO, 0, K_NO_WAIT);
	k_thread_suspend(data->thread_id);
	k_thread_name_set(data->thread_id, "max32664c_worker");

	LOG_DBG("Initial configuration:");

#ifndef CONFIG_MAX32664C_USE_STATIC_MEMORY
	LOG_DBG("\tUsing dynamic memory for queues and buffers");
#else
	LOG_DBG("\tUsing static memory for queues and buffers");
#endif /* CONFIG_MAX32664C_USE_STATIC_MEMORY */

#ifdef CONFIG_MAX32664C_USE_EXTENDED_REPORTS
	LOG_DBG("\tUsing extended reports");
#else
	LOG_DBG("\tUsing normal reports");
#endif /* CONFIG_MAX32664C_USE_EXTENDED_REPORTS*/

	LOG_DBG("\tReport period: %u", data->report_period);
	LOG_DBG("\tMinimum integration time: %u", data->min_integration_time_idx);
	LOG_DBG("\tMinimum sampling rate: %u", data->min_sampling_rate_idx);
	LOG_DBG("\tMaximum integration time: %u", data->max_integration_time_idx);
	LOG_DBG("\tMaximum sampling rate: %u", data->max_sampling_rate_idx);

	return 0;
}