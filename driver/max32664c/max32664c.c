/*
 * Copyright (c) 2025, Daniel Kampert
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/util.h>   /* IS_ENABLED() */

#include "max32664c.h"
#include "app_i2c_lock.h"

#define DT_DRV_COMPAT maxim_max32664c

#if (DT_NUM_INST_STATUS_OKAY(DT_DRV_COMPAT) == 0)
#warning "max32664c driver enabled without any devices"
#endif

LOG_MODULE_REGISTER(maxim_max32664c, CONFIG_SENSOR_LOG_LEVEL);

#define MAX32664C_POLL_MS           10
#define MAX32664C_POLL_TRIES        80     /* ~800ms */

static inline bool max32664c_status_is_busy(uint8_t st)
{
	return (st == MAX32664C_ST_BUSY) || (st == MAX32664C_ST_BUSY_BOOT);
}

const char *max32664c_status_str(uint8_t st)
{
	switch (st) {
	case 0x00: return "SUCCESS";
	case 0x01: return "ILLEGAL_FAMILY/CMD";
	case 0x02: return "UNIMPLEMENTED";
	case 0x03: return "BAD_LENGTH";
	case 0x04: return "BAD_VALUE";
	case 0x05: return "BUSY/WRONG_MODE";
	case 0xFE: return "BUSY";
	case 0xFF: return "NO_RESPONSE";
	default:   return "OTHER";
	}
}

/**
 * @brief Wakes the sensor.
 * LOGICAL 1 (Active) + DT ACTIVE_LOW = PHYSICAL LOW (GND)
 */
static void max32664c_mfio_assert_wake(const struct device *dev)
{
	const struct max32664c_config *cfg = dev->config;
	gpio_pin_set_dt(&cfg->mfio_gpio, 1);
}

/**
 * @brief Releases the line, allows the sensor to sleep.
 * LOGICAL 0 (Inactive) + DT ACTIVE_LOW = PHYSICAL HIGH (Hi-Z)
 */
static void max32664c_mfio_deassert_sleep(const struct device *dev)
{
	const struct max32664c_config *cfg = dev->config;
	gpio_pin_set_dt(&cfg->mfio_gpio, 0);
}

static inline void max32664c_mfio_hold(const struct device *dev)
{
	struct max32664c_data *data = dev->data;
	atomic_inc(&data->mfio_hold);
}

static inline void max32664c_mfio_release(const struct device *dev)
{
	struct max32664c_data *data = dev->data;
	atomic_dec(&data->mfio_hold);
}

static inline bool max32664c_should_keep_awake(struct max32664c_data *data)
{
	if (atomic_get(&data->mfio_hold) > 0) return true;

	switch (data->op_mode) {
	case MAX32664C_OP_MODE_IDLE:
	case MAX32664C_OP_MODE_STOP_ALGO:
		return false;
	default:
		return true;
	}
}

/* MAX86174 FW35 register configuration functions */
int max86174_write_itime_slot(const struct device *dev, uint8_t slot, uint8_t itime_code)
{
	uint8_t tx[] = { MAX32664C_FAM_AFE_WR, MAX32664C_IDX_AFE_CFG, MAX32664C_SUB_ITIME, slot, itime_code };
	uint8_t rx[1];
	return max32664c_i2c_transmit(dev, tx, sizeof(tx), rx, sizeof(rx), 50);
}

int max86174_write_spsavg_slot(const struct device *dev, uint8_t slot, uint8_t spsavg_code)
{
	uint8_t tx[] = { MAX32664C_FAM_AFE_WR, MAX32664C_IDX_AFE_CFG, MAX32664C_SUB_SPSAVG, slot, spsavg_code };
	uint8_t rx[1];
	return max32664c_i2c_transmit(dev, tx, sizeof(tx), rx, sizeof(rx), 50);
}

/* Debug readback: rx = [status][slot0..slot8] */
int max86174_read_itime_all(const struct device *dev, uint8_t out[MAX86174_MEAS_SLOTS])
{
	uint8_t tx[] = { MAX32664C_FAM_AFE_RD, MAX32664C_IDX_AFE_CFG, MAX32664C_SUB_ITIME };
	uint8_t rx[1 + MAX86174_MEAS_SLOTS];
	int ret = max32664c_i2c_transmit(dev, tx, sizeof(tx), rx, sizeof(rx), 50);
	if (ret == 0) {
		memcpy(out, &rx[1], MAX86174_MEAS_SLOTS);
	}
	return ret;
}

int max86174_read_spsavg_all(const struct device *dev, uint8_t out[MAX86174_MEAS_SLOTS])
{
	uint8_t tx[] = { MAX32664C_FAM_AFE_RD, MAX32664C_IDX_AFE_CFG, MAX32664C_SUB_SPSAVG };
	uint8_t rx[1 + MAX86174_MEAS_SLOTS];
	int ret = max32664c_i2c_transmit(dev, tx, sizeof(tx), rx, sizeof(rx), 50);
	if (ret == 0) {
		memcpy(out, &rx[1], MAX86174_MEAS_SLOTS);
	}
	return ret;
}

/* Forward declaration for deferred initialization helper */
static int max32664c_ensure_ready(const struct device *dev);

int max32664c_i2c_transmit(const struct device *dev, uint8_t *tx_buf, uint8_t tx_len,
			   uint8_t *rx_buf, uint32_t rx_len, uint16_t delay_ms)
{
	const struct max32664c_config *cfg = dev->config;
	struct max32664c_data *data = dev->data;
	int ret;

	k_mutex_lock(&i2c_lock, K_FOREVER);

	/* Wake hub: MFIO low >= 250us before I2C, hold low during transaction */
	max32664c_mfio_assert_wake(dev);
	
	/* FIX: Increase wait to 1ms. Deep sleep wake-up time varies; 1ms guarantees I2C ready */
	k_busy_wait(1000);

	ret = i2c_write_dt(&cfg->i2c, tx_buf, tx_len);
	if (ret) {
		LOG_ERR("I2C write failed: %d", ret);
		goto out;
	}

	k_msleep(delay_ms);

	/* Always read at least 1 byte status */
	uint8_t st_only;
	uint8_t *rb = (rx_buf && rx_len) ? rx_buf : &st_only;
	size_t rl = (rx_buf && rx_len) ? rx_len : 1;

	for (int i = 0; i < MAX32664C_POLL_TRIES; i++) {
		ret = i2c_read_dt(&cfg->i2c, rb, rl);
		if (ret) {
			LOG_ERR("I2C read failed: %d", ret);
			goto out;
		}

		data->last_status = rb[0];

		/* Handle 0xFF (no response) with retry */
		if (rb[0] == MAX32664C_ST_UNKNOWN) {
			k_msleep(5);
			ret = i2c_read_dt(&cfg->i2c, rb, rl);
			if (ret) {
				LOG_ERR("I2C read retry failed: %d", ret);
				goto out;
			}
			data->last_status = rb[0];
			if (rb[0] == MAX32664C_ST_UNKNOWN) {
				LOG_ERR("Hub returned 0xFF (no response) for tx_len=%u tx[0..3]=%02X %02X %02X %02X",
					(unsigned)tx_len,
					tx_len > 0 ? tx_buf[0] : 0,
					tx_len > 1 ? tx_buf[1] : 0,
					tx_len > 2 ? tx_buf[2] : 0,
					tx_len > 3 ? tx_buf[3] : 0);
				ret = -EIO;
				goto out;
			}
		}

		if (rb[0] == MAX32664C_ST_SUCCESS) {
			ret = 0;
			goto out;
		}

		if (max32664c_status_is_busy(rb[0])) {
			k_msleep(MAX32664C_POLL_MS);
			continue;
		}

		LOG_ERR("Hub status=0x%02X (%s) tx_len=%u tx[0..4]=%02X %02X %02X %02X %02X",
			rb[0], max32664c_status_str(rb[0]),
			(unsigned)tx_len,
			tx_len > 0 ? tx_buf[0] : 0,
			tx_len > 1 ? tx_buf[1] : 0,
			tx_len > 2 ? tx_buf[2] : 0,
			tx_len > 3 ? tx_buf[3] : 0,
			tx_len > 4 ? tx_buf[4] : 0);

		if (rb[0] == MAX32664C_ST_BAD_LEN || rb[0] == MAX32664C_ST_BAD_VALUE) {
			ret = -EINVAL;
		} else if (rb[0] == MAX32664C_ST_ILLEGAL_CMD || rb[0] == MAX32664C_ST_UNIMPL) {
			ret = -ENOTSUP;
		} else {
			ret = -EIO;
		}
		goto out;
	}

	ret = -EAGAIN; /* still busy */

out:
	/* Keep hub awake while algo/raw/scd/WOM is active */
	if (data->op_mode == MAX32664C_OP_MODE_IDLE ||
	    data->op_mode == MAX32664C_OP_MODE_STOP_ALGO) {
		max32664c_mfio_deassert_sleep(dev);   /* allow sleep */
	} else {
		max32664c_mfio_assert_wake(dev);      /* keep awake */
	}

	k_mutex_unlock(&i2c_lock);
	return ret;
}

/** @brief      Check the accelerometer and AFE WHOAMI registers.
 *              This function is called during device initialization.
 *  @param dev  Pointer to device
 *  @return     0 when successful
 */
static int max32664c_check_sensors(const struct device *dev)
{
	uint8_t tx[3];
	uint8_t rx[2];
	struct max32664c_data *data = dev->data;
	const struct max32664c_config *config = dev->config;

	LOG_DBG("Checking sensors...");

	/* Read MAX86141 WHOAMI */
	tx[0] = 0x41;
	tx[1] = 0x00;
	tx[2] = 0xFF;
	if (max32664c_i2c_transmit(dev, tx, 3, rx, 2, MAX32664C_DEFAULT_CMD_DELAY)) {
		return -EINVAL;
	}

	data->afe_id = rx[1];
	
	/* Check which AFE is detected */
	if (config->use_max86141 && data->afe_id == MAX86141_AFE_ID) {
		LOG_DBG("\tUsing MAX86141 as AFE");
	} else if (config->use_max86161 && data->afe_id == MAX86161_AFE_ID) {
		LOG_DBG("\tUsing MAX86161 as AFE");
	} else if (config->use_max86174 && data->afe_id == MAX86174_AFE_ID) {
		LOG_DBG("\tUsing MAX86174A as AFE");
	} else {
		LOG_ERR("\tAFE mismatch or undefined! Read: 0x%X", data->afe_id);
		return -ENODEV;
	}

	LOG_DBG("\tAFE WHOAMI OK: 0x%X", data->afe_id);

	/* Read Accelerometer WHOAMI */
	tx[0] = 0x41;
	tx[1] = 0x04;
	tx[2] = 0x0F;
	if (max32664c_i2c_transmit(dev, tx, 3, rx, 2, MAX32664C_DEFAULT_CMD_DELAY)) {
		return -EINVAL;
	}

	data->accel_id = rx[1];
	/* The sensor hub firmware supports only two accelerometers and one is set to */
	/* EoL. The remaining one is the ST LIS2DS12. */
	if (data->accel_id != 0x43) {
		LOG_ERR("\tAccelerometer WHOAMI failed: 0x%X", data->accel_id);
		return -ENODEV;
	}

	LOG_DBG("\tAccelerometer WHOAMI OK: 0x%X", data->accel_id);

	return 0;
}

int max32664c_get_ids(const struct device *dev, uint8_t *afe_id, uint8_t *acc_id)
{
	struct max32664c_data *data = dev->data;

	if (afe_id) {
		*afe_id = data->afe_id;
	}

	if (acc_id) {
		*acc_id = data->accel_id;
	}

	return 0;
}

/** @brief      Stop the current algorithm.
 *  @param dev  Pointer to device
 *  @return     0 when successful
 */
int max32664c_stop_algo(const struct device *dev)
{
	uint8_t rx;
	uint8_t tx[3];
	struct max32664c_data *data = dev->data;

	if (data->op_mode == MAX32664C_OP_MODE_IDLE) {
		LOG_DBG("No algorithm running, nothing to stop.");
		return 0;
	}

	LOG_DBG("Stop the current algorithm...");

	/* Stop the algorithm */
	tx[0] = 0x52;
	tx[1] = 0x07;
	tx[2] = 0x00;
	if (max32664c_i2c_transmit(dev, tx, 3, &rx, 1, 120)) {
		return -EINVAL;
	}

	switch (data->op_mode) {
	case MAX32664C_OP_MODE_RAW: {
#ifndef CONFIG_MAX32664C_USE_STATIC_MEMORY
		k_msgq_cleanup(&data->raw_report_queue);
#endif /* CONFIG_MAX32664C_USE_STATIC_MEMORY */
		break;
	}
#ifdef CONFIG_MAX32664C_USE_EXTENDED_REPORTS
	case MAX32664C_OP_MODE_ALGO_AEC_EXT:
	case MAX32664C_OP_MODE_ALGO_AGC_EXT: {
#ifndef CONFIG_MAX32664C_USE_STATIC_MEMORY
		k_msgq_cleanup(&data->ext_report_queue);
#endif /* CONFIG_MAX32664C_USE_STATIC_MEMORY */
		break;
	}
#else
	case MAX32664C_OP_MODE_ALGO_AEC:
	case MAX32664C_OP_MODE_ALGO_AGC: {
#ifndef CONFIG_MAX32664C_USE_STATIC_MEMORY
		k_msgq_cleanup(&data->report_queue);
#endif /* CONFIG_MAX32664C_USE_STATIC_MEMORY */
		break;
	}
#endif /* CONFIG_MAX32664C_USE_EXTENDED_REPORTS */
	case MAX32664C_OP_MODE_SCD: {
#ifndef CONFIG_MAX32664C_USE_STATIC_MEMORY
		k_msgq_cleanup(&data->scd_report_queue);
#endif /* CONFIG_MAX32664C_USE_STATIC_MEMORY */
		break;
	}
	default: {
		LOG_ERR("Unknown algorithm mode: %d", data->op_mode);
		return -EINVAL;
	}
	};

	data->op_mode = MAX32664C_OP_MODE_IDLE;

	k_thread_suspend(data->thread_id);

	return 0;
}

/** @brief      Put the device into raw measurement mode.
 *  @param dev  Pointer to device
 *  @return     0 when successful
 */
static int max32664c_set_mode_raw(const struct device *dev)
{
	uint8_t rx;
	uint8_t tx[4];
	struct max32664c_data *data = dev->data;
	int ret;

	max32664c_mfio_hold(dev);

	/* Stop the current algorithm mode */
	if (max32664c_stop_algo(dev)) {
		LOG_ERR("Failed to stop the algorithm!");
		ret = -EINVAL;
		goto out;
	}

	LOG_INF("Entering RAW mode...");

	/* Set mode early so MFIO stays low during initialization */
	data->op_mode = MAX32664C_OP_MODE_RAW;

	/* Set the output format to sensor data only */
	tx[0] = 0x10;
	tx[1] = 0x00;
	tx[2] = MAX32664C_OUT_SENSOR_ONLY;
	if (max32664c_i2c_transmit(dev, tx, 3, &rx, 1, MAX32664C_DEFAULT_CMD_DELAY)) {
		ret = -EINVAL;
		goto out;
	}

	/* Enable the AFE */
	tx[0] = 0x44;
	tx[1] = 0x00;
	tx[2] = 0x01;
	tx[3] = 0x00;
	if (max32664c_i2c_transmit(dev, tx, 4, &rx, 1, 250)) {
		ret = -EINVAL;
		goto out;
	}

	/* Enable the accelerometer */
	if (max32664c_acc_enable(dev, true)) {
		ret = -EINVAL;
		goto out;
	}

	/* Set AFE sample rate to 100 Hz */
	tx[0] = 0x40;
	tx[1] = 0x00;
	tx[2] = 0x12;
	tx[3] = 0x18;
	if (max32664c_i2c_transmit(dev, tx, 4, &rx, 1, MAX32664C_DEFAULT_CMD_DELAY)) {
		ret = -EINVAL;
		goto out;
	}

	/* Set the LED current */
	for (uint8_t i = 0; i < sizeof(data->led_current); i++) {
		tx[0] = 0x40;
		tx[1] = 0x00;
		tx[2] = 0x23 + i;
		tx[3] = data->led_current[i];
		LOG_INF("Set LED%d current: %u", i + 1, data->led_current[i]);
		if (max32664c_i2c_transmit(dev, tx, 4, &rx, 1, MAX32664C_DEFAULT_CMD_DELAY)) {
			LOG_ERR("Can not set LED%d current", i + 1);
			ret = -EINVAL;
			goto out;
		}
	}

#ifndef CONFIG_MAX32664C_USE_STATIC_MEMORY
	if (k_msgq_alloc_init(&data->raw_report_queue, sizeof(struct max32664c_raw_report_t),
			      CONFIG_MAX32664C_QUEUE_SIZE)) {
		LOG_ERR("Failed to allocate RAW report queue!");
		ret = -ENOMEM;
		goto out;
	}
#endif /* CONFIG_MAX32664C_USE_STATIC_MEMORY */

	k_thread_resume(data->thread_id);

	ret = 0;

out:
	max32664c_mfio_release(dev);
	return ret;
}

/** @brief              Put the sensor hub into algorithm mode.
 *  @param dev          Pointer to device
 *  @param device_mode  Target device mode
 *  @param algo_mode    Target algorithm mode
 *  @param extended     Set to #true when the extended mode should be used
 *  @return             0 when successful
 */
static int max32664c_set_mode_algo(const struct device *dev, enum max32664c_device_mode device_mode,
				   enum max32664c_algo_mode algo_mode, bool extended)
{
	uint8_t rx;
	uint8_t tx[5];
	struct max32664c_data *data = dev->data;
	int ret;

	max32664c_mfio_hold(dev);

	/* FIX: Force STOP Algorithm. Send command directly to ensure sensor is ready.
	 * Do NOT use max32664c_stop_algo() - it may return early if driver thinks it's IDLE,
	 * but hardware might still be running. Ignore return value - may already be stopped. */
	tx[0] = 0x52; tx[1] = 0x07; tx[2] = 0x00;
	max32664c_i2c_transmit(dev, tx, 3, &rx, 1, 100);

	LOG_DBG("Entering algorithm mode...");

	/* Set mode early so MFIO stays low during initialization */
	if (device_mode == MAX32664C_OP_MODE_ALGO_AEC) {
		data->op_mode = extended ? MAX32664C_OP_MODE_ALGO_AEC_EXT : MAX32664C_OP_MODE_ALGO_AEC;
	} else {
		data->op_mode = extended ? MAX32664C_OP_MODE_ALGO_AGC_EXT : MAX32664C_OP_MODE_ALGO_AGC;
	}

#ifndef CONFIG_MAX32664C_USE_EXTENDED_REPORTS
	if (extended) {
		LOG_ERR("No support for extended reports enabled!");
		ret = -EINVAL;
		goto out;
	}
#endif /* CONFIG_MAX32664C_USE_EXTENDED_REPORTS */

	/* 1. Set the output mode to sensor and algorithm data */
	tx[0] = 0x10;
	tx[1] = 0x00;
	tx[2] = MAX32664C_OUT_ALGO_AND_SENSOR;
	if (max32664c_i2c_transmit(dev, tx, 3, &rx, 1, MAX32664C_DEFAULT_CMD_DELAY)) {
		LOG_ERR("Failed to set Output Mode");
		ret = -EINVAL;
		goto out;
	}

	/* 2. Set the algorithm mode */
	tx[0] = 0x50;
	tx[1] = 0x07;
	tx[2] = 0x0A;
	tx[3] = algo_mode;
	if (max32664c_i2c_transmit(dev, tx, 4, &rx, 1, MAX32664C_DEFAULT_CMD_DELAY)) {
		LOG_ERR("Failed to set Algo Mode");
		ret = -EINVAL;
		goto out;
	}

	/* FIX: RELOAD CALIBRATION COEFFICIENTS HERE */
	/* The hub clears these on mode reset. We must restore them before starting. */
	if (max32664c_set_spo2_coeffs(dev)) {
		LOG_ERR("Failed to reload SpO2 coeffs!");
		ret = -EINVAL;
		goto out;
	}

	/* Enable accelerometer - algorithm requires accel data */
	if (max32664c_acc_enable(dev, true)) {
		LOG_ERR("Failed to enable accelerometer!");
		ret = -EINVAL;
		goto out;
	}

	/* Enable the AFE - without this, LEDs will never turn on! */
	tx[0] = 0x44;
	tx[1] = 0x00;
	tx[2] = 0x01;
	tx[3] = 0x00;
	if (max32664c_i2c_transmit(dev, tx, 4, &rx, 1, 250)) {
		LOG_ERR("Failed to enable AFE!");
		ret = -EINVAL;
		goto out;
	}

	if (device_mode == MAX32664C_OP_MODE_ALGO_AEC) {
		LOG_DBG("Entering AEC mode...");

		/* Enable AEC */
		tx[0] = 0x50;
		tx[1] = 0x07;
		tx[2] = 0x0B;
		tx[3] = 0x01;
		if (max32664c_i2c_transmit(dev, tx, 4, &rx, 1, MAX32664C_DEFAULT_CMD_DELAY)) {
			ret = -EINVAL;
			goto out;
		}

		/* Enable Auto PD */
		tx[0] = 0x50;
		tx[1] = 0x07;
		tx[2] = 0x12;
		tx[3] = 0x01;
		if (max32664c_i2c_transmit(dev, tx, 4, &rx, 1, MAX32664C_DEFAULT_CMD_DELAY)) {
			ret = -EINVAL;
			goto out;
		}

		/* Disable SCD - AEC handles skin contact automatically */
		LOG_DBG("Disabling SCD (AEC mode)...");
		tx[0] = 0x50;
		tx[1] = 0x07;
		tx[2] = 0x0C;
		tx[3] = 0x01;
		if (max32664c_i2c_transmit(dev, tx, 4, &rx, 1, MAX32664C_DEFAULT_CMD_DELAY)) {
			ret = -EINVAL;
			goto out;
		}
	} else if (device_mode == MAX32664C_OP_MODE_ALGO_AGC) {
		LOG_DBG("Entering AGC mode...");

		/* TODO: Test if this works */
		/* Set the LED current */
		for (uint8_t i = 0; i < sizeof(data->led_current); i++) {
			tx[0] = 0x40;
			tx[1] = 0x00;
			tx[2] = 0x23 + i;
			tx[3] = data->led_current[i];
			LOG_INF("Set LED%d current: %u", i + 1, data->led_current[i]);
			if (max32664c_i2c_transmit(dev, tx, 4, &rx, 1,
						   MAX32664C_DEFAULT_CMD_DELAY)) {
				LOG_ERR("Can not set LED%d current", i + 1);
				ret = -EINVAL;
				goto out;
			}
		}

		/* Enable AEC */
		tx[0] = 0x50;
		tx[1] = 0x07;
		tx[2] = 0x0B;
		tx[3] = 0x01;
		if (max32664c_i2c_transmit(dev, tx, 4, &rx, 1, MAX32664C_DEFAULT_CMD_DELAY)) {
			ret = -EINVAL;
			goto out;
		}

		/* Disable PD auto current calculation */
		tx[0] = 0x50;
		tx[1] = 0x07;
		tx[2] = 0x12;
		tx[3] = 0x00;
		if (max32664c_i2c_transmit(dev, tx, 4, &rx, 1, MAX32664C_DEFAULT_CMD_DELAY)) {
			ret = -EINVAL;
			goto out;
		}

		/* Enable SCD */
		tx[0] = 0x50;
		tx[1] = 0x07;
		tx[2] = 0x0C;
		tx[3] = 0x01;
		if (max32664c_i2c_transmit(dev, tx, 4, &rx, 1, MAX32664C_DEFAULT_CMD_DELAY)) {
			ret = -EINVAL;
			goto out;
		}

		/* Set AGC target PD current to 10 uA */
		/* TODO: Add setting of PD current via API or DT? */
		tx[0] = 0x50;
		tx[1] = 0x07;
		tx[2] = 0x11;
		tx[3] = 0x00;
		tx[4] = 0x64;
		if (max32664c_i2c_transmit(dev, tx, 5, &rx, 1, MAX32664C_DEFAULT_CMD_DELAY)) {
			ret = -EINVAL;
			goto out;
		}
	} else {
		LOG_ERR("Invalid mode!");
		ret = -EINVAL;
		goto out;
	}

	/* Enable HR and SpO2 algorithm */
	tx[2] = 0x01;
	if (extended) {
		tx[2] = 0x02;
	}

	tx[0] = 0x52;
	tx[1] = 0x07;

	/* Use the maximum time to cover all modes (see Table 6 and 12 in the User Guide) */
	if (max32664c_i2c_transmit(dev, tx, 3, &rx, 1, 500)) {
		ret = -EINVAL;
		goto out;
	}

#ifndef CONFIG_MAX32664C_USE_STATIC_MEMORY
	if (k_msgq_alloc_init(&data->raw_report_queue, sizeof(struct max32664c_raw_report_t),
			      CONFIG_MAX32664C_QUEUE_SIZE)) {
		LOG_ERR("Failed to allocate RAW report queue!");
		ret = -ENOMEM;
		goto out;
	}

	if (!extended && k_msgq_alloc_init(&data->report_queue, sizeof(struct max32664c_report_t),
					   CONFIG_MAX32664C_QUEUE_SIZE)) {
		LOG_ERR("Failed to allocate report queue!");
		ret = -ENOMEM;
		goto out;
	}

#ifdef CONFIG_MAX32664C_USE_EXTENDED_REPORTS
	if (extended &&
	    k_msgq_alloc_init(&data->ext_report_queue, sizeof(struct max32664c_ext_report_t),
			      CONFIG_MAX32664C_QUEUE_SIZE)) {
		LOG_ERR("Failed to allocate extended report queue!");
		ret = -ENOMEM;
		goto out;
	}
#endif /* CONFIG_MAX32664C_USE_EXTENDED_REPORTS */
#endif /* CONFIG_MAX32664C_USE_STATIC_MEMORY */

	k_thread_resume(data->thread_id);

	ret = 0;

out:
	max32664c_mfio_release(dev);
	return ret;
}

/** @brief      Enable the skin contact detection only mode.
 *  @param dev  Pointer to device
 *  @return     0 when successful
 */
static int max32664c_set_mode_scd(const struct device *dev)
{
	uint8_t rx;
	uint8_t tx[4];
	struct max32664c_data *data = dev->data;
	int ret;

	max32664c_mfio_hold(dev);

	/* Stop the current algorithm mode */
	if (max32664c_stop_algo(dev)) {
		LOG_ERR("Failed to stop the algorithm!");
		ret = -EINVAL;
		goto out;
	}

	LOG_DBG("MAX32664C entering SCD mode...");

	/* Set mode early so MFIO stays low during initialization */
	data->op_mode = MAX32664C_OP_MODE_SCD;

	/* Use LED2 for SCD */
	tx[0] = 0xE5;
	tx[1] = 0x02;
	if (max32664c_i2c_transmit(dev, tx, 2, &rx, 1, MAX32664C_DEFAULT_CMD_DELAY)) {
		ret = -EINVAL;
		goto out;
	}

	/* Set the output mode to algorithm data */
	tx[0] = 0x10;
	tx[1] = 0x00;
	tx[2] = MAX32664C_OUT_ALGORITHM_ONLY;
	if (max32664c_i2c_transmit(dev, tx, 3, &rx, 1, MAX32664C_DEFAULT_CMD_DELAY)) {
		ret = -EINVAL;
		goto out;
	}

	/* Enable SCD only algorithm */
	tx[0] = 0x52;
	tx[1] = 0x07;
	tx[2] = 0x03;
	if (max32664c_i2c_transmit(dev, tx, 3, &rx, 1, 500)) {
		ret = -EINVAL;
		goto out;
	}

#ifndef CONFIG_MAX32664C_USE_STATIC_MEMORY
	if (k_msgq_alloc_init(&data->scd_report_queue, sizeof(struct max32664c_scd_report_t),
			      CONFIG_MAX32664C_QUEUE_SIZE)) {
		LOG_ERR("Failed to allocate SCD report queue!");
		ret = -ENOMEM;
		goto out;
	}
#endif /* CONFIG_MAX32664C_USE_STATIC_MEMORY */

	k_thread_resume(data->thread_id);

	ret = 0;

out:
	max32664c_mfio_release(dev);
	return ret;
}

static int max32664c_set_mode_wake_on_motion(const struct device *dev)
{
	uint8_t rx;
	uint8_t tx[6];
	struct max32664c_data *data = dev->data;
	int ret;

	max32664c_mfio_hold(dev);

	LOG_DBG("MAX32664C entering wake on motion mode...");

	/* Set mode early so MFIO stays low during initialization */
	data->op_mode = MAX32664C_OP_MODE_WAKE_ON_MOTION;

	/* Stop the current algorithm */
	tx[0] = 0x52;
	tx[1] = 0x07;
	tx[2] = 0x00;
	if (max32664c_i2c_transmit(dev, tx, 3, &rx, 1, MAX32664C_DEFAULT_CMD_DELAY)) {
		ret = -EINVAL;
		goto out;
	}

	/* Set the motion detection threshold (see Table 12 in the SpO2 and Heart Rate Using Guide)
	 */
	tx[0] = 0x46;
	tx[1] = 0x04;
	tx[2] = 0x00;
	tx[3] = 0x01;
	tx[4] = MAX32664C_MOTION_TIME(data->motion_time);
	tx[5] = MAX32664C_MOTION_THRESHOLD(data->motion_threshold);
	if (max32664c_i2c_transmit(dev, tx, 6, &rx, 1, MAX32664C_DEFAULT_CMD_DELAY)) {
		ret = -EINVAL;
		goto out;
	}

	/* Set the output mode to sensor data */
	tx[0] = 0x10;
	tx[1] = 0x00;
	tx[2] = MAX32664C_OUT_SENSOR_ONLY;
	if (max32664c_i2c_transmit(dev, tx, 3, &rx, 1, MAX32664C_DEFAULT_CMD_DELAY)) {
		ret = -EINVAL;
		goto out;
	}

	/* Enable the accelerometer */
	if (max32664c_acc_enable(dev, true)) {
		ret = -EINVAL;
		goto out;
	}

	ret = 0;

out:
	max32664c_mfio_release(dev);
	return ret;
}

static int max32664c_exit_mode_wake_on_motion(const struct device *dev)
{
	uint8_t rx;
	uint8_t tx[6];
	struct max32664c_data *data = dev->data;

	LOG_DBG("MAX32664C exiting wake on motion mode...");

	/* Exit wake on motion mode */
	tx[0] = 0x46;
	tx[1] = 0x04;
	tx[2] = 0x00;
	tx[3] = 0x00;
	tx[4] = 0xFF;
	tx[5] = 0xFF;
	if (max32664c_i2c_transmit(dev, tx, 6, &rx, 1, MAX32664C_DEFAULT_CMD_DELAY)) {
		return -EINVAL;
	}

	/* Disable the accelerometer */
	if (max32664c_acc_enable(dev, false)) {
		return -EINVAL;
	}

	data->op_mode = MAX32664C_OP_MODE_IDLE;

	return 0;
}

static int max32664c_disable_sensors(const struct device *dev)
{
	uint8_t rx;
	uint8_t tx[4];
	struct max32664c_data *data = dev->data;

	if (max32664c_stop_algo(dev)) {
		LOG_ERR("Failed to stop the algorithm!");
		return -EINVAL;
	}

	/* Leave wake on motion first because we disable the accelerometer */
	if (data->op_mode == MAX32664C_OP_MODE_WAKE_ON_MOTION) {
		if (max32664c_exit_mode_wake_on_motion(dev)) {
			LOG_ERR("Failed to exit wake on motion mode!");
			return -EINVAL;
		}
	}

	LOG_DBG("Disable the sensors...");

	/* Disable the AFE */
	tx[0] = 0x44;
	tx[1] = 0x00;
	tx[2] = 0x00;
	tx[3] = 0x00;
	if (max32664c_i2c_transmit(dev, tx, 4, &rx, 1, 250)) {
		return -EINVAL;
	}

	/* Disable the accelerometer */
	if (max32664c_acc_enable(dev, false)) {
		return -EINVAL;
	}

	data->op_mode = MAX32664C_OP_MODE_IDLE;

	return 0;
}

static int max32664c_sample_fetch(const struct device *dev, enum sensor_channel chan)
{
	int r = max32664c_ensure_ready(dev);
	if (r) {
		return r;
	}

	struct max32664c_data *data = dev->data;
	int ret;

	switch (data->op_mode) {
	case MAX32664C_OP_MODE_STOP_ALGO:
	case MAX32664C_OP_MODE_IDLE:
		LOG_DBG("Device is idle, no data to fetch!");
		return -EAGAIN;
	case MAX32664C_OP_MODE_SCD:
		ret = k_msgq_get(&data->scd_report_queue, &data->scd, K_NO_WAIT);
		return ret;
#ifdef CONFIG_MAX32664C_USE_EXTENDED_REPORTS
	case MAX32664C_OP_MODE_ALGO_AEC_EXT:
	case MAX32664C_OP_MODE_ALGO_AGC_EXT:
		ret = k_msgq_get(&data->ext_report_queue, &data->ext, K_NO_WAIT);
		return ret;
#else
	case MAX32664C_OP_MODE_ALGO_AEC:
	case MAX32664C_OP_MODE_ALGO_AGC:
		ret = k_msgq_get(&data->report_queue, &data->report, K_NO_WAIT);
		return ret;
#endif /* CONFIG_MAX32664C_USE_EXTENDED_REPORTS */
	/* Raw data are reported with normal and extended algorithms so we need to fetch them too */
	case MAX32664C_OP_MODE_RAW:
		ret = k_msgq_get(&data->raw_report_queue, &data->raw, K_NO_WAIT);
		return ret;
	default:
		return -ENOTSUP;
	}
}

static int max32664c_channel_get(const struct device *dev, enum sensor_channel chan,
				 struct sensor_value *val)
{
	struct max32664c_data *data = dev->data;

	switch ((int)chan) {
	case SENSOR_CHAN_ACCEL_X: {
		val->val1 = data->raw.acc.x;
		break;
	}
	case SENSOR_CHAN_ACCEL_Y: {
		val->val1 = data->raw.acc.y;
		break;
	}
	case SENSOR_CHAN_ACCEL_Z: {
		val->val1 = data->raw.acc.z;
		break;
	}
	case SENSOR_CHAN_GREEN: {
		val->val1 = data->raw.PPG1;
		break;
	}
	case SENSOR_CHAN_IR: {
		val->val1 = data->raw.PPG2;
		break;
	}
	case SENSOR_CHAN_RED: {
		val->val1 = data->raw.PPG3;
		break;
	}
	case SENSOR_CHAN_MAX32664C_HEARTRATE: {
		if (IS_ENABLED(CONFIG_MAX32664C_USE_EXTENDED_REPORTS)) {
			val->val1 = data->ext.hr;
			val->val2 = data->ext.hr_confidence;
		} else {
			val->val1 = data->report.hr;
			val->val2 = data->report.hr_confidence;
		}
		break;
	}
	case SENSOR_CHAN_MAX32664C_RESPIRATION_RATE: {
		if (IS_ENABLED(CONFIG_MAX32664C_USE_EXTENDED_REPORTS)) {
			val->val1 = data->ext.rr;
			val->val2 = data->ext.rr_confidence;
		} else {
			val->val1 = data->report.rr;
			val->val2 = data->report.rr_confidence;
		}
		break;
	}
	case SENSOR_CHAN_MAX32664C_BLOOD_OXYGEN_SATURATION: {
		if (IS_ENABLED(CONFIG_MAX32664C_USE_EXTENDED_REPORTS)) {
			val->val1 = data->ext.spo2_meas.value;
			val->val2 = data->ext.spo2_meas.confidence;
		} else {
			val->val1 = data->report.spo2_meas.value;
			val->val2 = data->report.spo2_meas.confidence;
		}
		break;
	}
	case SENSOR_CHAN_MAX32664C_SKIN_CONTACT: {
		if (IS_ENABLED(CONFIG_MAX32664C_USE_EXTENDED_REPORTS)) {
			val->val1 = data->ext.scd_state;
		} else {
			val->val1 = data->report.scd_state;
		}
		val->val2 = 0;
		break;
	}
	default: {
		LOG_ERR("Channel %u not supported!", chan);
		return -ENOTSUP;
	}
	}

	return 0;
}

static int max32664c_ensure_ready(const struct device *dev)
{
	struct max32664c_data *data = dev->data;
	const struct max32664c_config *config = dev->config;

	if (data->hub_ready) {
		return 0;
	}

	/* Rate-limit retries */
	int64_t now = k_uptime_get();
	if ((now - data->last_try_ms) < 500) {
		return -EAGAIN;
	}
	data->last_try_ms = now;

	if (!i2c_is_ready_dt(&config->i2c)) {
		LOG_WRN("MAX32664C bring-up: I2C not ready yet");
		return -EAGAIN;
	}

	k_mutex_lock(&data->lock, K_FOREVER);

	/* check again after lock */
	if (data->hub_ready) {
		k_mutex_unlock(&data->lock);
		return 0;
	}

	LOG_INF("MAX32664C bring-up starting (deferred)");

	/* === Reset -> Application mode sequence ===
	 * RSTN low, MFIO high >=1ms before RSTN high, hold low >=10ms.
	 */
	gpio_pin_set_dt(&config->reset_gpio, true);   /* assert reset (physical LOW) */
	max32664c_mfio_deassert_sleep(dev);           /* MFIO physical HIGH */
	k_msleep(12);                                  /* >=10ms total reset low, MFIO already high */

	gpio_pin_set_dt(&config->reset_gpio, false);  /* deassert reset (physical HIGH) */
	k_msleep(50);
	k_msleep(1850);

	/* Read device mode */
	uint8_t tx[2] = { 0x02, 0x00 };
	uint8_t rx[4] = {0};

	if (max32664c_i2c_transmit(dev, tx, 2, rx, 2, MAX32664C_DEFAULT_CMD_DELAY)) {
		LOG_WRN("MAX32664C bring-up: mode read failed");
		k_mutex_unlock(&data->lock);
		return -EAGAIN;
	}

	data->op_mode = rx[1];
	LOG_INF("MAX32664C mode: 0x%02X", data->op_mode);

	/* Read firmware version (optional) */
	tx[0] = 0xFF;
	tx[1] = 0x03;

	if (max32664c_i2c_transmit(dev, tx, 2, rx, 4, MAX32664C_DEFAULT_CMD_DELAY)) {
		LOG_WRN("MAX32664C bring-up: FW read failed (continuing)");
		data->ext_report_len = MAX32664C_EXT_REPORT_V35_7_1_BYTES;
	} else {
		memcpy(data->hub_ver, &rx[1], 3);
		LOG_INF("MAX32664C FW: %u.%u.%u", data->hub_ver[0], data->hub_ver[1], data->hub_ver[2]);

		if (data->hub_ver[0] == 35) {
			if (data->hub_ver[1] > 7 || (data->hub_ver[1] == 7 && data->hub_ver[2] >= 1)) {
				data->ext_report_len = MAX32664C_EXT_REPORT_V35_7_1_BYTES;
			} else {
				data->ext_report_len = MAX32664C_EXT_REPORT_LEGACY_BYTES;
			}
		} else {
			data->ext_report_len = MAX32664C_EXT_REPORT_V35_7_1_BYTES;
		}
	}

	/* Check sensors + write default config + create worker thread */
	if (max32664c_check_sensors(dev) || max32664c_init_hub(dev)) {
		LOG_WRN("MAX32664C bring-up: hub init failed");
		k_mutex_unlock(&data->lock);
		return -EAGAIN;
	}

	data->hub_ready = true;
	LOG_INF("MAX32664C bring-up complete");

	k_mutex_unlock(&data->lock);
	return 0;
}

static int max32664c_attr_set(const struct device *dev, enum sensor_channel chan,
			      enum sensor_attribute attr, const struct sensor_value *val)
{
	int r = max32664c_ensure_ready(dev);
	if (r) {
		return r; /* -EAGAIN until rails are up and bring-up succeeds */
	}

	int err;
	uint8_t tx[5];
	uint8_t rx;
	struct max32664c_data *data = dev->data;

	err = 0;

	switch ((int)attr) {
	case SENSOR_ATTR_SAMPLING_FREQUENCY: {
		break;
	}
	case SENSOR_ATTR_MAX32664C_HEIGHT: {
		tx[0] = 0x50;
		tx[1] = 0x07;
		tx[2] = 0x06;
		tx[3] = (val->val1 & 0xFF00) >> 8;
		tx[4] = val->val1 & 0x00FF;
		if (max32664c_i2c_transmit(dev, tx, 5, &rx, 1, MAX32664C_DEFAULT_CMD_DELAY)) {
			LOG_ERR("Can not set height!");
			return -EINVAL;
		}

		break;
	}
	case SENSOR_ATTR_MAX32664C_WEIGHT: {
		tx[0] = 0x50;
		tx[1] = 0x07;
		tx[2] = 0x07;
		tx[3] = (val->val1 & 0xFF00) >> 8;
		tx[4] = val->val1 & 0x00FF;
		if (max32664c_i2c_transmit(dev, tx, 5, &rx, 1, MAX32664C_DEFAULT_CMD_DELAY)) {
			LOG_ERR("Can not set weight!");
			return -EINVAL;
		}

		break;
	}
	case SENSOR_ATTR_MAX32664C_AGE: {
		tx[0] = 0x50;
		tx[1] = 0x07;
		tx[2] = 0x08;
		tx[3] = val->val1 & 0x00FF;
		if (max32664c_i2c_transmit(dev, tx, 4, &rx, 1, MAX32664C_DEFAULT_CMD_DELAY)) {
			LOG_ERR("Can not set age!");
			return -EINVAL;
		}

		break;
	}
	case SENSOR_ATTR_MAX32664C_GENDER: {
		tx[0] = 0x50;
		tx[1] = 0x07;
		tx[2] = 0x08;
		tx[3] = val->val1 & 0x00FF;
		if (max32664c_i2c_transmit(dev, tx, 4, &rx, 1, MAX32664C_DEFAULT_CMD_DELAY)) {
			LOG_ERR("Can not set gender!");
			return -EINVAL;
		}

		break;
	}
	case SENSOR_ATTR_SLOPE_DUR: {
		data->motion_time = val->val1;
		break;
	}
	case SENSOR_ATTR_SLOPE_TH: {
		data->motion_threshold = val->val1;
		break;
	}
	case SENSOR_ATTR_CONFIGURATION: {
		switch ((int)chan) {
		case SENSOR_CHAN_GREEN: {
			data->led_current[0] = val->val1 & 0xFF;
			break;
		}
		case SENSOR_CHAN_IR: {
			data->led_current[1] = val->val1 & 0xFF;
			break;
		}
		case SENSOR_CHAN_RED: {
			data->led_current[2] = val->val1 & 0xFF;
			break;
		}
		default: {
			LOG_ERR("Channel %u not supported for setting attribute!", (int)chan);
			return -ENOTSUP;
		}
		}
		break;
	}
	case SENSOR_ATTR_MAX32664C_OP_MODE: {
		switch (val->val1) {
		case MAX32664C_OP_MODE_ALGO_AEC: {
#ifndef CONFIG_MAX32664C_USE_EXTENDED_REPORTS
			err = max32664c_set_mode_algo(dev, MAX32664C_OP_MODE_ALGO_AEC, val->val2,
						      false);
#else
			return -EINVAL;
#endif /* CONFIG_MAX32664C_USE_EXTENDED_REPORTS */
			break;
		}
		case MAX32664C_OP_MODE_ALGO_AEC_EXT: {
#ifdef CONFIG_MAX32664C_USE_EXTENDED_REPORTS
			err = max32664c_set_mode_algo(dev, MAX32664C_OP_MODE_ALGO_AEC, val->val2,
						      true);
#else
			return -EINVAL;
#endif /* CONFIG_MAX32664C_USE_EXTENDED_REPORTS */
			break;
		}
		case MAX32664C_OP_MODE_ALGO_AGC: {
#ifndef CONFIG_MAX32664C_USE_EXTENDED_REPORTS
			err = max32664c_set_mode_algo(dev, MAX32664C_OP_MODE_ALGO_AGC, val->val2,
						      false);
#else
			return -EINVAL;
#endif /* CONFIG_MAX32664C_USE_EXTENDED_REPORTS */
			break;
		}
		case MAX32664C_OP_MODE_ALGO_AGC_EXT: {
#ifdef CONFIG_MAX32664C_USE_EXTENDED_REPORTS
			err = max32664c_set_mode_algo(dev, MAX32664C_OP_MODE_ALGO_AGC, val->val2,
						      true);
#else
			return -EINVAL;
#endif /* CONFIG_MAX32664C_USE_EXTENDED_REPORTS */
			break;
		}
		case MAX32664C_OP_MODE_RAW: {
			err = max32664c_set_mode_raw(dev);
			break;
		}
		case MAX32664C_OP_MODE_SCD: {
			err = max32664c_set_mode_scd(dev);
			break;
		}
		case MAX32664C_OP_MODE_WAKE_ON_MOTION: {
			err = max32664c_set_mode_wake_on_motion(dev);
			break;
		}
		case MAX32664C_OP_MODE_EXIT_WAKE_ON_MOTION: {
			err = max32664c_exit_mode_wake_on_motion(dev);
			break;
		}
		case MAX32664C_OP_MODE_STOP_ALGO: {
			err = max32664c_stop_algo(dev);
			break;
		}
		case MAX32664C_OP_MODE_IDLE: {
			err = max32664c_disable_sensors(dev);
			break;
		}
		default: {
			LOG_ERR("Unsupported sensor operation mode");
			return -ENOTSUP;
		}
		}

		break;
	}
	default: {
		LOG_ERR("Unsupported sensor attribute!");
		return -ENOTSUP;
	}
	}

	return err;
}

static int max32664c_attr_get(const struct device *dev, enum sensor_channel chan,
			      enum sensor_attribute attr, struct sensor_value *val)
{
	int r = max32664c_ensure_ready(dev);
	if (r) {
		return r;
	}

	struct max32664c_data *data = dev->data;

	switch ((int)attr) {
	case SENSOR_ATTR_MAX32664C_OP_MODE: {
		val->val1 = data->op_mode;
		val->val2 = 0;
		break;
	}
	case SENSOR_ATTR_MAX32664C_AFE_ID: {
		val->val1 = data->afe_id;
		val->val2 = 0;
		break;
	}
	case SENSOR_ATTR_MAX32664C_ACCEL_ID: {
		val->val1 = data->accel_id;
		val->val2 = 0;
		break;
	}
	case SENSOR_ATTR_CONFIGURATION: {
		switch ((int)chan) {
		case SENSOR_CHAN_GREEN: {
			val->val1 = data->led_current[0];
			break;
		}
		case SENSOR_CHAN_IR: {
			val->val1 = data->led_current[1];
			break;
		}
		case SENSOR_CHAN_RED: {
			val->val1 = data->led_current[2];
			break;
		}
		default: {
			LOG_ERR("Channel %u not supported for getting attribute!", (int)chan);
			return -ENOTSUP;
		}
		}
		break;
	}
	default: {
		LOG_ERR("Unsupported sensor attribute!");
		return -ENOTSUP;
	}
	}

	return 0;
}

static DEVICE_API(sensor, max32664c_driver_api) = {
	.attr_set = max32664c_attr_set,
	.attr_get = max32664c_attr_get,
	.sample_fetch = max32664c_sample_fetch,
	.channel_get = max32664c_channel_get,
};

static int max32664c_init(const struct device *dev)
{
	const struct max32664c_config *config = dev->config;
	struct max32664c_data *data = dev->data;

	/* Init deferred state */
	k_mutex_init(&data->lock);
	data->hub_ready = false;
	data->last_try_ms = 0;
	data->op_mode = MAX32664C_OP_MODE_IDLE;

	if (!i2c_is_ready_dt(&config->i2c)) {
		LOG_ERR("MAX32664C: I2C not ready at boot (deferred init will retry)");
		/* IMPORTANT: return 0 so device remains 'ready' from Zephyr's POV */
	}

	/**
	 * Configure Reset Pin: Standard Push-Pull Output
	 * Initial state: Active (Logic 1) -> Physical Low (Reset held)
	 */
	gpio_pin_configure_dt(&config->reset_gpio, GPIO_OUTPUT_ACTIVE);

	/**
	 * Configure MFIO: Bidirectional Open-Drain
	 * 1. GPIO_INPUT: Enable input buffer (good practice for bi-dir lines)
	 * 2. GPIO_OUTPUT_INACTIVE: Start with Logic 0 -> Physical High (Hi-Z) due to ACTIVE_LOW
	 * 3. GPIO_OPEN_DRAIN: Enforce open-drain (redundant if in DT, but safe)
	 */
	gpio_pin_configure_dt(&config->mfio_gpio, 
	                      GPIO_INPUT | GPIO_OUTPUT_INACTIVE | GPIO_OPEN_DRAIN);

	LOG_INF("MAX32664C: Init complete (held in reset)");

	return 0; /* never brick readiness */
}

#ifdef CONFIG_PM_DEVICE
static int max32664c_pm_action(const struct device *dev, enum pm_device_action action)
{
	switch (action) {
	case PM_DEVICE_ACTION_RESUME: {
		break;
	}
	case PM_DEVICE_ACTION_SUSPEND: {
		/* Pulling MFIO high will cause the hub to enter sleep mode */
		max32664c_mfio_deassert_sleep(dev);
		k_msleep(20);
		break;
	}
	case PM_DEVICE_ACTION_TURN_OFF: {
		uint8_t rx;
		uint8_t tx[3];

		/* Send a shut down command */
		/* NOTE: Toggling RSTN is needed to wake the device */
		tx[0] = 0x01;
		tx[1] = 0x00;
		tx[2] = 0x01;
		if (max32664c_i2c_transmit(dev, tx, 3, &rx, 1, MAX32664C_DEFAULT_CMD_DELAY)) {
			return -EINVAL;
		}
		break;
	}
	case PM_DEVICE_ACTION_TURN_ON: {
		/* Toggling RSTN is needed to turn the device on */
		max32664c_init(dev);
		break;
	}
	default: {
		return -ENOTSUP;
	}
	}

	return 0;
}
#endif /* CONFIG_PM_DEVICE */

#define MAX32664C_INIT(inst)                                                                       \
	static struct max32664c_data max32664c_data_##inst;                                        \
                                                                                                   \
	static const struct max32664c_config max32664c_config_##inst = {                           \
		.i2c = I2C_DT_SPEC_INST_GET(inst),                                                 \
		.reset_gpio = GPIO_DT_SPEC_INST_GET(inst, reset_gpios),                            \
		.mfio_gpio = GPIO_DT_SPEC_INST_GET(inst, mfio_gpios),                              \
		.spo2_calib = DT_INST_PROP(inst, spo2_calib),                                      \
		.hr_config = DT_INST_PROP(inst, hr_config),                                        \
		.spo2_config = DT_INST_PROP(inst, spo2_config),                                    \
		.use_max86141 = DT_INST_PROP(inst, use_max86141),                                  \
		.use_max86161 = DT_INST_PROP(inst, use_max86161),                                  \
		.use_max86174 = DT_INST_PROP_OR(inst, use_max86174, 0),                            \
		.use_sample_counter_byte = DT_INST_PROP_OR(inst, use_sample_counter_byte, 0),      \
		.motion_time = DT_INST_PROP(inst, motion_time),                                    \
		.motion_threshold = DT_INST_PROP(inst, motion_threshold),                          \
		.min_integration_time_idx = DT_INST_ENUM_IDX(inst, min_integration_time),          \
		.min_sampling_rate_idx = DT_INST_ENUM_IDX(inst, min_sampling_rate),                \
		.max_integration_time_idx = DT_INST_ENUM_IDX(inst, max_integration_time),          \
		.max_sampling_rate_idx = DT_INST_ENUM_IDX(inst, max_sampling_rate),                \
		.report_period = DT_INST_PROP(inst, report_period),                                \
		.led_current = DT_INST_PROP(inst, led_current),                                    \
	};                                                                                         \
                                                                                                   \
	PM_DEVICE_DT_INST_DEFINE(inst, max32664c_pm_action);                                       \
                                                                                                   \
	SENSOR_DEVICE_DT_INST_DEFINE(inst, max32664c_init, PM_DEVICE_DT_INST_GET(inst),            \
				     &max32664c_data_##inst, &max32664c_config_##inst,             \
				     POST_KERNEL, CONFIG_SENSOR_INIT_PRIORITY,                     \
				     &max32664c_driver_api)

DT_INST_FOREACH_STATUS_OKAY(MAX32664C_INIT)