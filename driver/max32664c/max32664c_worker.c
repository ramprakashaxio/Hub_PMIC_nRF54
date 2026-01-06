/*
 * Background worker for the MAX32664C biometric sensor hub.
 *
 * Copyright (c) 2025, Daniel Kampert
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/sys/byteorder.h>

#include "max32664c.h"

LOG_MODULE_REGISTER(maxim_max32664c_worker, CONFIG_SENSOR_LOG_LEVEL);

/* On-wire protocol byte sizes (NOT sizeof(struct) due to padding) */
#define MAX32664C_RAW_BYTES        24U

#ifdef CONFIG_MAX32664C_USE_EXTENDED_REPORTS
#define MAX32664C_ALGO_BYTES       56U
#else
#define MAX32664C_ALGO_BYTES       20U
#endif

#define MAX32664C_SAMPLE_BYTES     (MAX32664C_RAW_BYTES + MAX32664C_ALGO_BYTES)
#define MAX32664C_READ_LEN(fifo)   (1U + ((uint32_t)(fifo) * MAX32664C_SAMPLE_BYTES))

/** @brief              Read the hub status from the sensor hub.
 *                      NOTE: Table 7 Sensor Hub Status Byte
 *  @param dev          Pointer to device
 *  @param hub_status   Pointer to hub status byte (DRDY, SCD, etc.)
 *  @return             0 when successful, otherwise an error code
 */
static int max32664c_get_hub_status(const struct device *dev, uint8_t *hub_status)
{
	uint8_t tx[2] = {0x00, 0x00};
	uint8_t rx[2];

	if (max32664c_i2c_transmit(dev, tx, sizeof(tx), rx, sizeof(rx),
				   MAX32664C_DEFAULT_CMD_DELAY)) {
		return -EIO;
	}

	/* rx[0] = cmd status (0x00 = success), rx[1] = hub status byte (DRDY/SCD bits) */
	*hub_status = rx[1];

	return 0;
}

/** @brief      Read the FIFO sample count.
 *  @param dev  Pointer to device
 *  @param fifo Pointer to FIFO count
 */
static int max32664c_get_fifo_count(const struct device *dev, uint8_t *fifo)
{
	uint8_t tx[2] = {0x12, 0x00};
	uint8_t rx[2];

	if (max32664c_i2c_transmit(dev, tx, sizeof(tx), rx, sizeof(rx),
				   MAX32664C_DEFAULT_CMD_DELAY)) {
		return -EINVAL;
	}

	*fifo = rx[1];

	return rx[0];
}

/** @brief      Push a item into the message queue.
 *  @param msgq Pointer to message queue
 *  @param data Pointer to data to push
 *  @return     0 on success, error code on failure
 */
static int max32664c_push_to_queue(struct k_msgq *msgq, const void *data)
{
	int ret = k_msgq_put(msgq, data, K_NO_WAIT);
	if (ret == 0) {
		return 0;
	}

	/* Only treat "queue full" as recoverable */
	if (ret == -EAGAIN || ret == -ENOMSG) {
		k_msgq_purge(msgq);
		ret = k_msgq_put(msgq, data, K_NO_WAIT);
		if (ret == 0) {
			return 0;
		}
	}

	return ret; /* -EMSGSIZE / -EINVAL etc */
}

/** @brief          Process the buffer to get the raw data from the sensor hub.
 *  @param dev      Pointer to device
 */
static void max32664c_parse_and_push_raw(const struct device *dev)
{
	struct max32664c_data *data = dev->data;
	struct max32664c_raw_report_t report;

	report.PPG1 = ((uint32_t)(data->max32664_i2c_buffer[1]) << 16) |
		       ((uint32_t)(data->max32664_i2c_buffer[2]) << 8) |
		       data->max32664_i2c_buffer[3];
	report.PPG2 = ((uint32_t)(data->max32664_i2c_buffer[4]) << 16) |
		       ((uint32_t)(data->max32664_i2c_buffer[5]) << 8) |
		       data->max32664_i2c_buffer[6];
	report.PPG3 = ((uint32_t)(data->max32664_i2c_buffer[7]) << 16) |
		       ((uint32_t)(data->max32664_i2c_buffer[8]) << 8) |
		       data->max32664_i2c_buffer[9];

	/* PPG4 to 6 are used for PD2 */
	report.PPG4 = 0;
	report.PPG5 = 0;
	report.PPG6 = 0;

	report.acc.x =
		((int16_t)(data->max32664_i2c_buffer[19]) << 8) | data->max32664_i2c_buffer[20];
	report.acc.y =
		((int16_t)(data->max32664_i2c_buffer[21]) << 8) | data->max32664_i2c_buffer[22];
	report.acc.z =
		((int16_t)(data->max32664_i2c_buffer[23]) << 8) | data->max32664_i2c_buffer[24];

	int ret = max32664c_push_to_queue(&data->raw_report_queue, &report);
	if (ret) {
		LOG_ERR("raw_report_queue put failed: %d (queue item size mismatch?)", ret);
	}
}

#ifdef CONFIG_MAX32664C_USE_EXTENDED_REPORTS
/** @brief          Process the buffer to get the extended report data from the sensor hub.
 *  @param dev      Pointer to device
 */
static void max32664c_parse_and_push_ext_report(const struct device *dev)
{
	struct max32664c_data *data = dev->data;
	struct max32664c_ext_report_t report;

	report.op_mode = data->max32664_i2c_buffer[25];
	report.hr =
		(((uint16_t)(data->max32664_i2c_buffer[26]) << 8) | data->max32664_i2c_buffer[27]) /
		10;
	report.hr_confidence = data->max32664_i2c_buffer[28];
	report.rr =
		(((uint16_t)(data->max32664_i2c_buffer[29]) << 8) | data->max32664_i2c_buffer[30]) /
		10;
	report.rr_confidence = data->max32664_i2c_buffer[31];
	report.activity_class = data->max32664_i2c_buffer[32];
	
	/* FIX: The sensor sends 32-bit data in Big Endian (Network Order).
	 * We must use sys_get_be32 to interpret 0x00000007 correctly as 7,
	 * otherwise it becomes 0x07000000 (117 million).
	 */
	report.total_walk_steps  = sys_get_be32(&data->max32664_i2c_buffer[33]);
	report.total_run_steps   = sys_get_be32(&data->max32664_i2c_buffer[37]);
	report.total_energy_kcal = sys_get_be32(&data->max32664_i2c_buffer[41]);
	report.total_amr_kcal    = sys_get_be32(&data->max32664_i2c_buffer[45]);
	report.led_current_adj1.adj_flag = data->max32664_i2c_buffer[49];
	report.led_current_adj1.adj_val =
		(((uint16_t)(data->max32664_i2c_buffer[50]) << 8) | data->max32664_i2c_buffer[51]) /
		10;
	report.led_current_adj2.adj_flag = data->max32664_i2c_buffer[52];
	report.led_current_adj2.adj_val =
		(((uint16_t)(data->max32664_i2c_buffer[53]) << 8) | data->max32664_i2c_buffer[54]) /
		10;
	report.led_current_adj3.adj_flag = data->max32664_i2c_buffer[55];
	report.led_current_adj3.adj_val =
		(((uint16_t)(data->max32664_i2c_buffer[56]) << 8) | data->max32664_i2c_buffer[57]) /
		10;
	report.integration_time_adj_flag = data->max32664_i2c_buffer[58];
	report.requested_integration_time = data->max32664_i2c_buffer[59];
	report.sampling_rate_adj_flag = data->max32664_i2c_buffer[60];
	report.requested_sampling_rate = data->max32664_i2c_buffer[61];
	report.requested_sampling_average = data->max32664_i2c_buffer[62];
	report.hrm_afe_ctrl_state = data->max32664_i2c_buffer[63];
	report.is_high_motion_for_hrm = data->max32664_i2c_buffer[64];
	report.scd_state = data->max32664_i2c_buffer[65];
	
	/* FIX 1: Read raw integers (Do NOT divide yet to avoid truncation) */
	report.r_value = sys_get_be16(&data->max32664_i2c_buffer[66]);
	report.spo2_meas.confidence = data->max32664_i2c_buffer[68];
	report.spo2_meas.value = sys_get_be16(&data->max32664_i2c_buffer[69]);
	report.spo2_meas.valid_percent = data->max32664_i2c_buffer[71];
	report.spo2_meas.low_signal_flag = data->max32664_i2c_buffer[72];
	report.spo2_meas.motion_flag = data->max32664_i2c_buffer[73];
	report.spo2_meas.low_pi_flag = data->max32664_i2c_buffer[74];
	report.spo2_meas.unreliable_r_flag = data->max32664_i2c_buffer[75];
	report.spo2_meas.state = data->max32664_i2c_buffer[76];
	report.ibi_offset = data->max32664_i2c_buffer[77];
	report.unreliable_orientation_flag = data->max32664_i2c_buffer[78];
	report.reserved[0] = data->max32664_i2c_buffer[79];
	report.reserved[1] = data->max32664_i2c_buffer[80];

	/* ========================================================================= */
	/* DEBUG: PRINT FULL EXTENDED REPORT (Datasheet Table 8)                     */
	/* ========================================================================= */
	LOG_INF("=== EXTENDED ALGO REPORT ===");
	LOG_INF("Mode: %d | SCD: %d (%s)", 
	        report.op_mode, 
	        report.scd_state,
	        (report.scd_state == 3) ? "ON SKIN" : 
	        (report.scd_state == 0) ? "UNDETECTED" : "NO CONTACT");

	LOG_INF("HR: %d bpm (Conf: %d%%) | R-to-R: %d ms (Conf: %d%%)",
	        report.hr, report.hr_confidence,
	        report.rr, report.rr_confidence); /* Note: Struct 'rr' is actually R-to-R interval */

	/* FIX 2: Convert to float ONLY for logging (avoid integer truncation in display) */
	float spo2_f = (float)report.spo2_meas.value / 10.0f;
	float r_val_f = (float)report.r_value / 1000.0f;

	/* FIX 3: Print clearly with proper float formatting */
	LOG_INF("SpO2: %.1f%% (Conf: %d%%) | R-Val: %.3f | State: %d",
	        (double)spo2_f,
	        report.spo2_meas.confidence,
	        (double)r_val_f,
	        report.spo2_meas.state);

	LOG_INF("Activity: %d | Walk: %d | Run: %d",
	        report.activity_class,
	        report.total_walk_steps,
	        report.total_run_steps);

	LOG_INF("Energy: %d.%d kcal | AMR: %d.%d kcal",
	        report.total_energy_kcal / 10, report.total_energy_kcal % 10,
	        report.total_amr_kcal / 10, report.total_amr_kcal % 10);

	LOG_INF("Motion Flag: %d | Low Signal: %d", 
	        report.spo2_meas.motion_flag, 
	        report.spo2_meas.low_signal_flag);
	/* ========================================================================= */

	int ret = max32664c_push_to_queue(&data->ext_report_queue, &report);
	if (ret) {
		LOG_ERR("ext_report_queue put failed: %d (queue item size mismatch?)", ret);
	}
}
#else
/** @brief          Process the buffer to get the report data from the sensor hub.
 *  @param dev      Pointer to device
 */
static void max32664c_parse_and_push_report(const struct device *dev)
{
	struct max32664c_data *data = dev->data;
	struct max32664c_report_t report;

	report.op_mode = data->max32664_i2c_buffer[25];
	report.hr =
		(((uint16_t)(data->max32664_i2c_buffer[26]) << 8) | data->max32664_i2c_buffer[27]) /
		10;
	report.hr_confidence = data->max32664_i2c_buffer[28];
	report.rr =
		(((uint16_t)(data->max32664_i2c_buffer[29]) << 8) | data->max32664_i2c_buffer[30]) /
		10;
	report.rr_confidence = data->max32664_i2c_buffer[31];
	report.activity_class = data->max32664_i2c_buffer[32];
	report.r =
		(((uint16_t)(data->max32664_i2c_buffer[33]) << 8) | data->max32664_i2c_buffer[34]) /
		1000;
	report.spo2_meas.confidence = data->max32664_i2c_buffer[35];
	report.spo2_meas.value =
		(((uint16_t)(data->max32664_i2c_buffer[36]) << 8) | data->max32664_i2c_buffer[37]) /
		10;
	report.spo2_meas.complete = data->max32664_i2c_buffer[38];
	report.spo2_meas.low_signal_quality = data->max32664_i2c_buffer[39];
	report.spo2_meas.motion = data->max32664_i2c_buffer[40];
	report.spo2_meas.low_pi = data->max32664_i2c_buffer[41];
	report.spo2_meas.unreliable_r = data->max32664_i2c_buffer[42];
	report.spo2_meas.state = data->max32664_i2c_buffer[43];
	report.scd_state = data->max32664_i2c_buffer[44];

	int ret = max32664c_push_to_queue(&data->report_queue, &report);
	if (ret) {
		LOG_ERR("report_queue put failed: %d (queue item size mismatch?)", ret);
	}
}
#endif /* CONFIG_MAX32664C_USE_EXTENDED_REPORTS */

/** @brief      Worker thread to read the sensor hub.
 *              This thread does the following:
 *                  - It polls the sensor hub periodically for new results
 *                  - If new messages are available it reads the number of samples
 *                  - Then it reads all the samples to clear the FIFO.
 *                    It's necessary to clear the complete FIFO because the sensor hub
 *                    doesn´t support the reading of a single message and not clearing
 *                    the FIFO can cause a FIFO overrun.
 *                  - Extract the message data from the FIRST item from the FIFO and
 *                    copy them into the right message structure
 *                  - Put the message into a message queue
 *  @param dev  Pointer to device
 */
void max32664c_worker(const struct device *dev)
{
	int err;
	uint8_t fifo = 0;
	struct max32664c_data *data = dev->data;
	int retries = 0;

	LOG_INF("===== WORKER THREAD ALIVE ===== Device: %s", dev->name);

	while (data->is_thread_running) {
		
		/* 1. Check FIFO count with simple retry */
		err = max32664c_get_fifo_count(dev, &fifo);
		if (err) {
			retries++;
			if (retries > 5) {
				LOG_WRN("FIFO check failed (%d)", err);
				k_msleep(200);
				retries = 0;
			} else {
				k_msleep(10);
			}
			continue;
		}
		retries = 0;

		if (fifo == 0) {
			k_msleep(40);
			continue;
		}

		/* 2. Read the entire FIFO content */
		/* Note: We read all samples to clear the sensor's buffer */
		uint32_t read_len = MAX32664C_READ_LEN(fifo);

#ifdef CONFIG_MAX32664C_USE_STATIC_MEMORY
		if (fifo > CONFIG_MAX32664C_SAMPLE_BUFFER_SIZE) {
			LOG_WRN("FIFO count %u exceeds buffer size %u, capping",
				fifo, CONFIG_MAX32664C_SAMPLE_BUFFER_SIZE);
			fifo = CONFIG_MAX32664C_SAMPLE_BUFFER_SIZE;
			read_len = MAX32664C_READ_LEN(fifo);
		}
#else
		data->max32664_i2c_buffer = (uint8_t *)k_malloc(read_len);
		if (data->max32664_i2c_buffer == NULL) {
			LOG_ERR("OOM: Failed to allocate %u bytes!", read_len);
			k_msleep(100);
			continue;
		}
#endif

		uint8_t tx[2] = {0x12, 0x01};

		switch (data->op_mode) {
		case MAX32664C_OP_MODE_RAW:
		case MAX32664C_OP_MODE_ALGO_AEC:
		case MAX32664C_OP_MODE_ALGO_AGC:
#ifdef CONFIG_MAX32664C_USE_EXTENDED_REPORTS
		case MAX32664C_OP_MODE_ALGO_AEC_EXT:
		case MAX32664C_OP_MODE_ALGO_AGC_EXT:
#endif
		{
			if (max32664c_i2c_transmit(dev, tx, 2, data->max32664_i2c_buffer, read_len,
						   MAX32664C_DEFAULT_CMD_DELAY) == 0) {
				
				/* Check Status Byte (Byte 0) */
				if (data->max32664_i2c_buffer[0] == 0) {
					
					/* FIX: Parse ONLY the LAST sample (Freshest Data) */
					/* Offset = 1 (status) + (last_index * sample_size) */
					uint8_t last_idx = (fifo > 0) ? (fifo - 1) : 0;
					uint8_t *sample_ptr = &data->max32664_i2c_buffer[1 + ((uint32_t)last_idx * MAX32664C_SAMPLE_BYTES)];

					/* 1. Push RAW Data (if enabled or in raw mode) */
					/* Note: sample_ptr points to the start of the sample (Raw block) */
					if (data->op_mode == MAX32664C_OP_MODE_RAW) {
						max32664c_parse_and_push_raw(dev);
					}

					/* 2. Push Algo Report */
					if (data->op_mode != MAX32664C_OP_MODE_RAW) {
#ifdef CONFIG_MAX32664C_USE_EXTENDED_REPORTS
						max32664c_parse_and_push_ext_report(dev);
#else
						max32664c_parse_and_push_report(dev);
#endif
						LOG_INF("ALGO: Processed sample %d/%d", last_idx + 1, fifo);
					}
				}
			}
			break;
		}
		default:
			break;
		}
		
#ifndef CONFIG_MAX32664C_USE_STATIC_MEMORY
		k_free(data->max32664_i2c_buffer);
#endif

		k_msleep(40);
	}
}