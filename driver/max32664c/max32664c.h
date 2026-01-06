/*
 * Copyright (c) 2025, Daniel Kampert
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/pm/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>
#include <zephyr/kernel.h>

#define MAX32664C_BIT_STATUS_NO_ERR   1
#define MAX32664C_BIT_STATUS_DATA_RDY 3
#define MAX32664C_BIT_STATUS_OUT_OVFL 4
#define MAX32664C_BIT_STATUS_IN_OVFL  5
#define MAX32664C_BIT_STATUS_BUSY     6

/* Hub status byte masks */
#define MAX32664C_HUB_STAT_DRDY_MASK  (1 << MAX32664C_BIT_STATUS_DATA_RDY)  /* 0x08 */
#define MAX32664C_HUB_STAT_SCD_MASK   0x80  /* Skin Contact Detection bit 7 */

#define MAX32664C_DEFAULT_CMD_DELAY 10

/* AFE IDs */
#define MAX86141_AFE_ID 0x25
#define MAX86161_AFE_ID 0x36
#define MAX86174_AFE_ID 0x40

/* MAX86174 FW35 register configuration constants */
#define MAX32664C_FAM_AFE_WR     0x50
#define MAX32664C_FAM_AFE_RD     0x51
#define MAX32664C_IDX_AFE_CFG    0x07
#define MAX32664C_SUB_ITIME      0x1A
#define MAX32664C_SUB_SPSAVG     0x1B
#define MAX86174_MEAS_SLOTS      6  /* FW35: only first 6 slots are valid AEC measurement slots */

/* Status byte values from MAX32664C User Guide */
#define MAX32664C_ST_SUCCESS        0x00
#define MAX32664C_ST_ILLEGAL_CMD    0x01
#define MAX32664C_ST_UNIMPL         0x02
#define MAX32664C_ST_BAD_LEN        0x03
#define MAX32664C_ST_BAD_VALUE      0x04
#define MAX32664C_ST_BUSY_BOOT      0x05
#define MAX32664C_ST_BUSY           0xFE
#define MAX32664C_ST_UNKNOWN        0xFF

/* Report Sizes */
#define MAX32664C_SENSOR_FRAME_25HZ_BYTES           24
#define MAX32664C_SENSOR_FRAME_25HZ_COUNTER_BYTES   25
#define MAX32664C_SENSOR_FRAME_200HZ_BYTES          150 /* 8 samples * 18 bytes + 6 bytes accel */
#define MAX32664C_ALGO_REPORT_BYTES                 20
#define MAX32664C_EXT_REPORT_LEGACY_BYTES           52
#define MAX32664C_EXT_REPORT_V35_7_1_BYTES          56

/** @brief Output formats of the sensor hub.
 */
enum max32664c_output_format {
	MAX32664C_OUT_PAUSE,
	MAX32664C_OUT_SENSOR_ONLY,
	MAX32664C_OUT_ALGORITHM_ONLY,
	MAX32664C_OUT_ALGO_AND_SENSOR,
};

/** @brief Operating modes of the MAX32664C sensor hub.
 */
enum max32664c_device_mode {
	MAX32664C_OP_MODE_IDLE,
	MAX32664C_OP_MODE_RAW,
	MAX32664C_OP_MODE_ALGO_AEC,
	MAX32664C_OP_MODE_ALGO_AEC_EXT,
	MAX32664C_OP_MODE_ALGO_AGC,
	MAX32664C_OP_MODE_ALGO_AGC_EXT,
	
	/* Unsupported in FW 35.x.y - kept for legacy compatibility only */
	MAX32664C_OP_MODE_SCD, 
	MAX32664C_OP_MODE_WAKE_ON_MOTION,
	MAX32664C_OP_MODE_EXIT_WAKE_ON_MOTION,

	MAX32664C_OP_MODE_STOP_ALGO,
};

/** @brief Algorithm modes for the MAX32664C sensor hub.
 */
enum max32664c_algo_mode {
	MAX32664C_ALGO_MODE_CONT_HR_CONT_SPO2         = 0x00,
	MAX32664C_ALGO_MODE_CONT_HR_ONESHOT_SPO2      = 0x01,
	MAX32664C_ALGO_MODE_CONT_HR                   = 0x02,
	
	/* FW 35.x.y Specific Modes */
	MAX32664C_ALGO_MODE_ONESHOT_HR                = 0x03, /* Measure then sleep */
	MAX32664C_ALGO_MODE_ONESHOT_HR_ONESHOT_SPO2   = 0x04, /* Measure then sleep */
	MAX32664C_ALGO_MODE_ACTIVITY_TRACKING_ONLY    = 0x05, /* Accel only, no LEDs */
	MAX32664C_ALGO_MODE_SPO2_CALIBRATION          = 0x06, 
	MAX32664C_ALGO_MODE_CONT_HR_FAST_SPO2         = 0x07  /* Faster lock time */
};

/* Sensor-specific channel definitions */
enum sensor_channel_max32664c {
	SENSOR_CHAN_MAX32664C_HEARTRATE = SENSOR_CHAN_PRIV_START,
	SENSOR_CHAN_MAX32664C_RESPIRATION_RATE,
	SENSOR_CHAN_MAX32664C_BLOOD_OXYGEN_SATURATION,
	SENSOR_CHAN_MAX32664C_SKIN_CONTACT,
};

/* Sensor-specific attribute definitions */
enum sensor_attribute_max32664c {
	SENSOR_ATTR_MAX32664C_OP_MODE = SENSOR_ATTR_PRIV_START,
	SENSOR_ATTR_MAX32664C_HEIGHT,
	SENSOR_ATTR_MAX32664C_WEIGHT,
	SENSOR_ATTR_MAX32664C_AGE,
	SENSOR_ATTR_MAX32664C_GENDER,
	SENSOR_ATTR_MAX32664C_AFE_ID,
	SENSOR_ATTR_MAX32664C_ACCEL_ID,
};

/* Motion detection macros */
#define MAX32664C_MOTION_TIME(ms) (uint8_t)((ms) / 20)
#define MAX32664C_MOTION_THRESHOLD(mg) (uint8_t)((mg) / 31)

/** @brief Accelerometer data structure.
 */
struct max32664c_acc_data_t {
	int16_t x;
	int16_t y;
	int16_t z;
} __packed;

/** @brief Skin contact detection states.
 *  @note The SCD states are only available when the SCD only mode is enabled.
 */
enum max32664c_scd_states {
	MAX32664C_SCD_STATE_UNKNOWN,
	MAX32664C_SCD_STATE_OFF_SKIN,
	MAX32664C_SCD_STATE_ON_OBJECT,
	MAX32664C_SCD_STATE_ON_SKIN,
};

/** @brief LED current structure.
 */
struct max32664c_led_current_t {
	uint8_t adj_flag;
	uint16_t adj_val;
} __packed;

/** @brief SpO2 measurement result structure.
 */
struct max32664c_spo2_meas_t {
	uint8_t confidence;
	uint16_t value;
	uint8_t complete;
	uint8_t low_signal_quality;
	uint8_t motion;
	uint8_t low_pi;
	uint8_t unreliable_r;
	uint8_t state;
} __packed;

/** @brief Extended SpO2 measurement result structure.
 */
struct max32664c_ext_spo2_meas_t {
	uint8_t confidence;
	uint16_t value;
	uint8_t valid_percent;
	uint8_t low_signal_flag;
	uint8_t motion_flag;
	uint8_t low_pi_flag;
	uint8_t unreliable_r_flag;
	uint8_t state;
} __packed;

/** @brief Raw data structure, reported by the sensor hub.
 */
struct max32664c_raw_report_t {
	uint32_t PPG1: 24;
	uint32_t PPG2: 24;
	uint32_t PPG3: 24;
	uint32_t PPG4: 24;
	uint32_t PPG5: 24;
	uint32_t PPG6: 24;
	struct max32664c_acc_data_t acc;
} __packed;

/** @brief SCD only data structure, reported by the sensor hub.
 */
struct max32664c_scd_report_t {
	uint8_t scd_classifier;
} __packed;

/** @brief Algorithm data structure, reported by the sensor hub.
 */
struct max32664c_report_t {
	uint8_t op_mode;
	uint16_t hr;
	uint8_t hr_confidence;
	uint16_t rr;
	uint8_t rr_confidence;
	uint8_t activity_class;
	uint16_t r;
	struct max32664c_spo2_meas_t spo2_meas;
	uint8_t scd_state;
} __packed;

/** @brief Extended algorithm data structure, reported by the sensor hub.
 */
struct max32664c_ext_report_t {
	uint8_t op_mode;
	uint16_t hr;
	uint8_t hr_confidence;
	uint16_t rr;
	uint8_t rr_confidence;
	uint8_t activity_class;

	uint32_t total_walk_steps;
	uint32_t total_run_steps;
	uint32_t total_energy_kcal;
	uint32_t total_amr_kcal;

	struct max32664c_led_current_t led_current_adj1;
	struct max32664c_led_current_t led_current_adj2;
	struct max32664c_led_current_t led_current_adj3;

	uint8_t integration_time_adj_flag;
	uint8_t requested_integration_time;

	uint8_t sampling_rate_adj_flag;
	uint8_t requested_sampling_rate;
	uint8_t requested_sampling_average;

	uint8_t hrm_afe_ctrl_state;
	uint8_t is_high_motion_for_hrm;

	uint8_t scd_state;

	uint16_t r_value;
	struct max32664c_ext_spo2_meas_t spo2_meas;

	/* Firmware 35.7.1+ fields (bytes 52-55) */
	uint8_t ibi_offset;
	uint8_t unreliable_orientation_flag;
	uint8_t reserved[2];
} __packed;

/** @brief Device configuration structure.
 */
struct max32664c_config {
	struct i2c_dt_spec i2c;
	struct gpio_dt_spec reset_gpio;

#ifdef CONFIG_MAX32664C_USE_INTERRUPT
	const struct device *dev;
	struct gpio_callback gpio_cb;
	struct k_work interrupt_work;
#endif /* CONFIG_MAX32664C_USE_INTERRUPT */

	struct gpio_dt_spec mfio_gpio;

	int32_t spo2_calib[3];
	uint16_t motion_time;
	uint16_t motion_threshold;

	uint8_t hr_config[2];
	uint8_t spo2_config[2];
	uint32_t led_current[4];          /**< Initial LED current in mA (supports 4 LEDs) */
	uint8_t min_integration_time_idx;
	uint8_t min_sampling_rate_idx;
	uint8_t max_integration_time_idx;
	uint8_t max_sampling_rate_idx;
	uint8_t report_period;            /*< Samples report period */

	bool use_max86141;
	bool use_max86161;
	bool use_max86174;                /**< Added MAX86174 flag */
	bool use_sample_counter_byte;
};

/** @brief Device runtime data structure.
 */
struct max32664c_data {
	struct max32664c_raw_report_t raw;
	struct max32664c_scd_report_t scd;
	struct max32664c_report_t report;
	struct max32664c_ext_report_t ext;

	enum max32664c_device_mode op_mode; /**< Current device mode */

	uint8_t motion_time;              /**< Motion time in milliseconds */
	uint8_t motion_threshold;         /**< Motion threshold in milli-g */
	uint32_t led_current[4];          /**< LED current in mA (supports 4 LEDs) */
	uint8_t min_integration_time_idx;
	uint8_t min_sampling_rate_idx;
	uint8_t max_integration_time_idx;
	uint8_t max_sampling_rate_idx;
	uint8_t report_period;            /*< Samples report period */
	uint8_t afe_id;
	uint8_t accel_id;
	uint8_t hub_ver[3];
	uint8_t ext_report_len;           /**< Dynamic extended report length (52 or 56) */

	/* Internal */
	struct k_thread thread;
	k_tid_t thread_id;
	bool is_thread_running;
	bool hub_online;
	bool thread_started;

	/* Deferred initialization state */
	struct k_mutex lock;      /* protects bring-up / one-time init */
	bool hub_ready;           /* true after successful bring-up */
	int64_t last_try_ms;      /* rate-limit retries */
	uint8_t last_status;      /* last hub Status byte (first byte of every response) */
	atomic_t mfio_hold;       /* >0 => keep hub awake between transactions */

#ifdef CONFIG_MAX32664C_USE_STATIC_MEMORY
	/** @brief This buffer is used to read all available messages from the sensor hub plus the
	 * status byte. The buffer size is defined by the CONFIG_MAX32664C_SAMPLE_BUFFER_SIZE
	 * Kconfig and the largest possible message. The buffer must contain enough space to store
	 * all available messages at every time because it is not possible to read a single message
	 * from the sensor hub.
	 */
#ifdef CONFIG_MAX32664C_USE_EXTENDED_REPORTS
	uint8_t max32664_i2c_buffer[(CONFIG_MAX32664C_SAMPLE_BUFFER_SIZE *
				     (sizeof(struct max32664c_raw_report_t) +
				      sizeof(struct max32664c_ext_report_t))) +
				    1];
#else
	uint8_t max32664_i2c_buffer[(CONFIG_MAX32664C_SAMPLE_BUFFER_SIZE *
				     (sizeof(struct max32664c_raw_report_t) +
				      sizeof(struct max32664c_report_t))) +
				    1];
#endif /* CONFIG_MAX32664C_USE_EXTENDED_REPORTS */
#else
	uint8_t *max32664_i2c_buffer;
#endif /* CONFIG_MAX32664C_USE_STATIC_MEMORY */

	K_KERNEL_STACK_MEMBER(thread_stack, CONFIG_MAX32664C_THREAD_STACK_SIZE);

	struct k_msgq raw_report_queue;
	struct k_msgq scd_report_queue;

#ifdef CONFIG_MAX32664C_USE_EXTENDED_REPORTS
	struct k_msgq ext_report_queue;
#else
	struct k_msgq report_queue;
#endif /* CONFIG_MAX32664C_USE_EXTENDED_REPORTS */

#ifdef CONFIG_MAX32664C_USE_STATIC_MEMORY
	uint8_t raw_report_queue_buffer[CONFIG_MAX32664C_QUEUE_SIZE *
					sizeof(struct max32664c_raw_report_t)];
	uint8_t scd_report_queue_buffer[CONFIG_MAX32664C_QUEUE_SIZE *
					sizeof(struct max32664c_scd_report_t)];

#ifdef CONFIG_MAX32664C_USE_EXTENDED_REPORTS
	uint8_t ext_report_queue_buffer[CONFIG_MAX32664C_QUEUE_SIZE *
					(sizeof(struct max32664c_raw_report_t) +
					 sizeof(struct max32664c_ext_report_t))];
#else
	uint8_t report_queue_buffer[CONFIG_MAX32664C_QUEUE_SIZE *
				    (sizeof(struct max32664c_raw_report_t) +
				     sizeof(struct max32664c_report_t))];
#endif /* CONFIG_MAX32664C_USE_EXTENDED_REPORTS */
#endif /* CONFIG_MAX32664C_USE_STATIC_MEMORY*/
};

/** @brief          Enable / Disable the accelerometer.
 *                  NOTE: This code is untested and may not work as expected.
 *  @param dev      Pointer to device
 *  @param enable   Enable / Disable
 *  @return         0 when successful
 */
int max32664c_acc_enable(const struct device *dev, bool enable);

/** @brief      Background worker for reading the sensor hub.
 *  @param dev  Pointer to device
 */
void max32664c_worker(const struct device *dev);

/** @brief          Read / write data from / to the sensor hub.
 *  @param dev      Pointer to device
 *  @param tx_buf   Pointer to transmit buffer
 *  @param tx_len   Length of transmit buffer
 *  @param rx_buf   Pointer to receive buffer
 *                  NOTE: The buffer must be large enough to store the response and the status byte!
 *  @param rx_len   Length of the receive buffer
 *  @param delay    Command delay (milliseconds)
 *  @return         0 when successful
 */
int max32664c_i2c_transmit(const struct device *dev, uint8_t *tx_buf, uint8_t tx_len,
			   uint8_t *rx_buf, uint32_t rx_len, uint16_t delay);

/** @brief      Run a basic initialization on the sensor hub.
 *  @param dev  Pointer to device
 *  @return     0 when successful
 */
int max32664c_init_hub(const struct device *dev);

/** @brief          Get AFE and accelerometer IDs from the sensor hub.
 *  @param dev      Pointer to device
 *  @param afe_id   Pointer to store AFE ID
 *  @param acc_id   Pointer to store accelerometer ID
 *  @return         0 when successful
 */
int max32664c_get_ids(const struct device *dev, uint8_t *afe_id, uint8_t *acc_id);

/* MAX86174 FW35 register configuration helpers */
int max86174_write_itime_slot(const struct device *dev, uint8_t slot, uint8_t itime_code);
int max86174_write_spsavg_slot(const struct device *dev, uint8_t slot, uint8_t spsavg_code);
int max86174_read_itime_all(const struct device *dev, uint8_t out[MAX86174_MEAS_SLOTS]);
int max86174_read_spsavg_all(const struct device *dev, uint8_t out[MAX86174_MEAS_SLOTS]);

/* Status code helper */
const char *max32664c_status_str(uint8_t st);

/* Algorithm control */
int max32664c_stop_algo(const struct device *dev);

#if CONFIG_MAX32664C_USE_INTERRUPT
/** @brief      Initialize the interrupt support for the sensor hub.
 *  @param dev  Pointer to device
 *  @return     0 when successful
 */
int max32664c_init_interrupt(const struct device *dev);
#endif /* CONFIG_MAX32664C_USE_INTERRUPT */

/**
 *  @brief Set SpO2 Calibration Coefficients
 *  Reload the SpO2 calibration coefficients after mode changes.
 *  @param dev  Pointer to device
 *  @return     0 when successful, negative errno on failure
 */
int max32664c_set_spo2_coeffs(const struct device *dev);