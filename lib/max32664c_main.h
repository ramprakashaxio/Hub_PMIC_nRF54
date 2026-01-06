/*
 * max32664c_main.h
 * Public API for the MAX32664C Application Logic
 */

#ifndef MAX32664C_MAIN_H_
#define MAX32664C_MAIN_H_

/**
 * @brief Initialize the MAX32664C application layer.
 * @return 0 on success, negative errno on failure.
 */
int max32664c_app_init(void);

/**
 * @brief Start the sensor application thread
 * Creates and starts a background thread that polls the sensor
 * at 25Hz (every 40ms).
 */
void max32664c_app_start(void);

/**
 * @brief Checks for new sensor data, processes it, and logs the result.
 * * This function is NON-BLOCKING. It returns immediately if no data is available.
 * It should be called periodically from the main loop.
 */
void max32664c_app_process_events(void);

#endif /* MAX32664C_MAIN_H_ */