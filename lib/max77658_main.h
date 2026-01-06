/*
 * max77658_main.h
 * Application Interface for MAX77658 PMIC
 */

#ifndef MAX77658_MAIN_H_
#define MAX77658_MAIN_H_

/**
 * @brief Initialize the entire PMIC Subsystem
 * Handles I2C, GPIO, Reset, Rails, Charger, and Fuel Gauge.
 * Includes soft-start logic for 5V rail.
 * 
 * @return 0 on success, negative on critical failure
 */
int max77658_app_init(void);

/**
 * @brief Start the PMIC thread
 * Creates and starts a background thread that monitors PMIC status
 * at 0.5Hz (every 2 seconds).
 */
void max77658_app_start(void);

/**
 * @brief Process PMIC Events
 * Call this inside your main while(1) loop.
 * Handles interrupts and logs status.
 */
void max77658_app_process_events(void);

/**
 * @brief Turn off the system (Ship Mode)
 */
void max77658_enter_ship_mode(void);

#endif /* MAX77658_MAIN_H_ */
