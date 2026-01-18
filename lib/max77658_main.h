/*
 * max77658_main.h
 * Application Interface for MAX77658 PMIC
 */

#ifndef MAX77658_MAIN_H_
#define MAX77658_MAIN_H_

#include <stdbool.h>

/**
 * @brief Check if software power off has been requested
 * @return true if shutdown was requested, false otherwise
 */
bool max77658_shutdown_requested(void);

/**
 * @brief Request software power off (atomic, single-shot)
 * @param reason Optional string describing why shutdown was requested (can be NULL)
 * @note This is thread-safe and will only allow one shutdown request
 */
void max77658_request_software_off(const char *reason);

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
 * @brief Software Power Off (SFT_OFF)
 * Enters software shutdown using CNFG_GLBL.SFT_CTRL = 0x02.
 * ✅ WILL wake on CHGIN insertion (charger plugged in)
 * ✅ WILL wake on nEN button press (if enabled in config)
 * ⚠️  This is NOT factory ship mode - button wake may be enabled
 * 
 * Use for: Normal shutdowns, user power-off button, idle timeout
 */
void max77658_enter_software_off(void);

/**
 * @brief Software Power Off (SFT_OFF) - No Lock Variant
 * Same as max77658_enter_software_off() but assumes i2c_lock is already held by caller.
 * Uses SFT_CTRL = 0x02 (software off with configurable wake sources).
 */
void max77658_enter_software_off_nolock(void);

/**
 * @brief Factory Ship Mode (FSM)
 * Enters deepest shutdown state using CNFG_GLBL.SFT_CTRL = 0x03.
 * ✅ WILL wake ONLY on CHGIN insertion (charger plugged in)
 * ❌ Will NOT wake on nEN button press
 * ✅ This is TRUE "factory ship mode" - minimal power consumption
 * 
 * Use for: Factory shipping, long-term storage, RMA returns
 * Disables all rails, releases nEN to Hi-Z, stops IRQ processing.
 */
void max77658_enter_ship_mode(void);

#endif /* MAX77658_MAIN_H_ */
