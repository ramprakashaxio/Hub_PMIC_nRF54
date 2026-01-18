/*
 * pmic_gpio.h - MAX77658 PMIC GPIO Control Interface
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * GPIO Pin Configuration (P1.xx):
 *   P1.07 - nEN:   Open-drain wake control (drive LOW or Hi-Z)
 *   P1.06 - nIRQ:  PMIC interrupt output (input with pull-up)
 *   P1.05 - nRST:  PMIC reset output (input, monitor only)
 *   P1.04 - AMUX:  Analog multiplexer output (input for ADC)
 */

#ifndef PMIC_GPIO_H
#define PMIC_GPIO_H

#include <zephyr/kernel.h>
#include <stdbool.h>

/**
 * @brief Initialize PMIC GPIO pins
 * 
 * Configures:
 *   - nEN:  Input (Hi-Z, released state)
 *   - nRST: Input with pull-up (PMIC drives this pin)
 *   - nIRQ: Input with pull-up and interrupt on falling edge
 *   - AMUX: Input for ADC reading
 * 
 * @return 0 on success, negative error code on failure
 */
int pmic_gpio_init(void);

/**
 * @brief Log current raw GPIO pin levels for debugging
 * 
 * Reads and logs the current state of nRST, nEN, and nIRQ pins.
 * Useful for diagnosing PMIC power state and GPIO configuration.
 */
void pmic_gpio_log_raw_levels(void);

/* ============================================================================
 * Interrupt Functions
 * ============================================================================ */

/**
 * @brief Register callback for PMIC interrupts
 * 
 * @param cb Function to call when interrupt occurs (NULL to disable)
 * @param user_data Context data passed to callback
 */
void pmic_irq_register_callback(void (*cb)(void *), void *user_data);

/**
 * @brief Enable PMIC interrupt
 */
void pmic_irq_enable(void);

/**
 * @brief Disable PMIC interrupt
 */
void pmic_irq_disable(void);

/**
 * @brief Check if PMIC interrupt is active
 * 
 * @return true if nIRQ is LOW (interrupt pending)
 * @return false if nIRQ is HIGH (no interrupt)
 */
bool pmic_irq_is_active(void);

/* ============================================================================
 * nEN Wake Control (Open-Drain)
 * ============================================================================ */

/**
 * @brief Send nEN pulse to wake PMIC from ship mode
 * 
 * Drives nEN LOW for specified duration, then releases to Hi-Z.
 * External pull-up resistor brings nEN back HIGH.
 * 
 * @param ms Pulse duration in milliseconds (typical: 10-100ms)
 */
void pmic_nen_pulse_ms(uint32_t ms);

/**
 * @brief Release nEN to Hi-Z state
 * 
 * Ensures nEN is in high-impedance (input) state.
 * External pull-up holds pin HIGH.
 * Use before entering ship mode to prevent accidental wake.
 */
void pmic_nen_release_hiz(void);

/* ============================================================================
 * nRST Monitoring (PMIC drives this pin)
 * ============================================================================ */

/**
 * @brief Check if nRST is HIGH
 * 
 * The PMIC drives nRST. When HIGH, PMIC has completed power-on.
 * 
 * @return true if nRST is HIGH (PMIC ready)
 * @return false if nRST is LOW (PMIC in reset/startup)
 */
bool pmic_is_nrst_high(void);

/**
 * @brief Wait for nRST to go HIGH (PMIC startup complete)
 * 
 * @param timeout_ms Maximum time to wait in milliseconds
 * @return 0 if nRST went HIGH, -ETIMEDOUT if timeout expired
 */
int pmic_wait_nrst_high(uint32_t timeout_ms);

/* ============================================================================
 * Raw GPIO Pin Access
 * ============================================================================ */

/**
 * @brief Get current nEN pin level
 * 
 * @return Pin level (0=LOW, 1=HIGH), -ENODEV if GPIO not initialized
 */
int pmic_gpio_get_nen(void);

/**
 * @brief Get current nRST pin level
 * 
 * @return Pin level (0=LOW, 1=HIGH), -ENODEV if GPIO not initialized
 */
int pmic_gpio_get_nrst(void);

/**
 * @brief Get current nIRQ pin level
 * 
 * @return Pin level (0=LOW, 1=HIGH), -ENODEV if GPIO not initialized
 */
int pmic_gpio_get_nirq(void);

/* ============================================================================
 * Legacy Functions (Deprecated)
 * ============================================================================ */

/**
 * @brief DEPRECATED - nRST is now PMIC output, cannot be driven
 * 
 * This function is kept for backward compatibility but does nothing.
 * Use pmic_nen_pulse_ms() instead to wake PMIC.
 */
void pmic_reset_assert(void);

/**
 * @brief DEPRECATED - nRST is now PMIC output, cannot be driven
 * 
 * This function is kept for backward compatibility but does nothing.
 */
void pmic_reset_deassert(void);

/**
 * @brief DEPRECATED - Use nEN pulse instead
 * 
 * Performs wake sequence using nEN pulse and waits for nRST.
 * 
 * @param reset_time_ms Duration to hold nEN LOW (recommended: 10ms)
 * @param startup_time_ms Maximum time to wait for nRST HIGH (recommended: 50-100ms)
 */
void pmic_hardware_reset(uint32_t reset_time_ms, uint32_t startup_time_ms);

#endif /* PMIC_GPIO_H */
