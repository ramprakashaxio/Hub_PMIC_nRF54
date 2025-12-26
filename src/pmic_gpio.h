/*
 * pmic_gpio.h - MAX77658 PMIC GPIO Control Interface
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * GPIO Pin Configuration:
 *   P1.13 - PMIC_nRST: Active LOW hardware reset output
 *   P0.30 - PMIC_nIRQ: Active LOW interrupt input with pull-up
 */

#ifndef PMIC_GPIO_H
#define PMIC_GPIO_H

#include <zephyr/kernel.h>
#include <stdbool.h>

/**
 * @brief PMIC interrupt callback type
 * 
 * @param user_data User-provided context data
 */
typedef void (*pmic_irq_callback_t)(void *user_data);

/**
 * @brief Initialize PMIC GPIO pins
 * 
 * Configures:
 *   - nRST pin as output (deasserted HIGH by default)
 *   - nIRQ pin as input with pull-up
 * 
 * @return 0 on success, negative error code on failure
 */
int pmic_gpio_init(void);

/**
 * @brief Assert PMIC reset (active LOW)
 * 
 * Pulls nRST pin LOW to put PMIC in reset state.
 * The PMIC will not operate while reset is asserted.
 */
void pmic_reset_assert(void);

/**
 * @brief Deassert PMIC reset
 * 
 * Pulls nRST pin HIGH to release PMIC from reset.
 * PMIC will begin normal operation after reset delay.
 */
void pmic_reset_deassert(void);

/**
 * @brief Perform PMIC hardware reset sequence
 * 
 * Executes a complete reset cycle:
 *   1. Assert reset (LOW)
 *   2. Wait for specified duration
 *   3. Deassert reset (HIGH)
 *   4. Wait for PMIC startup
 * 
 * @param reset_time_ms Duration to hold reset (recommended: 10ms)
 * @param startup_time_ms Time to wait after reset release (recommended: 50ms)
 */
void pmic_hardware_reset(uint32_t reset_time_ms, uint32_t startup_time_ms);

/**
 * @brief Check if PMIC interrupt is active
 * 
 * @return true if nIRQ is LOW (interrupt pending)
 * @return false if nIRQ is HIGH (no interrupt)
 */
bool pmic_irq_is_active(void);

/**
 * @brief Register callback for PMIC interrupts
 * 
 * Configures GPIO interrupt on falling edge of nIRQ pin.
 * Callback is invoked from interrupt context.
 * 
 * @param callback Function to call when interrupt occurs (NULL to disable)
 * @param user_data Context data passed to callback
 * @return 0 on success, negative error code on failure
 */
int pmic_irq_register_callback(pmic_irq_callback_t callback, void *user_data);

/**
 * @brief Enable PMIC interrupt
 * 
 * Enables GPIO interrupt detection on nIRQ pin.
 */
void pmic_irq_enable(void);

/**
 * @brief Disable PMIC interrupt
 * 
 * Disables GPIO interrupt detection on nIRQ pin.
 */
void pmic_irq_disable(void);

#endif /* PMIC_GPIO_H */
