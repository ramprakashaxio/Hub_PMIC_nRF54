/*
 * pmic_gpio.c - MAX77658 PMIC GPIO Control Implementation
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * GPIO Pin Configuration (P1.xx):
 *   P1.07 - nEN:   Open-drain wake control (drive LOW or Hi-Z)
 *   P1.06 - nIRQ:  PMIC interrupt output (input with pull-up)
 *   P1.05 - nRST:  PMIC reset output (input, monitor only)
 *   P1.04 - AMUX:  Analog multiplexer output (input for ADC)
 */

#include "pmic_gpio.h"
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(pmic_gpio, CONFIG_LOG_DEFAULT_LEVEL);

#define PMIC_PORT_NODE   DT_NODELABEL(gpio1)

#define PIN_NEN   7  // P1.07
#define PIN_NIRQ  6  // P1.06
#define PIN_NRST  5  // P1.05
#define PIN_AMUX  4  // P1.04

static const struct device *gpio1;

static struct gpio_callback irq_cb;
static void (*user_irq_cb)(void *user_data);
static void *user_irq_ud;

/*
 * Internal GPIO interrupt handler
 */
static void irq_isr(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
    ARG_UNUSED(dev);
    ARG_UNUSED(cb);
    ARG_UNUSED(pins);
    
    if (user_irq_cb) {
        user_irq_cb(user_irq_ud);
    }
}

int pmic_gpio_init(void)
{
    gpio1 = DEVICE_DT_GET(PMIC_PORT_NODE);
    if (!device_is_ready(gpio1)) {
        LOG_ERR("gpio1 not ready");
        return -ENODEV;
    }

    /* nEN: Start RELEASED as true Hi-Z (no internal pull-up) */
    gpio_pin_configure(gpio1, PIN_NEN, GPIO_INPUT);
    LOG_INF("  nEN: P1.%02d configured (Hi-Z, released, no-pull)", PIN_NEN);

    /* nRST: PMIC output -> Monitor as INPUT (external pull-up present) */
    gpio_pin_configure(gpio1, PIN_NRST, GPIO_INPUT);
    LOG_INF("  nRST: P1.%02d configured (input, monitor)", PIN_NRST);

    /* nIRQ: PMIC output open-drain -> INPUT + interrupt
     * Internal pull-up enabled for robustness (backup to external pull-up) */
    gpio_pin_configure(gpio1, PIN_NIRQ, GPIO_INPUT | GPIO_PULL_UP);
    gpio_pin_interrupt_configure(gpio1, PIN_NIRQ, GPIO_INT_EDGE_FALLING);
    
    gpio_init_callback(&irq_cb, irq_isr, BIT(PIN_NIRQ));
    gpio_add_callback(gpio1, &irq_cb);
    LOG_INF("  nIRQ: P1.%02d configured (input, interrupt enabled)", PIN_NIRQ);

    /* AMUX_OUT: Input for ADC reading */
    gpio_pin_configure(gpio1, PIN_AMUX, GPIO_INPUT);
    LOG_INF("  AMUX: P1.%02d configured (input, ADC)", PIN_AMUX);

    LOG_INF("PMIC GPIO initialization complete");
    return 0;
}

void pmic_irq_register_callback(void (*cb)(void *), void *user_data)
{
    user_irq_cb = cb;
    user_irq_ud = user_data;
    LOG_INF("PMIC IRQ callback registered");
}

void pmic_irq_enable(void)
{
    /* Already configured as edge falling; nothing else needed */
    LOG_DBG("PMIC IRQ enabled");
}

void pmic_irq_disable(void)
{
    gpio_pin_interrupt_configure(gpio1, PIN_NIRQ, GPIO_INT_DISABLE);
    LOG_DBG("PMIC IRQ disabled");
}

bool pmic_irq_is_active(void)
{
    int val = gpio_pin_get(gpio1, PIN_NIRQ);
    return (val == 0);  /* Active LOW */
}

/* ============================================================================
 * Open-Drain nEN Control (Wake PMIC from Ship Mode)
 * ============================================================================ */

static inline void pmic_nen_drive_low(void)
{
    gpio_pin_configure(gpio1, PIN_NEN, GPIO_OUTPUT_LOW);
}

void pmic_nen_release_hiz(void)
{
    /* True Hi-Z: input, no pull (external pull-up only if present) */
    gpio_pin_configure(gpio1, PIN_NEN, GPIO_INPUT);
}

void pmic_nen_pulse_ms(uint32_t ms)
{
    LOG_INF("nEN pulse: %u ms", ms);
    pmic_nen_drive_low();
    k_msleep(ms);
    pmic_nen_release_hiz();
}

/* ============================================================================
 * nRST Monitoring (PMIC drives this pin)
 * ============================================================================ */

bool pmic_is_nrst_high(void)
{
    int v = gpio_pin_get(gpio1, PIN_NRST);
    return (v > 0);
}

int pmic_wait_nrst_high(uint32_t timeout_ms)
{
    uint32_t t0 = k_uptime_get_32();
    while ((k_uptime_get_32() - t0) < timeout_ms) {
        if (pmic_is_nrst_high()) {
            LOG_INF("nRST went HIGH after %u ms", k_uptime_get_32() - t0);
            return 0;
        }
        k_msleep(2);
    }
    LOG_ERR("nRST timeout after %u ms", timeout_ms);
    return -ETIMEDOUT;
}

/* ============================================================================
 * Raw GPIO Pin Access
 * ============================================================================ */

int pmic_gpio_get_nen(void)
{
    if (!gpio1) return -ENODEV;
    return gpio_pin_get(gpio1, PIN_NEN);
}

int pmic_gpio_get_nrst(void)
{
    if (!gpio1) return -ENODEV;
    return gpio_pin_get(gpio1, PIN_NRST);
}

int pmic_gpio_get_nirq(void)
{
    if (!gpio1) return -ENODEV;
    return gpio_pin_get(gpio1, PIN_NIRQ);
}

/* ============================================================================
 * Legacy Functions (Deprecated - nRST is now PMIC output)
 * ============================================================================ */

void pmic_reset_assert(void)
{
    LOG_WRN("pmic_reset_assert: Deprecated - nRST is PMIC output, cannot drive");
}

void pmic_reset_deassert(void)
{
    LOG_WRN("pmic_reset_deassert: Deprecated - nRST is PMIC output, cannot drive");
}

void pmic_hardware_reset(uint32_t reset_time_ms, uint32_t startup_time_ms)
{
    LOG_WRN("pmic_hardware_reset: Deprecated - Use nEN pulse instead");
    LOG_INF("Performing nEN pulse wake sequence");
    
    /* Drive nEN low for reset duration */
    pmic_nen_pulse_ms(reset_time_ms);
    
    /* Wait for PMIC to start up and drive nRST high */
    LOG_INF("Waiting for nRST to go HIGH...");
    if (pmic_wait_nrst_high(startup_time_ms) == 0) {
        LOG_INF("PMIC wake complete");
    } else {
        LOG_ERR("PMIC failed to wake (nRST timeout)");
    }
}
