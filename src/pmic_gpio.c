/*
 * pmic_gpio.c - MAX77658 PMIC GPIO Control Implementation
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * GPIO Pin Configuration:
 *   P1.13 - PMIC_nRST: Active LOW hardware reset output
 *   P0.30 - PMIC_nIRQ: Active LOW interrupt input with pull-up
 */

#include "pmic_gpio.h"
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(pmic_gpio, CONFIG_LOG_DEFAULT_LEVEL);

/*
 * GPIO Pin Definitions from Devicetree
 *
 * nRST: P1.13 - Output, Active LOW, default HIGH (deasserted)
 * nIRQ: P0.30 - Input, Active LOW, Pull-up enabled
 */
#define PMIC_NRST_NODE  DT_ALIAS(pmic_rst)
#define PMIC_NIRQ_NODE  DT_ALIAS(pmic_irq)

#if DT_NODE_EXISTS(PMIC_NRST_NODE)
static const struct gpio_dt_spec pmic_nrst = GPIO_DT_SPEC_GET(PMIC_NRST_NODE, gpios);
#define PMIC_NRST_AVAILABLE 1
#else
#define PMIC_NRST_AVAILABLE 0
#warning "PMIC nRST pin not defined in devicetree"
#endif

#if DT_NODE_EXISTS(PMIC_NIRQ_NODE)
static const struct gpio_dt_spec pmic_nirq = GPIO_DT_SPEC_GET(PMIC_NIRQ_NODE, gpios);
#define PMIC_NIRQ_AVAILABLE 1
#else
#define PMIC_NIRQ_AVAILABLE 0
#warning "PMIC nIRQ pin not defined in devicetree"
#endif

/* Interrupt callback storage */
static struct gpio_callback nirq_cb_data;
static pmic_irq_callback_t user_callback = NULL;
static void *user_callback_data = NULL;

/*
 * Internal GPIO interrupt handler
 */
#if PMIC_NIRQ_AVAILABLE
static void nirq_isr(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
    ARG_UNUSED(dev);
    ARG_UNUSED(cb);
    ARG_UNUSED(pins);

    LOG_DBG("PMIC IRQ triggered");

    if (user_callback != NULL) {
        user_callback(user_callback_data);
    }
}
#endif

int pmic_gpio_init(void)
{
    int ret;

    LOG_INF("Initializing PMIC GPIO pins");

#if PMIC_NRST_AVAILABLE
    /* Configure nRST as output, initially HIGH (deasserted) */
    if (!gpio_is_ready_dt(&pmic_nrst)) {
        LOG_ERR("nRST GPIO device not ready");
        return -ENODEV;
    }

    ret = gpio_pin_configure_dt(&pmic_nrst, GPIO_OUTPUT_HIGH);
    if (ret < 0) {
        LOG_ERR("Failed to configure nRST pin: %d", ret);
        return ret;
    }

    LOG_INF("  nRST: P%d.%02d configured (output, deasserted)",
            pmic_nrst.port == DEVICE_DT_GET(DT_NODELABEL(gpio0)) ? 0 : 1,
            pmic_nrst.pin);
#else
    LOG_WRN("  nRST: Not available");
#endif

#if PMIC_NIRQ_AVAILABLE
    /* Configure nIRQ as input with pull-up */
    if (!gpio_is_ready_dt(&pmic_nirq)) {
        LOG_ERR("nIRQ GPIO device not ready");
        return -ENODEV;
    }

    ret = gpio_pin_configure_dt(&pmic_nirq, GPIO_INPUT | GPIO_PULL_UP);
    if (ret < 0) {
        LOG_ERR("Failed to configure nIRQ pin: %d", ret);
        return ret;
    }

    LOG_INF("  nIRQ: P%d.%02d configured (input, pull-up)",
            pmic_nirq.port == DEVICE_DT_GET(DT_NODELABEL(gpio0)) ? 0 : 1,
            pmic_nirq.pin);
#else
    LOG_WRN("  nIRQ: Not available");
#endif

    LOG_INF("PMIC GPIO initialization complete");
    return 0;
}

void pmic_reset_assert(void)
{
#if PMIC_NRST_AVAILABLE
    /* Set pin LOW (active) - PMIC enters reset */
    gpio_pin_set_dt(&pmic_nrst, 1);  /* 1 = active (LOW due to GPIO_ACTIVE_LOW) */
    LOG_INF("PMIC reset ASSERTED (nRST=LOW)");
#else
    LOG_WRN("pmic_reset_assert: nRST pin not available");
#endif
}

void pmic_reset_deassert(void)
{
#if PMIC_NRST_AVAILABLE
    /* Set pin HIGH (inactive) - PMIC starts operating */
    gpio_pin_set_dt(&pmic_nrst, 0);  /* 0 = inactive (HIGH due to GPIO_ACTIVE_LOW) */
    LOG_INF("PMIC reset DEASSERTED (nRST=HIGH)");
#else
    LOG_WRN("pmic_reset_deassert: nRST pin not available");
#endif
}

void pmic_hardware_reset(uint32_t reset_time_ms, uint32_t startup_time_ms)
{
    LOG_INF("Performing PMIC hardware reset sequence");
    LOG_INF("  Reset hold time: %u ms", reset_time_ms);
    LOG_INF("  Startup wait time: %u ms", startup_time_ms);

    /* Step 1: Assert reset */
    pmic_reset_assert();

    /* Step 2: Wait for reset to take effect */
    k_msleep(reset_time_ms);

    /* Step 3: Release reset */
    pmic_reset_deassert();

    /* Step 4: Wait for PMIC to start up */
    k_msleep(startup_time_ms);

    LOG_INF("PMIC hardware reset complete");
}

bool pmic_irq_is_active(void)
{
#if PMIC_NIRQ_AVAILABLE
    /* Returns 1 if active (pin is LOW), 0 if inactive (pin is HIGH) */
    int val = gpio_pin_get_dt(&pmic_nirq);
    return (val == 1);  /* 1 = active due to GPIO_ACTIVE_LOW */
#else
    return false;
#endif
}

int pmic_irq_register_callback(pmic_irq_callback_t callback, void *user_data)
{
#if PMIC_NIRQ_AVAILABLE
    int ret;

    user_callback = callback;
    user_callback_data = user_data;

    if (callback == NULL) {
        /* Disable and remove callback */
        pmic_irq_disable();
        gpio_remove_callback(pmic_nirq.port, &nirq_cb_data);
        LOG_INF("PMIC IRQ callback removed");
        return 0;
    }

    /* Configure interrupt for falling edge (HIGH to LOW transition) */
    ret = gpio_pin_interrupt_configure_dt(&pmic_nirq, GPIO_INT_EDGE_TO_ACTIVE);
    if (ret < 0) {
        LOG_ERR("Failed to configure nIRQ interrupt: %d", ret);
        return ret;
    }

    /* Initialize callback structure */
    gpio_init_callback(&nirq_cb_data, nirq_isr, BIT(pmic_nirq.pin));

    /* Add callback */
    ret = gpio_add_callback(pmic_nirq.port, &nirq_cb_data);
    if (ret < 0) {
        LOG_ERR("Failed to add nIRQ callback: %d", ret);
        return ret;
    }

    LOG_INF("PMIC IRQ callback registered");
    return 0;
#else
    LOG_WRN("pmic_irq_register_callback: nIRQ pin not available");
    return -ENOTSUP;
#endif
}

void pmic_irq_enable(void)
{
#if PMIC_NIRQ_AVAILABLE
    gpio_pin_interrupt_configure_dt(&pmic_nirq, GPIO_INT_EDGE_TO_ACTIVE);
    LOG_DBG("PMIC IRQ enabled");
#endif
}

void pmic_irq_disable(void)
{
#if PMIC_NIRQ_AVAILABLE
    gpio_pin_interrupt_configure_dt(&pmic_nirq, GPIO_INT_DISABLE);
    LOG_DBG("PMIC IRQ disabled");
#endif
}
