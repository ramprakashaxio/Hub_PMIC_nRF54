/*
 * bsp.c - Board Support Package implementation for Zephyr
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/logging/log.h>
#include <string.h>

#include "bsp.h"

LOG_MODULE_REGISTER(bsp, CONFIG_LOG_DEFAULT_LEVEL);

/*
 * I2C Device Configuration
 * nRF52840DK uses i2c0, nRF5340DK uses i2c1, nRF54L15DK uses i2c21
 */
#if DT_NODE_HAS_STATUS(DT_NODELABEL(i2c21), okay)
#define I2C_NODE DT_NODELABEL(i2c21)
#elif DT_NODE_HAS_STATUS(DT_NODELABEL(i2c1), okay)
#define I2C_NODE DT_NODELABEL(i2c1)
#elif DT_NODE_HAS_STATUS(DT_NODELABEL(i2c0), okay)
#define I2C_NODE DT_NODELABEL(i2c0)
#else
#error "No I2C device found in devicetree (i2c0, i2c1, or i2c21)"
#endif

/* I2C device handle */
static const struct device *i2c_dev;

/*
 * BSP delay implementations
 */
static void bsp_delay_ms_impl(uint32_t ms)
{
    k_msleep(ms);
}

static void bsp_delay_us_impl(uint32_t us)
{
    k_usleep(us);
}

/* Global BSP HAL instance */
bsp_hal_t g_bsp_hal = {
    .i2c_write = NULL,
    .i2c_read = NULL,
    .i2c_write_read = NULL,
    .delay_ms = NULL,
    .delay_us = NULL
};

int bsp_init(void)
{
    /* Initialize I2C device */
    i2c_dev = DEVICE_DT_GET(I2C_NODE);

    if (!device_is_ready(i2c_dev)) {
        LOG_ERR("I2C device not ready");
        return -ENODEV;
    }

    LOG_INF("I2C device ready: %s", i2c_dev->name);

    /* Initialize HAL function pointers */
    g_bsp_hal.delay_ms = bsp_delay_ms_impl;
    g_bsp_hal.delay_us = bsp_delay_us_impl;
    g_bsp_hal.i2c_write = NULL;
    g_bsp_hal.i2c_read = NULL;
    g_bsp_hal.i2c_write_read = NULL;

    LOG_INF("BSP initialized");
    return 0;
}

int32_t bsp_i2c_reg_read(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint32_t len)
{
    int ret;

    ret = i2c_burst_read(i2c_dev, dev_addr, reg_addr, data, len);
    if (ret != 0) {
        LOG_ERR("I2C read failed: dev=0x%02X reg=0x%02X err=%d", dev_addr, reg_addr, ret);
        return -1;
    }

    return 0;
}

int32_t bsp_i2c_reg_write(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint32_t len)
{
    int ret;
    uint8_t buf[len + 1];

    buf[0] = reg_addr;
    memcpy(&buf[1], data, len);

    ret = i2c_write(i2c_dev, buf, len + 1, dev_addr);
    if (ret != 0) {
        LOG_ERR("I2C write failed: dev=0x%02X reg=0x%02X err=%d", dev_addr, reg_addr, ret);
        return -1;
    }

    return 0;
}

void bsp_i2c_scan(void)
{
    uint8_t dummy;
    int found = 0;

    LOG_INF("=== I2C Bus Scan ===");

    for (uint8_t addr = 0x08; addr < 0x78; addr++) {
        if (i2c_read(i2c_dev, &dummy, 0, addr) == 0) {
            LOG_INF("Found device at address: 0x%02X", addr);
            found++;
        }
    }

    if (found == 0) {
        LOG_WRN("No I2C devices found!");
    } else {
        LOG_INF("Total devices found: %d", found);
    }
}
