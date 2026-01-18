/*
 * main.c - Application Entry Point
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>

/* Application Libraries */
#include "max77658_main.h"
#include "max32664c_main.h"
#include "app_i2c_lock.h"

/* BLE Store-and-Forward Thread */
void start_ble_thread(void);
K_THREAD_DEFINE(ble_thread_id, 4096, start_ble_thread, NULL, NULL, NULL, 7, 0, 0);

LOG_MODULE_REGISTER(main, CONFIG_LOG_DEFAULT_LEVEL);

/* I2C Device */
#define I2C_NODE DT_NODELABEL(i2c21)
static const struct device *i2c_dev = DEVICE_DT_GET(I2C_NODE);

/* I2C Scanner Function */
static void i2c_scan(void)
{
    uint8_t addr;
    uint8_t cnt = 0;

    LOG_INF("--- I2C Bus Scan ---");
    
    if (!device_is_ready(i2c_dev)) {
        LOG_ERR("I2C device not ready!");
        return;
    }

    k_mutex_lock(&i2c_lock, K_FOREVER);

    for (addr = 0x03; addr <= 0x77; addr++) {
        struct i2c_msg msgs[1];
        uint8_t dst;

        msgs[0].buf = &dst;
        msgs[0].len = 0;
        msgs[0].flags = I2C_MSG_WRITE | I2C_MSG_STOP;

        if (i2c_transfer(i2c_dev, &msgs[0], 1, addr) == 0) {
            LOG_INF("Device found at 0x%02X", addr);
            cnt++;
        }
    }

    k_mutex_unlock(&i2c_lock);
    
    LOG_INF("Found %d device(s) on I2C bus", cnt);
}

int main(void)
{
    int ret;
    
    LOG_INF("--- System Startup ---");

    /* 1. Initialize PMIC Hardware (Protected) */
    LOG_INF("Initializing PMIC...");
    k_mutex_lock(&i2c_lock, K_FOREVER);
    ret = max77658_app_init();
    k_mutex_unlock(&i2c_lock);
    
    if (ret != 0) {
        LOG_ERR("MAX77658 Init Failed! Halting.");
        while (1) { k_msleep(1000); }
    }

    /* 2. Start Application Threads */
    /* The internal driver init for max32664c is handled by the sensor thread */
    LOG_INF("Starting Application Threads...");
    max77658_app_start();    /* Starts PMIC thread (Priority 7, 2000ms polling) */
    max32664c_app_start();   /* Starts Sensor thread (Priority 5, 40ms polling) */

    /* 3. Main thread runs I2C scanner in loop */
    LOG_INF("Main thread starting I2C scanner loop.");
    while (1) {
        k_sleep(K_SECONDS(5));

        if (max77658_shutdown_requested()) {
            LOG_WRN("Shutdown requested; skipping I2C scan.");
            continue;
        }

        i2c_scan();
    }

    return 0;
}