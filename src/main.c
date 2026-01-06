/*
 * main.c - Application Entry Point
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

/* Application Libraries */
#include "max77658_main.h"
#include "max32664c_main.h"
#include "app_i2c_lock.h"

LOG_MODULE_REGISTER(main, CONFIG_LOG_DEFAULT_LEVEL);

int main(void)
{
    LOG_INF("--- System Startup ---");

    /* 1. Initialize PMIC Hardware (Protected) */
    LOG_INF("Initializing PMIC...");
    k_mutex_lock(&i2c_lock, K_FOREVER);
    if (max77658_app_init() != 0) {
        LOG_ERR("MAX77658 Init Failed!");
    }
    k_mutex_unlock(&i2c_lock);

    /* 2. Start Application Threads */
    /* The internal driver init for max32664c is handled by the sensor thread */
    LOG_INF("Starting Application Threads...");
    max77658_app_start();    /* Starts PMIC thread (Priority 7, 2000ms polling) */
    max32664c_app_start();   /* Starts Sensor thread (Priority 5, 40ms polling) */

    /* 3. Main thread sleeps forever */
    LOG_INF("Main thread idle. Application threads running.");
    k_sleep(K_FOREVER);

    return 0;
}