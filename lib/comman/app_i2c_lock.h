/*
 * Copyright (c) 2025
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef APP_I2C_LOCK_H_
#define APP_I2C_LOCK_H_

#include <zephyr/kernel.h>

/**
 * @brief Global I2C mutex for MAX32664C driver
 * 
 * This mutex protects I2C bus access for the MAX32664C sensor hub
 * to prevent race conditions during wake-delay-write-read sequences.
 */
extern struct k_mutex i2c_lock;

#endif /* APP_I2C_LOCK_H_ */
