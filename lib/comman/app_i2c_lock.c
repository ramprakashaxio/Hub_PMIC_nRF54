/*
 * Copyright (c) 2025
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "app_i2c_lock.h"

/**
 * @brief Global I2C mutex definition
 * 
 * This mutex is used by the MAX32664C driver to ensure atomic
 * I2C transactions (wake + write + delay + read).
 */
K_MUTEX_DEFINE(i2c_lock);
