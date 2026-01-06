/*
 * bsp.h - Board Support Package for Zephyr
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef BSP_H_
#define BSP_H_

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief I2C write function pointer type
 * The application must provide implementation and register it
 */
typedef int (*bsp_i2c_write_fn)(uint8_t addr, const uint8_t *data, uint32_t len);

/**
 * @brief I2C read function pointer type
 * The application must provide implementation and register it
 */
typedef int (*bsp_i2c_read_fn)(uint8_t addr, uint8_t *data, uint32_t len);

/**
 * @brief I2C write-read (repeated start) function pointer type
 */
typedef int (*bsp_i2c_write_read_fn)(uint8_t addr, const uint8_t *tx_data, uint32_t tx_len, 
                                     uint8_t *rx_data, uint32_t rx_len);

/**
 * @brief Delay function pointer type
 */
typedef void (*bsp_delay_ms_fn)(uint32_t ms);
typedef void (*bsp_delay_us_fn)(uint32_t us);

/**
 * @brief BSP HAL structure for I2C operations
 */
typedef struct {
    bsp_i2c_write_fn      i2c_write;
    bsp_i2c_read_fn       i2c_read;
    bsp_i2c_write_read_fn i2c_write_read;
    bsp_delay_ms_fn       delay_ms;
    bsp_delay_us_fn       delay_us;
} bsp_hal_t;

/* Global BSP HAL instance - must be initialized by application */
extern bsp_hal_t g_bsp_hal;

/**
 * @brief Initialize BSP (I2C, delays, GPIO)
 * 
 * @return 0 on success, negative error code on failure
 */
int bsp_init(void);

/**
 * @brief I2C register read function for PMIC drivers
 * 
 * @param dev_addr I2C device address
 * @param reg_addr Register address
 * @param data Buffer to store read data
 * @param len Number of bytes to read
 * @return 0 on success, -1 on failure
 */
int32_t bsp_i2c_reg_read(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint32_t len);

/**
 * @brief I2C register write function for PMIC drivers
 * 
 * @param dev_addr I2C device address
 * @param reg_addr Register address
 * @param data Data to write
 * @param len Number of bytes to write
 * @return 0 on success, -1 on failure
 */
int32_t bsp_i2c_reg_write(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint32_t len);

/**
 * @brief Scan I2C bus for devices
 */
void bsp_i2c_scan(void);

/**
 * @brief Delay for specified milliseconds
 */
static inline void bsp_delay_ms(uint32_t ms)
{
    if (g_bsp_hal.delay_ms) {
        g_bsp_hal.delay_ms(ms);
    }
}

/**
 * @brief Delay for specified microseconds
 */
static inline void bsp_delay_us(uint32_t us)
{
    if (g_bsp_hal.delay_us) {
        g_bsp_hal.delay_us(us);
    }
}

#ifdef __cplusplus
}
#endif

#endif /* BSP_H_ */
