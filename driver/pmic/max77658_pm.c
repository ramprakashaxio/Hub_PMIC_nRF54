/*
 * max77658_pm.c
 *
 * Created on: Nov 9, 2021
 * Author: kai
 * corrected: [Current Date]
 */

/* Includes ----------------------------------------------------------- */
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include "max77658_pm.h"
#include "max77658_defines.h"

LOG_MODULE_REGISTER(max77658_pm, CONFIG_PMIC_MAX77658_LOG_LEVEL);

/* Private defines ---------------------------------------------------- */
#define SUCCESS   0
#define ERROR     -1

/* Function definitions ----------------------------------------------- */

/**
  * @brief  Baseline Initialization following rules printed in MAX77650 Programmers Guide Chapter 4 Page 5
  *
  */
void max77658_pm_base_line_init(max77658_pm_t *ctx)
{
    LOG_INF("Baseline Initialization!");

    if(max77658_pm_set_SBIA_LPM(ctx, 0x00) == SUCCESS)  //Set Main Bias to normal Mode
    {
       LOG_INF("Set Main Bias to normal Mode: OK");
    }
    if(max77658_pm_set_nEN_MODE(ctx, 0x00) == SUCCESS)  //set on/off-button to push-button
    {
       LOG_INF("Set on/off-button to push-button: OK");
    }
    if(max77658_pm_set_DBEN_nEN(ctx, 0x00) == SUCCESS)  //Set nEN input debounce time to 30ms
    {
       LOG_INF("Set nEN input debounce time to 30ms: OK");
    }
    
    // 0 = MAX77658
    if(max77658_pm_get_DIDM(ctx) == 0x00) 
    {
       LOG_INF("Get DIDM: 0 = MAX77658-OK");
    }
    
    if(max77658_pm_get_CID(ctx) > -1) //Checking OTP options
    {
       LOG_INF("Get CID: OK");
    }
}

uint8_t max77658_pm_get_bit(uint8_t input, uint8_t bit_order)
{
    return (input >> bit_order) & 0x01;
}

/**
  * @brief  Helper to convert SBB register value to millivolts
  * @note   Formula: V_out = (REG_VAL * 25mV) + 500mV
  *         Datasheet: 0x00-0x0B may be invalid/reserved or low range (starts 0.5V)
  *         0x0C (12) = 0.8V, each step adds 25mV
  */
int max77658_pm_sbb_reg_to_mv(uint8_t reg_val)
{
    /* Simplified linear conversion for typical range > 0.8V */
    /* Exact datasheet logic:
       0x00 - 0x0B: invalid/reserved or low range (starts 0.5V)
       0x0C (12) = 0.8V
       Each step adds 25mV
    */
    if (reg_val < 12) return 500 + (reg_val * 25);
    
    /* Formula for >= 0.8V: 800mV + ((reg_val - 12) * 25mV) */
    /* Simplified: (reg_val * 25) + 500 */
    return 500 + (reg_val * 25);
}

/**
  * @brief  Read generic device register
  */
int32_t max77658_pm_read_reg(max77658_pm_t *ctx, uint8_t reg, uint8_t *data)
{
    int32_t ret;
    ret = ctx->read_reg(ctx->device_address, reg, data, 1);
    return ret;
}

/**
  * @brief  Write generic device register
  */
int32_t max77658_pm_write_reg(max77658_pm_t *ctx, uint8_t reg, uint8_t *data)
{
    int32_t ret;
    ret = ctx->write_reg(ctx->device_address, reg, data, 1);
    return ret;
}

/* ================================================================= */
/* ======================== READ FUNCTIONS ========================= */
/* ================================================================= */

int32_t max77658_pm_get_INT_GLBL0(max77658_pm_t *ctx)
{
    int32_t ret;
    uint8_t data;
    ret = max77658_pm_read_reg(ctx, MAX77658_INT_GLBL0, &data);
    if(ret == 0) return data;
    return ERROR;
}

int32_t max77658_pm_get_INT_GLBL1(max77658_pm_t *ctx)
{
    int32_t ret;
    uint8_t data;
    ret = max77658_pm_read_reg(ctx, MAX77658_INT_GLBL1, &data);
    if(ret == 0) return (data & 0x7F); // Mask reserved bit 7
    return ERROR;
}

int32_t max77658_pm_get_ERCFLAG(max77658_pm_t *ctx)
{
    int32_t ret;
    uint8_t data;
    ret = max77658_pm_read_reg(ctx, MAX77658_ERCFLAG, &data);
    if(ret == 0) return data;
    return ERROR;
}

/* STAT_GLBL Register Fields */

int32_t max77658_pm_get_DIDM(max77658_pm_t *ctx)
{
    int32_t ret;
    uint8_t data;
    ret = max77658_pm_read_reg(ctx, MAX77658_STAT_GLBL, &data);
    if(ret == 0) return (data >> 7) & 0x01;
    return ERROR;
}

int32_t max77658_pm_get_BOK(max77658_pm_t *ctx)
{
    int32_t ret;
    uint8_t data;
    ret = max77658_pm_read_reg(ctx, MAX77658_STAT_GLBL, &data);
    if(ret == 0) return (data >> 6) & 0x01;
    return ERROR;
}

int32_t max77658_pm_get_DOD0_S(max77658_pm_t *ctx)
{
    int32_t ret;
    uint8_t data;
    ret = max77658_pm_read_reg(ctx, MAX77658_STAT_GLBL, &data);
    if(ret == 0) return (data >> 5) & 0x01;
    return ERROR;
}

int32_t max77658_pm_get_DOD1_S(max77658_pm_t *ctx)
{
    int32_t ret;
    uint8_t data;
    ret = max77658_pm_read_reg(ctx, MAX77658_STAT_GLBL, &data);
    if(ret == 0) return (data >> 4) & 0x01;
    return ERROR;
}

int32_t max77658_pm_get_TJAL2_S(max77658_pm_t *ctx)
{
    int32_t ret;
    uint8_t data;
    ret = max77658_pm_read_reg(ctx, MAX77658_STAT_GLBL, &data);
    if(ret == 0) return (data >> 3) & 0x01;
    return ERROR;
}

int32_t max77658_pm_get_TJAL1_S(max77658_pm_t *ctx)
{
    int32_t ret;
    uint8_t data;
    ret = max77658_pm_read_reg(ctx, MAX77658_STAT_GLBL, &data);
    if(ret == 0) return (data >> 2) & 0x01;
    return ERROR;
}

int32_t max77658_pm_get_STAT_EN(max77658_pm_t *ctx)
{
    int32_t ret;
    uint8_t data;
    ret = max77658_pm_read_reg(ctx, MAX77658_STAT_GLBL, &data);
    if(ret == 0) return (data >> 1) & 0x01;
    return ERROR;
}

int32_t max77658_pm_get_STAT_IRQ(max77658_pm_t *ctx)
{
    int32_t ret;
    uint8_t data;
    ret = max77658_pm_read_reg(ctx, MAX77658_STAT_GLBL, &data);
    if(ret == 0) return data & 0x01;
    return ERROR;
}

/* Mask Registers */

int32_t max77658_pm_get_INTM_GLBL0(max77658_pm_t *ctx)
{
    int32_t ret;
    uint8_t data;
    ret = max77658_pm_read_reg(ctx, MAX77658_INTM_GLBL0, &data);
    if(ret == 0) return data;
    return ERROR;
}

int32_t max77658_pm_get_INTM_GLBL1(max77658_pm_t *ctx)
{
    int32_t ret;
    uint8_t data;
    ret = max77658_pm_read_reg(ctx, MAX77658_INTM_GLBL1, &data);
    if(ret == 0) return data & 0x7F;
    return ERROR;
}

/* Config Global */

int32_t max77658_pm_get_PU_DIS(max77658_pm_t *ctx)
{
    int32_t ret;
    uint8_t data;
    ret = max77658_pm_read_reg(ctx, MAX77658_CNFG_GLBL, &data);
    if(ret == 0) return (data >> 7) & 0x01;
    return ERROR;
}

int32_t max77658_pm_get_T_MRST(max77658_pm_t *ctx)
{
    int32_t ret;
    uint8_t data;
    ret = max77658_pm_read_reg(ctx, MAX77658_CNFG_GLBL, &data);
    if(ret == 0) return (data >> 6) & 0x01;
    return ERROR;
}

int32_t max77658_pm_get_SBIA_LPM(max77658_pm_t *ctx)
{
    int32_t ret;
    uint8_t data;
    ret = max77658_pm_read_reg(ctx, MAX77658_CNFG_GLBL, &data);
    if(ret == 0) return (data >> 5) & 0x01;
    return ERROR;
}

int32_t max77658_pm_get_nEN_MODE(max77658_pm_t *ctx)
{
    int32_t ret;
    uint8_t data;
    ret = max77658_pm_read_reg(ctx, MAX77658_CNFG_GLBL, &data);
    if(ret == 0) return (data >> 3) & 0x03; // 2 bits
    return ERROR;
}

int32_t max77658_pm_get_DBEN_nEN(max77658_pm_t *ctx)
{
    int32_t ret;
    uint8_t data;
    ret = max77658_pm_read_reg(ctx, MAX77658_CNFG_GLBL, &data);
    if(ret == 0) return (data >> 2) & 0x01; // Correction: Bit 2
    return ERROR;
}

int32_t max77658_pm_get_SFT_CTRL(max77658_pm_t *ctx)
{
    int32_t ret;
    uint8_t data;
    ret = max77658_pm_read_reg(ctx, MAX77658_CNFG_GLBL, &data);
    if(ret == 0) return data & 0x03;
    return ERROR;
}

/* GPIO0 */

int32_t max77658_pm_get_SBB_F_SHUTDN(max77658_pm_t *ctx)
{
    int32_t ret;
    uint8_t data;
    ret = max77658_pm_read_reg(ctx, MAX77658_CNFG_GPIO0, &data);
    if(ret == 0) return (data >> 7) & 0x01;
    return ERROR;
}

int32_t max77658_pm_get_ALT_GPIO0(max77658_pm_t *ctx)
{
    int32_t ret;
    uint8_t data;
    ret = max77658_pm_read_reg(ctx, MAX77658_CNFG_GPIO0, &data);
    if(ret == 0) return (data >> 5) & 0x01;
    return ERROR;
}

int32_t max77658_pm_get_DBEN_GPI_0(max77658_pm_t *ctx)
{
    int32_t ret;
    uint8_t data;
    ret = max77658_pm_read_reg(ctx, MAX77658_CNFG_GPIO0, &data);
    if(ret == 0) return (data >> 4) & 0x01;
    return ERROR;
}

int32_t max77658_pm_get_DO_0(max77658_pm_t *ctx)
{
    int32_t ret;
    uint8_t data;
    ret = max77658_pm_read_reg(ctx, MAX77658_CNFG_GPIO0, &data);
    if(ret == 0) return (data >> 3) & 0x01;
    return ERROR;
}

int32_t max77658_pm_get_DRV_0(max77658_pm_t *ctx)
{
    int32_t ret;
    uint8_t data;
    ret = max77658_pm_read_reg(ctx, MAX77658_CNFG_GPIO0, &data);
    if(ret == 0) return (data >> 2) & 0x01;
    return ERROR;
}

int32_t max77658_pm_get_DI_0(max77658_pm_t *ctx)
{
    int32_t ret;
    uint8_t data;
    ret = max77658_pm_read_reg(ctx, MAX77658_CNFG_GPIO0, &data);
    if(ret == 0) return (data >> 1) & 0x01;
    return ERROR;
}

int32_t max77658_pm_get_DIR_0(max77658_pm_t *ctx)
{
    int32_t ret;
    uint8_t data;
    ret = max77658_pm_read_reg(ctx, MAX77658_CNFG_GPIO0, &data);
    if(ret == 0) return data & 0x01;
    return ERROR;
}

/* GPIO1 */

int32_t max77658_pm_get_ALT_GPIO1(max77658_pm_t *ctx)
{
    int32_t ret;
    uint8_t data;
    ret = max77658_pm_read_reg(ctx, MAX77658_CNFG_GPIO1, &data);
    if(ret == 0) return (data >> 5) & 0x01;
    return ERROR;
}

int32_t max77658_pm_get_DBEN_GPI_1(max77658_pm_t *ctx)
{
    int32_t ret;
    uint8_t data;
    ret = max77658_pm_read_reg(ctx, MAX77658_CNFG_GPIO1, &data);
    if(ret == 0) return (data >> 4) & 0x01;
    return ERROR;
}

int32_t max77658_pm_get_DO_1(max77658_pm_t *ctx)
{
    int32_t ret;
    uint8_t data;
    ret = max77658_pm_read_reg(ctx, MAX77658_CNFG_GPIO1, &data);
    if(ret == 0) return (data >> 3) & 0x01;
    return ERROR;
}

int32_t max77658_pm_get_DRV_1(max77658_pm_t *ctx)
{
    int32_t ret;
    uint8_t data;
    ret = max77658_pm_read_reg(ctx, MAX77658_CNFG_GPIO1, &data);
    if(ret == 0) return (data >> 2) & 0x01;
    return ERROR;
}

int32_t max77658_pm_get_DI_1(max77658_pm_t *ctx)
{
    int32_t ret;
    uint8_t data;
    ret = max77658_pm_read_reg(ctx, MAX77658_CNFG_GPIO1, &data);
    if(ret == 0) return (data >> 1) & 0x01;
    return ERROR;
}

int32_t max77658_pm_get_DIR_1(max77658_pm_t *ctx)
{
    int32_t ret;
    uint8_t data;
    ret = max77658_pm_read_reg(ctx, MAX77658_CNFG_GPIO1, &data);
    if(ret == 0) return data & 0x01;
    return ERROR;
}

/* GPIO2 */

int32_t max77658_pm_get_ALT_GPIO2(max77658_pm_t *ctx)
{
    int32_t ret;
    uint8_t data;
    ret = max77658_pm_read_reg(ctx, MAX77658_CNFG_GPIO2, &data);
    if(ret == 0) return (data >> 5) & 0x01;
    return ERROR;
}

int32_t max77658_pm_get_DBEN_GPI_2(max77658_pm_t *ctx)
{
    int32_t ret;
    uint8_t data;
    ret = max77658_pm_read_reg(ctx, MAX77658_CNFG_GPIO2, &data);
    if(ret == 0) return (data >> 4) & 0x01;
    return ERROR;
}

int32_t max77658_pm_get_DO_2(max77658_pm_t *ctx)
{
    int32_t ret;
    uint8_t data;
    ret = max77658_pm_read_reg(ctx, MAX77658_CNFG_GPIO2, &data);
    if(ret == 0) return (data >> 3) & 0x01;
    return ERROR;
}

int32_t max77658_pm_get_DRV_2(max77658_pm_t *ctx)
{
    int32_t ret;
    uint8_t data;
    ret = max77658_pm_read_reg(ctx, MAX77658_CNFG_GPIO2, &data);
    if(ret == 0) return (data >> 2) & 0x01;
    return ERROR;
}

int32_t max77658_pm_get_DI_2(max77658_pm_t *ctx)
{
    int32_t ret;
    uint8_t data;
    ret = max77658_pm_read_reg(ctx, MAX77658_CNFG_GPIO2, &data);
    if(ret == 0) return (data >> 1) & 0x01;
    return ERROR;
}

int32_t max77658_pm_get_DIR_2(max77658_pm_t *ctx)
{
    int32_t ret;
    uint8_t data;
    ret = max77658_pm_read_reg(ctx, MAX77658_CNFG_GPIO2, &data);
    if(ret == 0) return data & 0x01;
    return ERROR;
}

int32_t max77658_pm_get_CID(max77658_pm_t *ctx)
{
    int32_t ret;
    uint8_t data;
    ret = max77658_pm_read_reg(ctx, MAX77658_CID, &data);
    if(ret == 0) return data & 0x0F;
    return ERROR;
}

/* WDT */

int32_t max77658_pm_get_WDT_PER(max77658_pm_t *ctx)
{
    int32_t ret;
    uint8_t data;
    ret = max77658_pm_read_reg(ctx, MAX77658_CNFG_WDT, &data);
    if(ret == 0) return (data >> 4) & 0x03;
    return ERROR;
}

int32_t max77658_pm_get_WDT_MODE(max77658_pm_t *ctx)
{
    int32_t ret;
    uint8_t data;
    ret = max77658_pm_read_reg(ctx, MAX77658_CNFG_WDT, &data);
    if(ret == 0) return (data >> 3) & 0x01;
    return ERROR;
}

int32_t max77658_pm_get_WDT_CLR(max77658_pm_t *ctx)
{
    int32_t ret;
    uint8_t data;
    ret = max77658_pm_read_reg(ctx, MAX77658_CNFG_WDT, &data);
    if(ret == 0) return (data >> 2) & 0x01;
    return ERROR;
}

int32_t max77658_pm_get_WDT_EN(max77658_pm_t *ctx)
{
    int32_t ret;
    uint8_t data;
    ret = max77658_pm_read_reg(ctx, MAX77658_CNFG_WDT, &data);
    if(ret == 0) return (data >> 1) & 0x01;
    return ERROR;
}

int32_t max77658_pm_get_WDT_LOCK(max77658_pm_t *ctx)
{
    int32_t ret;
    uint8_t data;
    ret = max77658_pm_read_reg(ctx, MAX77658_CNFG_WDT, &data);
    if(ret == 0) return data & 0x01;
    return ERROR;
}

/* Charger Interrupts */

int32_t max77658_pm_get_SYS_CNFG_I(max77658_pm_t *ctx)
{
    int32_t ret;
    uint8_t data;
    ret = max77658_pm_read_reg(ctx, MAX77658_INT_CHG, &data);
    if(ret == 0) return (data >> 6) & 0x01;
    return ERROR;
}

int32_t max77658_pm_get_SYS_CTRL_I(max77658_pm_t *ctx)
{
    int32_t ret;
    uint8_t data;
    ret = max77658_pm_read_reg(ctx, MAX77658_INT_CHG, &data);
    if(ret == 0) return (data >> 5) & 0x01;
    return ERROR;
}

int32_t max77658_pm_get_CHGIN_CTRL_I(max77658_pm_t *ctx)
{
    int32_t ret;
    uint8_t data;
    ret = max77658_pm_read_reg(ctx, MAX77658_INT_CHG, &data);
    if(ret == 0) return (data >> 4) & 0x01;
    return ERROR;
}

int32_t max77658_pm_get_TJ_REG_I(max77658_pm_t *ctx)
{
    int32_t ret;
    uint8_t data;
    ret = max77658_pm_read_reg(ctx, MAX77658_INT_CHG, &data);
    if(ret == 0) return (data >> 3) & 0x01;
    return ERROR;
}

int32_t max77658_pm_get_CHGIN_I(max77658_pm_t *ctx)
{
    int32_t ret;
    uint8_t data;
    ret = max77658_pm_read_reg(ctx, MAX77658_INT_CHG, &data);
    if(ret == 0) return (data >> 2) & 0x01;
    return ERROR;
}

int32_t max77658_pm_get_CHG_I(max77658_pm_t *ctx)
{
    int32_t ret;
    uint8_t data;
    ret = max77658_pm_read_reg(ctx, MAX77658_INT_CHG, &data);
    if(ret == 0) return (data >> 1) & 0x01;
    return ERROR;
}

int32_t max77658_pm_get_THM_I(max77658_pm_t *ctx)
{
    int32_t ret;
    uint8_t data;
    ret = max77658_pm_read_reg(ctx, MAX77658_INT_CHG, &data);
    if(ret == 0) return data & 0x01;
    return ERROR;
}

/* Charger Status */

int32_t max77658_pm_get_VCHGIN_MIN_STAT(max77658_pm_t *ctx)
{
    int32_t ret;
    uint8_t data;
    ret = max77658_pm_read_reg(ctx, MAX77658_STAT_CHG_A, &data);
    if(ret == 0) return (data >> 6) & 0x01;
    return ERROR;
}

int32_t max77658_pm_get_ICHGIN_LIM_STAT(max77658_pm_t *ctx)
{
    int32_t ret;
    uint8_t data;
    ret = max77658_pm_read_reg(ctx, MAX77658_STAT_CHG_A, &data);
    if(ret == 0) return (data >> 5) & 0x01;
    return ERROR;
}

int32_t max77658_pm_get_VSYS_MIN_STAT(max77658_pm_t *ctx)
{
    int32_t ret;
    uint8_t data;
    ret = max77658_pm_read_reg(ctx, MAX77658_STAT_CHG_A, &data);
    if(ret == 0) return (data >> 4) & 0x01;
    return ERROR;
}

int32_t max77658_pm_get_TJ_REG_STAT(max77658_pm_t *ctx)
{
    int32_t ret;
    uint8_t data;
    ret = max77658_pm_read_reg(ctx, MAX77658_STAT_CHG_A, &data);
    if(ret == 0) return (data >> 3) & 0x01;
    return ERROR;
}

int32_t max77658_pm_get_THM_DTLS(max77658_pm_t *ctx)
{
    int32_t ret;
    uint8_t data;
    ret = max77658_pm_read_reg(ctx, MAX77658_STAT_CHG_A, &data);
    if(ret == 0) return (data >> 0) & 0x07; // 3 bits at pos 0-2
    return ERROR;
}

int32_t max77658_pm_get_CHG_DTLS(max77658_pm_t *ctx)
{
    int32_t ret;
    uint8_t data;
    ret = max77658_pm_read_reg(ctx, MAX77658_STAT_CHG_B, &data);
    if(ret == 0) return (data >> 4) & 0x0F;
    return ERROR;
}

int32_t max77658_pm_get_CHGIN_DTLS(max77658_pm_t *ctx)
{
    int32_t ret;
    uint8_t data;
    ret = max77658_pm_read_reg(ctx, MAX77658_STAT_CHG_B, &data);
    if(ret == 0) return (data >> 2) & 0x03;
    return ERROR;
}

int32_t max77658_pm_get_CHG(max77658_pm_t *ctx)
{
    int32_t ret;
    uint8_t data;
    ret = max77658_pm_read_reg(ctx, MAX77658_STAT_CHG_B, &data);
    if(ret == 0) return (data >> 1) & 0x01;
    return ERROR;
}

int32_t max77658_pm_get_TIME_SUS(max77658_pm_t *ctx)
{
    int32_t ret;
    uint8_t data;
    ret = max77658_pm_read_reg(ctx, MAX77658_STAT_CHG_B, &data);
    if(ret == 0) return data & 0x01;
    return ERROR;
}

int32_t max77658_pm_get_INT_M_CHG(max77658_pm_t *ctx)
{
    int32_t ret;
    uint8_t data;
    ret = max77658_pm_read_reg(ctx, MAX77658_INT_M_CHG, &data);
    if(ret == 0) return data;
    return ERROR;
}

/* Charger Configs */

int32_t max77658_pm_get_THM_HOT(max77658_pm_t *ctx)
{
    int32_t ret;
    uint8_t data;
    ret = max77658_pm_read_reg(ctx, MAX77658_CNFG_CHG_A, &data);
    if(ret == 0) return (data >> 6) & 0x03;
    return ERROR;
}

int32_t max77658_pm_get_THM_WARM(max77658_pm_t *ctx)
{
    int32_t ret;
    uint8_t data;
    ret = max77658_pm_read_reg(ctx, MAX77658_CNFG_CHG_A, &data);
    if(ret == 0) return (data >> 4) & 0x03;
    return ERROR;
}

int32_t max77658_pm_get_THM_COOL(max77658_pm_t *ctx)
{
    int32_t ret;
    uint8_t data;
    ret = max77658_pm_read_reg(ctx, MAX77658_CNFG_CHG_A, &data);
    if(ret == 0) return (data >> 2) & 0x03;
    return ERROR;
}

int32_t max77658_pm_get_THM_COLD(max77658_pm_t *ctx)
{
    int32_t ret;
    uint8_t data;
    ret = max77658_pm_read_reg(ctx, MAX77658_CNFG_CHG_A, &data);
    if(ret == 0) return data & 0x03;
    return ERROR;
}

int32_t max77658_pm_get_VCHGIN_MIN(max77658_pm_t *ctx)
{
    int32_t ret;
    uint8_t data;
    ret = max77658_pm_read_reg(ctx, MAX77658_CNFG_CHG_B, &data);
    if(ret == 0) return (data >> 5) & 0x07;
    return ERROR;
}

int32_t max77658_pm_get_ICHGIN_LIM(max77658_pm_t *ctx)
{
    int32_t ret;
    uint8_t data;
    ret = max77658_pm_read_reg(ctx, MAX77658_CNFG_CHG_B, &data);
    if(ret == 0) return (data >> 2) & 0x07;
    return ERROR;
}

int32_t max77658_pm_get_I_PQ(max77658_pm_t *ctx)
{
    int32_t ret;
    uint8_t data;
    ret = max77658_pm_read_reg(ctx, MAX77658_CNFG_CHG_B, &data);
    if(ret == 0) return (data >> 1) & 0x01; // Corrected: shifted down 1
    return ERROR;
}

int32_t max77658_pm_get_CHG_EN(max77658_pm_t *ctx)
{
    int32_t ret;
    uint8_t data;
    ret = max77658_pm_read_reg(ctx, MAX77658_CNFG_CHG_B, &data);
    if(ret == 0) return data & 0x01;
    return ERROR;
}

int32_t max77658_pm_get_CHG_PQ(max77658_pm_t *ctx)
{
    int32_t ret;
    uint8_t data;
    ret = max77658_pm_read_reg(ctx, MAX77658_CNFG_CHG_C, &data);
    if(ret == 0) return (data >> 5) & 0x07;
    return ERROR;
}

int32_t max77658_pm_get_I_TERM(max77658_pm_t *ctx)
{
    int32_t ret;
    uint8_t data;
    ret = max77658_pm_read_reg(ctx, MAX77658_CNFG_CHG_C, &data);
    if(ret == 0) return (data >> 3) & 0x03;
    return ERROR;
}

int32_t max77658_pm_get_T_TOPOFF(max77658_pm_t *ctx)
{
    int32_t ret;
    uint8_t data;
    ret = max77658_pm_read_reg(ctx, MAX77658_CNFG_CHG_C, &data);
    if(ret == 0) return data & 0x07;
    return ERROR;
}

int32_t max77658_pm_get_TJ_REG(max77658_pm_t *ctx)
{
    int32_t ret;
    uint8_t data;
    ret = max77658_pm_read_reg(ctx, MAX77658_CNFG_CHG_D, &data);
    if(ret == 0) return (data >> 5) & 0x07;
    return ERROR;
}

int32_t max77658_pm_get_VSYS_REG(max77658_pm_t *ctx)
{
    int32_t ret;
    uint8_t data;
    ret = max77658_pm_read_reg(ctx, MAX77658_CNFG_CHG_D, &data);
    if(ret == 0) return data & 0x1F;
    return ERROR;
}

int32_t max77658_pm_get_CHG_CC(max77658_pm_t *ctx)
{
    int32_t ret;
    uint8_t data;
    ret = max77658_pm_read_reg(ctx, MAX77658_CNFG_CHG_E, &data);
    if(ret == 0) return (data >> 2) & 0x3F;
    return ERROR;
}

int32_t max77658_pm_get_T_FAST_CHG(max77658_pm_t *ctx)
{
    int32_t ret;
    uint8_t data;
    ret = max77658_pm_read_reg(ctx, MAX77658_CNFG_CHG_E, &data);
    if(ret == 0) return data & 0x03;
    return ERROR;
}

int32_t max77658_pm_get_CHG_CC_JEITA(max77658_pm_t *ctx)
{
    int32_t ret;
    uint8_t data;
    ret = max77658_pm_read_reg(ctx, MAX77658_CNFG_CHG_F, &data);
    if(ret == 0) return (data >> 2) & 0x3F;
    return ERROR;
}

int32_t max77658_pm_get_CHG_CV(max77658_pm_t *ctx)
{
    int32_t ret;
    uint8_t data;
    ret = max77658_pm_read_reg(ctx, MAX77658_CNFG_CHG_G, &data);
    if(ret == 0) return (data >> 2) & 0x3F;
    return ERROR;
}

int32_t max77658_pm_get_USBS(max77658_pm_t *ctx)
{
    int32_t ret;
    uint8_t data;
    ret = max77658_pm_read_reg(ctx, MAX77658_CNFG_CHG_G, &data);
    if(ret == 0) return (data >> 1) & 0x01;
    return ERROR;
}

int32_t max77658_pm_get_FUS_M(max77658_pm_t *ctx)
{
    int32_t ret;
    uint8_t data;
    ret = max77658_pm_read_reg(ctx, MAX77658_CNFG_CHG_G, &data);
    if(ret == 0) return data & 0x01;
    return ERROR;
}

int32_t max77658_pm_get_CHG_CV_JEITA(max77658_pm_t *ctx)
{
    int32_t ret;
    uint8_t data;
    ret = max77658_pm_read_reg(ctx, MAX77658_CNFG_CHG_H, &data);
    if(ret == 0) return (data >> 2) & 0x3F;
    return ERROR;
}

int32_t max77658_pm_get_SYS_BAT_PRT(max77658_pm_t *ctx)
{
    int32_t ret;
    uint8_t data;
    ret = max77658_pm_read_reg(ctx, MAX77658_CNFG_CHG_H, &data);
    if(ret == 0) return (data >> 1) & 0x01;
    return ERROR;
}

int32_t max77658_pm_get_CHR_TH_EN(max77658_pm_t *ctx)
{
    int32_t ret;
    uint8_t data;
    ret = max77658_pm_read_reg(ctx, MAX77658_CNFG_CHG_H, &data);
    if(ret == 0) return data & 0x01;
    return ERROR;
}

int32_t max77658_pm_get_IMON_DISCHG_SCALE(max77658_pm_t *ctx)
{
    int32_t ret;
    uint8_t data;
    ret = max77658_pm_read_reg(ctx, MAX77658_CNFG_CHG_I, &data);
    if(ret == 0) return (data >> 4) & 0x0F;
    return ERROR;
}

int32_t max77658_pm_get_MUX_SEL(max77658_pm_t *ctx)
{
    int32_t ret;
    uint8_t data;
    ret = max77658_pm_read_reg(ctx, MAX77658_CNFG_CHG_I, &data);
    if(ret == 0) return data & 0x0F;
    return ERROR;
}

/* SBB / LDO */

int32_t max77658_pm_get_DIS_LPM(max77658_pm_t *ctx)
{
    int32_t ret;
    uint8_t data;
    ret = max77658_pm_read_reg(ctx, MAX77658_CNFG_SBB_TOP, &data);
    if(ret == 0) return (data >> 7) & 0x01;
    return ERROR;
}

int32_t max77658_pm_get_IPK_1P5A(max77658_pm_t *ctx)
{
    int32_t ret;
    uint8_t data;
    ret = max77658_pm_read_reg(ctx, MAX77658_CNFG_SBB_TOP, &data);
    if(ret == 0) return (data >> 6) & 0x01;
    return ERROR;
}

int32_t max77658_pm_get_DRV_SBB(max77658_pm_t *ctx)
{
    int32_t ret;
    uint8_t data;
    ret = max77658_pm_read_reg(ctx, MAX77658_CNFG_SBB_TOP, &data);
    if(ret == 0) return data & 0x03;
    return ERROR;
}

int32_t max77658_pm_get_TV_SBB0(max77658_pm_t *ctx)
{
    int32_t ret;
    uint8_t data;
    ret = max77658_pm_read_reg(ctx, MAX77658_CNFG_SBB0_A, &data);
    if(ret == 0) return data;
    return ERROR;
}

int32_t max77658_pm_get_OP_MODE(max77658_pm_t *ctx)
{
    int32_t ret;
    uint8_t data;
    ret = max77658_pm_read_reg(ctx, MAX77658_CNFG_SBB0_B, &data);
    if(ret == 0) return (data >> 6) & 0x03;
    return ERROR;
}

int32_t max77658_pm_get_IP_SBB0(max77658_pm_t *ctx)
{
    int32_t ret;
    uint8_t data;
    ret = max77658_pm_read_reg(ctx, MAX77658_CNFG_SBB0_B, &data);
    if(ret == 0) return (data >> 4) & 0x03;
    return ERROR;
}

int32_t max77658_pm_get_ADE_SBB0(max77658_pm_t *ctx)
{
    int32_t ret;
    uint8_t data;
    ret = max77658_pm_read_reg(ctx, MAX77658_CNFG_SBB0_B, &data);
    if(ret == 0) return (data >> 3) & 0x01;
    return ERROR;
}

int32_t max77658_pm_get_EN_SBB0(max77658_pm_t *ctx)
{
    int32_t ret;
    uint8_t data;
    ret = max77658_pm_read_reg(ctx, MAX77658_CNFG_SBB0_B, &data);
    if(ret == 0) return data & 0x07;
    return ERROR;
}

int32_t max77658_pm_get_TV_SBB1(max77658_pm_t *ctx)
{
    int32_t ret;
    uint8_t data;
    ret = max77658_pm_read_reg(ctx, MAX77658_CNFG_SBB1_A, &data);
    if(ret == 0) return data;
    return ERROR;
}

int32_t max77658_pm_get_OP_MODE_1(max77658_pm_t *ctx)
{
    int32_t ret;
    uint8_t data;
    ret = max77658_pm_read_reg(ctx, MAX77658_CNFG_SBB1_B, &data);
    if(ret == 0) return (data >> 6) & 0x03;
    return ERROR;
}

int32_t max77658_pm_get_IP_SBB1(max77658_pm_t *ctx)
{
    int32_t ret;
    uint8_t data;
    ret = max77658_pm_read_reg(ctx, MAX77658_CNFG_SBB1_B, &data);
    if(ret == 0) return (data >> 4) & 0x03;
    return ERROR;
}

int32_t max77658_pm_get_ADE_SBB1(max77658_pm_t *ctx)
{
    int32_t ret;
    uint8_t data;
    ret = max77658_pm_read_reg(ctx, MAX77658_CNFG_SBB1_B, &data);
    if(ret == 0) return (data >> 3) & 0x01;
    return ERROR;
}

int32_t max77658_pm_get_EN_SBB1(max77658_pm_t *ctx)
{
    int32_t ret;
    uint8_t data;
    ret = max77658_pm_read_reg(ctx, MAX77658_CNFG_SBB1_B, &data);
    if(ret == 0) return data & 0x07;
    return ERROR;
}

int32_t max77658_pm_get_TV_SBB2(max77658_pm_t *ctx)
{
    int32_t ret;
    uint8_t data;
    ret = max77658_pm_read_reg(ctx, MAX77658_CNFG_SBB2_A, &data);
    if(ret == 0) return data;
    return ERROR;
}

int32_t max77658_pm_get_OP_MODE_2(max77658_pm_t *ctx)
{
    int32_t ret;
    uint8_t data;
    ret = max77658_pm_read_reg(ctx, MAX77658_CNFG_SBB2_B, &data);
    if(ret == 0) return (data >> 6) & 0x03;
    return ERROR;
}

int32_t max77658_pm_get_IP_SBB2(max77658_pm_t *ctx)
{
    int32_t ret;
    uint8_t data;
    ret = max77658_pm_read_reg(ctx, MAX77658_CNFG_SBB2_B, &data);
    if(ret == 0) return (data >> 4) & 0x03;
    return ERROR;
}

int32_t max77658_pm_get_ADE_SBB2(max77658_pm_t *ctx)
{
    int32_t ret;
    uint8_t data;
    ret = max77658_pm_read_reg(ctx, MAX77658_CNFG_SBB2_B, &data);
    if(ret == 0) return (data >> 3) & 0x01;
    return ERROR;
}

int32_t max77658_pm_get_EN_SBB2(max77658_pm_t *ctx)
{
    int32_t ret;
    uint8_t data;
    ret = max77658_pm_read_reg(ctx, MAX77658_CNFG_SBB2_B, &data);
    if(ret == 0) return data & 0x07;
    return ERROR;
}

int32_t max77658_pm_get_TV_SBB0_DVS (max77658_pm_t *ctx)
{
    int32_t ret;
    uint8_t data;
    ret = max77658_pm_read_reg(ctx, MAX77658_CNFG_DVS_SBB0_A, &data);
    if(ret == 0) return data;
    return ERROR;
}

int32_t max77658_pm_get_TV_OFS_LDO0(max77658_pm_t *ctx)
{
    int32_t ret;
    uint8_t data;
    ret = max77658_pm_read_reg(ctx, MAX77658_CNFG_LDO0_A, &data);
    if(ret == 0) return (data >> 7) & 0x01;
    return ERROR;
}

int32_t max77658_pm_get_TV_LDO0(max77658_pm_t *ctx)
{
    int32_t ret;
    uint8_t data;
    ret = max77658_pm_read_reg(ctx, MAX77658_CNFG_LDO0_A, &data);
    if(ret == 0) return data & 0x7F;
    return ERROR;
}

int32_t max77658_pm_get_LDO0_MD(max77658_pm_t *ctx)
{
    int32_t ret;
    uint8_t data;
    ret = max77658_pm_read_reg(ctx, MAX77658_CNFG_LDO0_B, &data);
    if(ret == 0) return (data >> 4) & 0x01;
    return ERROR;
}

int32_t max77658_pm_get_ADE_LDO0(max77658_pm_t *ctx)
{
    int32_t ret;
    uint8_t data;
    ret = max77658_pm_read_reg(ctx, MAX77658_CNFG_LDO0_B, &data);
    if(ret == 0) return (data >> 3) & 0x01;
    return ERROR;
}

int32_t max77658_pm_get_EN_LDO0(max77658_pm_t *ctx)
{
    int32_t ret;
    uint8_t data;
    ret = max77658_pm_read_reg(ctx, MAX77658_CNFG_LDO0_B, &data);
    if(ret == 0) return data & 0x07;
    return ERROR;
}

int32_t max77658_pm_get_TV_OFS_LDO1(max77658_pm_t *ctx)
{
    int32_t ret;
    uint8_t data;
    ret = max77658_pm_read_reg(ctx, MAX77658_CNFG_LDO1_A, &data);
    if(ret == 0) return (data >> 7) & 0x01;
    return ERROR;
}

int32_t max77658_pm_get_TV_LDO1(max77658_pm_t *ctx)
{
    int32_t ret;
    uint8_t data;
    ret = max77658_pm_read_reg(ctx, MAX77658_CNFG_LDO1_A, &data);
    if(ret == 0) return data & 0x7F;
    return ERROR;
}

int32_t max77658_pm_get_LDO1_MD(max77658_pm_t *ctx)
{
    int32_t ret;
    uint8_t data;
    ret = max77658_pm_read_reg(ctx, MAX77658_CNFG_LDO1_B, &data);
    if(ret == 0) return (data >> 4) & 0x01;
    return ERROR;
}

int32_t max77658_pm_get_ADE_LDO1(max77658_pm_t *ctx)
{
    int32_t ret;
    uint8_t data;
    ret = max77658_pm_read_reg(ctx, MAX77658_CNFG_LDO1_B, &data);
    if(ret == 0) return (data >> 3) & 0x01;
    return ERROR;
}

int32_t max77658_pm_get_EN_LDO1(max77658_pm_t *ctx)
{
    int32_t ret;
    uint8_t data;
    ret = max77658_pm_read_reg(ctx, MAX77658_CNFG_LDO1_B, &data);
    if(ret == 0) return data & 0x07;
    return ERROR;
}

/* ================================================================= */
/* ======================== WRITE FUNCTIONS ======================== */
/* ================================================================= */

int32_t max77658_pm_set_INTM_GLBL0(max77658_pm_t *ctx, uint8_t target_val)
{
    int32_t ret;
    uint8_t write_data[1];
    write_data[0] = target_val;
    ret = max77658_pm_write_reg(ctx, MAX77658_INTM_GLBL0, write_data);
    ret = (max77658_pm_get_INTM_GLBL0(ctx) == target_val)?SUCCESS:ERROR;
    return ret;
}

int32_t max77658_pm_set_INTM_GLBL1(max77658_pm_t *ctx, uint8_t target_val)
{
    int32_t ret;
    uint8_t write_data[1];
    write_data[0] = target_val;
    ret = max77658_pm_write_reg(ctx, MAX77658_INTM_GLBL1, write_data);
    ret = (max77658_pm_get_INTM_GLBL1(ctx) == target_val)?SUCCESS:ERROR;
    return ret;
}

int32_t max77658_pm_set_PU_DIS(max77658_pm_t *ctx, uint8_t target_val)
{
    int32_t ret;
    uint8_t curr_data;
    uint8_t write_data[1];
    ret = max77658_pm_read_reg(ctx, MAX77658_CNFG_GLBL, &curr_data);
    write_data[0] = (curr_data & 0x7F) | ((target_val & 0x01) << 7);
    ret = max77658_pm_write_reg(ctx, MAX77658_CNFG_GLBL, write_data);
    ret = (max77658_pm_get_PU_DIS(ctx) == target_val)?SUCCESS:ERROR;
    return ret;
}

int32_t max77658_pm_set_T_MRST(max77658_pm_t *ctx, uint8_t target_val)
{
    int32_t ret;
    uint8_t curr_data;
    uint8_t write_data[1];
    ret = max77658_pm_read_reg(ctx, MAX77658_CNFG_GLBL, &curr_data);
    write_data[0] = (curr_data & 0xBF) | ((target_val & 0x01) << 6);
    ret = max77658_pm_write_reg(ctx, MAX77658_CNFG_GLBL, write_data);
    ret = (max77658_pm_get_T_MRST(ctx) == target_val)?SUCCESS:ERROR;
    return ret;
}

int32_t max77658_pm_set_SBIA_LPM(max77658_pm_t *ctx, uint8_t target_val)
{
    int32_t ret;
    uint8_t curr_data = 0xFF;
    uint8_t write_data[1];
    ret = max77658_pm_read_reg(ctx, MAX77658_CNFG_GLBL, &curr_data);
    write_data[0]  = (curr_data & 0xDF) | ((target_val & 0x01) << 5);
    ret = max77658_pm_write_reg(ctx, MAX77658_CNFG_GLBL, write_data);
    ret = (max77658_pm_get_SBIA_LPM(ctx) == target_val)?SUCCESS:ERROR;
    return ret;
}

int32_t max77658_pm_set_nEN_MODE(max77658_pm_t *ctx, uint8_t target_val)
{
    int32_t ret;
    uint8_t curr_data;
    uint8_t write_data[1];
    ret = max77658_pm_read_reg(ctx, MAX77658_CNFG_GLBL, &curr_data);
    write_data[0] = (curr_data & 0xE7) | ((target_val & 0x03) << 3);
    ret = max77658_pm_write_reg(ctx, MAX77658_CNFG_GLBL, write_data);
    ret = (max77658_pm_get_nEN_MODE(ctx) == target_val)?SUCCESS:ERROR;
    return ret;
}

int32_t max77658_pm_set_DBEN_nEN(max77658_pm_t *ctx, uint8_t target_val)
{
    int32_t ret;
    uint8_t curr_data;
    uint8_t write_data[1];
    ret = max77658_pm_read_reg(ctx, MAX77658_CNFG_GLBL, &curr_data);
    write_data[0] = (curr_data & 0xFB) | ((target_val & 0x01) << 2);
    ret = max77658_pm_write_reg(ctx, MAX77658_CNFG_GLBL, write_data);
    ret = (max77658_pm_get_DBEN_nEN(ctx) == target_val)?SUCCESS:ERROR;
    return ret;
}

/**
  * @brief  Set SFT_CTRL without readback verification
  * @note   Use this for SFT_OFF (0x02) to avoid I2C hang when rails are cut
  * @param  ctx: pointer to max77658_pm context
  * @param  target_val: value to write (0-3)
  * @retval SUCCESS or ERROR
  */
int32_t max77658_pm_set_SFT_CTRL_novfy(max77658_pm_t *ctx, uint8_t target_val)
{
    int32_t ret;
    uint8_t curr;
    uint8_t wr;

    ret = max77658_pm_read_reg(ctx, MAX77658_CNFG_GLBL, &curr);
    if (ret < 0) return ret;

    wr = (uint8_t)((curr & 0xFCu) | (target_val & 0x03u));
    ret = max77658_pm_write_reg(ctx, MAX77658_CNFG_GLBL, &wr);
    return ret; // NO read-back verification
}

int32_t max77658_pm_set_SFT_CTRL(max77658_pm_t *ctx, uint8_t target_val)
{
    int32_t ret;
    uint8_t curr_data;
    uint8_t write_data[1];
    ret = max77658_pm_read_reg(ctx, MAX77658_CNFG_GLBL, &curr_data);
    write_data[0] = (curr_data & 0xFC) | ((target_val & 0x03) << 0);
    ret = max77658_pm_write_reg(ctx, MAX77658_CNFG_GLBL, write_data);
    ret = (max77658_pm_get_SFT_CTRL(ctx) == target_val)?SUCCESS:ERROR;
    return ret;
}

int32_t max77658_pm_set_SBB_F_SHUTDN(max77658_pm_t *ctx, uint8_t target_val)
{
    int32_t ret;
    uint8_t curr_data;
    uint8_t write_data[1];
    ret = max77658_pm_read_reg(ctx, MAX77658_CNFG_GPIO0, &curr_data);
    write_data[0] = (curr_data & 0x7F) | ((target_val & 0x01) << 7);
    ret = max77658_pm_write_reg(ctx, MAX77658_CNFG_GPIO0, write_data);
    ret = (max77658_pm_get_SBB_F_SHUTDN(ctx) == target_val)?SUCCESS:ERROR;
    return ret;
}

int32_t max77658_pm_set_ALT_GPIO0(max77658_pm_t *ctx, uint8_t target_val)
{
    int32_t ret;
    uint8_t curr_data;
    uint8_t write_data[1];
    ret = max77658_pm_read_reg(ctx, MAX77658_CNFG_GPIO0, &curr_data);
    write_data[0] = (curr_data & 0xDF) | ((target_val & 0x01) << 5);
    ret = max77658_pm_write_reg(ctx, MAX77658_CNFG_GPIO0, write_data);
    ret = (max77658_pm_get_ALT_GPIO0(ctx) == target_val)?SUCCESS:ERROR;
    return ret;
}

int32_t max77658_pm_set_DBEN_GPI_0(max77658_pm_t *ctx, uint8_t target_val)
{
    int32_t ret;
    uint8_t curr_data;
    uint8_t write_data[1];
    ret = max77658_pm_read_reg(ctx, MAX77658_CNFG_GPIO0, &curr_data);
    write_data[0] = (curr_data & 0xEF) | ((target_val & 0x01) << 4);
    ret = max77658_pm_write_reg(ctx, MAX77658_CNFG_GPIO0, write_data);
    ret = (max77658_pm_get_DBEN_GPI_0(ctx) == target_val)?SUCCESS:ERROR;
    return ret;
}

int32_t max77658_pm_set_DO_0(max77658_pm_t *ctx, uint8_t target_val)
{
    int32_t ret;
    uint8_t curr_data;
    uint8_t write_data[1];
    ret = max77658_pm_read_reg(ctx, MAX77658_CNFG_GPIO0, &curr_data);
    write_data[0] = (curr_data & 0xF7) | ((target_val & 0x01) << 3);
    ret = max77658_pm_write_reg(ctx, MAX77658_CNFG_GPIO0, write_data);
    ret = (max77658_pm_get_DO_0(ctx) == target_val)?SUCCESS:ERROR;
    return ret;
}

int32_t max77658_pm_set_DRV_0(max77658_pm_t *ctx, uint8_t target_val)
{
    int32_t ret;
    uint8_t curr_data;
    uint8_t write_data[1];
    ret = max77658_pm_read_reg(ctx, MAX77658_CNFG_GPIO0, &curr_data);
    write_data[0] = (curr_data & 0xFB) | ((target_val & 0x01) << 2);
    ret = max77658_pm_write_reg(ctx, MAX77658_CNFG_GPIO0, write_data);
    ret = (max77658_pm_get_DRV_0(ctx) == target_val)?SUCCESS:ERROR;
    return ret;
}

int32_t max77658_pm_set_DIR_0(max77658_pm_t *ctx, uint8_t target_val)
{
    int32_t ret;
    uint8_t curr_data;
    uint8_t write_data[1];
    ret = max77658_pm_read_reg(ctx, MAX77658_CNFG_GPIO0, &curr_data);
    write_data[0] = (curr_data & 0xFE) | ((target_val & 0x01) << 0);
    ret = max77658_pm_write_reg(ctx, MAX77658_CNFG_GPIO0, write_data);
    ret = (max77658_pm_get_DIR_0(ctx) == target_val)?SUCCESS:ERROR;
    return ret;
}

int32_t max77658_pm_set_ALT_GPIO1(max77658_pm_t *ctx, uint8_t target_val)
{
    int32_t ret;
    uint8_t curr_data;
    uint8_t write_data[1];
    ret = max77658_pm_read_reg(ctx, MAX77658_CNFG_GPIO1, &curr_data);
    write_data[0] = (curr_data & 0xDF) | ((target_val & 0x01) << 5);
    ret = max77658_pm_write_reg(ctx, MAX77658_CNFG_GPIO1, write_data);
    ret = (max77658_pm_get_ALT_GPIO1(ctx) == target_val)?SUCCESS:ERROR;
    return ret;
}

int32_t max77658_pm_set_DBEN_GPI_1(max77658_pm_t *ctx, uint8_t target_val)
{
    int32_t ret;
    uint8_t curr_data;
    uint8_t write_data[1];
    ret = max77658_pm_read_reg(ctx, MAX77658_CNFG_GPIO1, &curr_data);
    write_data[0] = (curr_data & 0xEF) | ((target_val & 0x01) << 4);
    ret = max77658_pm_write_reg(ctx, MAX77658_CNFG_GPIO1, write_data);
    ret = (max77658_pm_get_DBEN_GPI_1(ctx) == target_val)?SUCCESS:ERROR;
    return ret;
}

int32_t max77658_pm_set_DO_1(max77658_pm_t *ctx, uint8_t target_val)
{
    int32_t ret;
    uint8_t curr_data;
    uint8_t write_data[1];
    ret = max77658_pm_read_reg(ctx, MAX77658_CNFG_GPIO1, &curr_data);
    write_data[0] = (curr_data & 0xF7) | ((target_val & 0x01) << 3);
    ret = max77658_pm_write_reg(ctx, MAX77658_CNFG_GPIO1, write_data);
    ret = (max77658_pm_get_DO_1(ctx) == target_val)?SUCCESS:ERROR;
    return ret;
}

int32_t max77658_pm_set_DRV_1(max77658_pm_t *ctx, uint8_t target_val)
{
    int32_t ret;
    uint8_t curr_data;
    uint8_t write_data[1];
    ret = max77658_pm_read_reg(ctx, MAX77658_CNFG_GPIO1, &curr_data);
    write_data[0] = (curr_data & 0xFB) | ((target_val & 0x01) << 2);
    ret = max77658_pm_write_reg(ctx, MAX77658_CNFG_GPIO1, write_data);
    ret = (max77658_pm_get_DRV_1(ctx) == target_val)?SUCCESS:ERROR;
    return ret;
}

int32_t max77658_pm_set_DIR_1(max77658_pm_t *ctx, uint8_t target_val)
{
    int32_t ret;
    uint8_t curr_data;
    uint8_t write_data[1];
    ret = max77658_pm_read_reg(ctx, MAX77658_CNFG_GPIO1, &curr_data);
    write_data[0] = (curr_data & 0xFE) | ((target_val & 0x01) << 0);
    ret = max77658_pm_write_reg(ctx, MAX77658_CNFG_GPIO1, write_data);
    ret = (max77658_pm_get_DIR_1(ctx) == target_val)?SUCCESS:ERROR;
    return ret;
}

int32_t max77658_pm_set_ALT_GPIO2(max77658_pm_t *ctx, uint8_t target_val)
{
    int32_t ret;
    uint8_t curr_data;
    uint8_t write_data[1];
    ret = max77658_pm_read_reg(ctx, MAX77658_CNFG_GPIO2, &curr_data);
    write_data[0] = (curr_data & 0xDF) | ((target_val & 0x01) << 5);
    ret = max77658_pm_write_reg(ctx, MAX77658_CNFG_GPIO2, write_data);
    ret = (max77658_pm_get_ALT_GPIO2(ctx) == target_val)?SUCCESS:ERROR;
    return ret;
}

int32_t max77658_pm_set_DBEN_GPI_2(max77658_pm_t *ctx, uint8_t target_val)
{
    int32_t ret;
    uint8_t curr_data;
    uint8_t write_data[1];
    ret = max77658_pm_read_reg(ctx, MAX77658_CNFG_GPIO2, &curr_data);
    write_data[0] = (curr_data & 0xEF) | ((target_val & 0x01) << 4);
    ret = max77658_pm_write_reg(ctx, MAX77658_CNFG_GPIO2, write_data);
    ret = (max77658_pm_get_DBEN_GPI_2(ctx) == target_val)?SUCCESS:ERROR;
    return ret;
}

int32_t max77658_pm_set_DO_2(max77658_pm_t *ctx, uint8_t target_val)
{
    int32_t ret;
    uint8_t curr_data;
    uint8_t write_data[1];
    ret = max77658_pm_read_reg(ctx, MAX77658_CNFG_GPIO2, &curr_data);
    write_data[0] = (curr_data & 0xF7) | ((target_val & 0x01) << 3);
    ret = max77658_pm_write_reg(ctx, MAX77658_CNFG_GPIO2, write_data);
    ret = (max77658_pm_get_DO_2(ctx) == target_val)?SUCCESS:ERROR;
    return ret;
}

int32_t max77658_pm_set_DRV_2(max77658_pm_t *ctx, uint8_t target_val)
{
    int32_t ret;
    uint8_t curr_data;
    uint8_t write_data[1];
    ret = max77658_pm_read_reg(ctx, MAX77658_CNFG_GPIO2, &curr_data);
    write_data[0] = (curr_data & 0xFB) | ((target_val & 0x01) << 2);
    ret = max77658_pm_write_reg(ctx, MAX77658_CNFG_GPIO2, write_data);
    ret = (max77658_pm_get_DRV_2(ctx) == target_val)?SUCCESS:ERROR;
    return ret;
}

int32_t max77658_pm_set_DIR_2(max77658_pm_t *ctx, uint8_t target_val)
{
    int32_t ret;
    uint8_t curr_data;
    uint8_t write_data[1];
    ret = max77658_pm_read_reg(ctx, MAX77658_CNFG_GPIO2, &curr_data);
    write_data[0] = (curr_data & 0xFE) | ((target_val & 0x01) << 0);
    ret = max77658_pm_write_reg(ctx, MAX77658_CNFG_GPIO2, write_data);
    ret = (max77658_pm_get_DIR_2(ctx) == target_val)?SUCCESS:ERROR;
    return ret;
}

int32_t max77658_pm_set_WDT_PER(max77658_pm_t *ctx, uint8_t target_val)
{
    int32_t ret;
    uint8_t curr_data;
    uint8_t write_data[1];
    ret = max77658_pm_read_reg(ctx, MAX77658_CNFG_WDT, &curr_data);
    write_data[0] = (curr_data & 0xCF) | ((target_val & 0x03) << 4);
    ret = max77658_pm_write_reg(ctx, MAX77658_CNFG_WDT, write_data);
    ret = (max77658_pm_get_WDT_PER(ctx) == target_val)?SUCCESS:ERROR;
    return ret;
}

int32_t max77658_pm_set_WDT_MODE(max77658_pm_t *ctx, uint8_t target_val)
{
    int32_t ret;
    uint8_t curr_data;
    uint8_t write_data[1];
    ret = max77658_pm_read_reg(ctx, MAX77658_CNFG_WDT, &curr_data);
    write_data[0] = (curr_data & 0xF7) | ((target_val & 0x01) << 3);
    ret = max77658_pm_write_reg(ctx, MAX77658_CNFG_WDT, write_data);
    ret = (max77658_pm_get_WDT_MODE(ctx) == target_val)?SUCCESS:ERROR;
    return ret;
}

int32_t max77658_pm_set_WDT_CLR(max77658_pm_t *ctx, uint8_t target_val)
{
    int32_t ret;
    uint8_t curr_data;
    uint8_t write_data[1];
    ret = max77658_pm_read_reg(ctx, MAX77658_CNFG_WDT, &curr_data);
    write_data[0] = (curr_data & 0xFB) | ((target_val & 0x01) << 2);
    ret = max77658_pm_write_reg(ctx, MAX77658_CNFG_WDT, write_data);
    ret = (max77658_pm_get_WDT_CLR(ctx) == target_val)?SUCCESS:ERROR;
    return ret;
}

int32_t max77658_pm_set_WDT_EN(max77658_pm_t *ctx, uint8_t target_val)
{
    int32_t ret;
    uint8_t curr_data;
    uint8_t write_data[1];
    ret = max77658_pm_read_reg(ctx, MAX77658_CNFG_WDT, &curr_data);
    write_data[0] = (curr_data & 0xFD) | ((target_val & 0x01) << 1);
    ret = max77658_pm_write_reg(ctx, MAX77658_CNFG_WDT, write_data);
    ret = (max77658_pm_get_WDT_EN(ctx) == target_val)?SUCCESS:ERROR;
    return ret;
}

int32_t max77658_pm_set_INT_M_CHG(max77658_pm_t *ctx, uint8_t target_val)
{
    int32_t ret;
    uint8_t write_data[1];
    write_data[0] = target_val;
    ret = max77658_pm_write_reg(ctx, MAX77658_INT_M_CHG, write_data);
    ret = (max77658_pm_get_INT_M_CHG(ctx) == target_val)?SUCCESS:ERROR;
    return ret;
}

int32_t max77658_pm_set_THM_HOT(max77658_pm_t *ctx, uint8_t target_val)
{
    int32_t ret;
    uint8_t curr_data;
    uint8_t write_data[1];
    ret = max77658_pm_read_reg(ctx, MAX77658_CNFG_CHG_A, &curr_data);
    write_data[0] = (curr_data & 0x3F) | ((target_val & 0x03) << 6);
    ret = max77658_pm_write_reg(ctx, MAX77658_CNFG_CHG_A, write_data);
    ret = (max77658_pm_get_THM_HOT(ctx) == target_val)?SUCCESS:ERROR;
    return ret;
}

int32_t max77658_pm_set_THM_WARM(max77658_pm_t *ctx, uint8_t target_val)
{
    int32_t ret;
    uint8_t curr_data;
    uint8_t write_data[1];
    ret = max77658_pm_read_reg(ctx, MAX77658_CNFG_CHG_A, &curr_data);
    write_data[0] = (curr_data & 0xCF) | ((target_val & 0x03) << 4);
    ret = max77658_pm_write_reg(ctx, MAX77658_CNFG_CHG_A, write_data);
    ret = (max77658_pm_get_THM_WARM(ctx) == target_val)?SUCCESS:ERROR;
    return ret;
}

int32_t max77658_pm_set_THM_COOL(max77658_pm_t *ctx, uint8_t target_val)
{
    int32_t ret;
    uint8_t curr_data;
    uint8_t write_data[1];
    ret = max77658_pm_read_reg(ctx, MAX77658_CNFG_CHG_A, &curr_data);
    write_data[0] = (curr_data & 0xF3) | ((target_val & 0x03) << 2);
    ret = max77658_pm_write_reg(ctx, MAX77658_CNFG_CHG_A, write_data);
    ret = (max77658_pm_get_THM_COOL(ctx) == target_val)?SUCCESS:ERROR;
    return ret;
}

int32_t max77658_pm_set_THM_COLD(max77658_pm_t *ctx, uint8_t target_val)
{
    int32_t ret;
    uint8_t curr_data;
    uint8_t write_data[1];
    ret = max77658_pm_read_reg(ctx, MAX77658_CNFG_CHG_A, &curr_data);
    write_data[0] = (curr_data & 0xFC) | (target_val & 0x03);
    ret = max77658_pm_write_reg(ctx, MAX77658_CNFG_CHG_A, write_data);
    ret = (max77658_pm_get_THM_COLD(ctx) == target_val)?SUCCESS:ERROR;
    return ret;
}

int32_t max77658_pm_set_VCHGIN_MIN(max77658_pm_t *ctx, uint8_t target_val)
{
    int32_t ret;
    uint8_t curr_data;
    uint8_t write_data[1];
    ret = max77658_pm_read_reg(ctx, MAX77658_CNFG_CHG_B, &curr_data);
    write_data[0] = (curr_data & 0x1F) | ((target_val & 0x07) << 5);
    ret = max77658_pm_write_reg(ctx, MAX77658_CNFG_CHG_B, write_data);
    ret = (max77658_pm_get_VCHGIN_MIN(ctx) == target_val)?SUCCESS:ERROR;
    return ret;
}

int32_t max77658_pm_set_ICHGIN_LIM(max77658_pm_t *ctx, uint8_t target_val)
{
    int32_t ret;
    uint8_t curr_data;
    uint8_t write_data[1];
    ret = max77658_pm_read_reg(ctx, MAX77658_CNFG_CHG_B, &curr_data);
    write_data[0] = (curr_data & 0xE3) | ((target_val & 0x07) << 2);
    ret = max77658_pm_write_reg(ctx, MAX77658_CNFG_CHG_B, write_data);
    ret = (max77658_pm_get_ICHGIN_LIM(ctx) == target_val)?SUCCESS:ERROR;
    return ret;
}

int32_t max77658_pm_set_I_PQ(max77658_pm_t *ctx, uint8_t target_val)
{
    int32_t ret;
    uint8_t curr_data;
    uint8_t write_data[1];
    ret = max77658_pm_read_reg(ctx, MAX77658_CNFG_CHG_B, &curr_data);
    write_data[0] = (curr_data & 0xFD) | ((target_val & 0x01) << 1);
    ret = max77658_pm_write_reg(ctx, MAX77658_CNFG_CHG_B, write_data);
    ret = (max77658_pm_get_I_PQ(ctx) == target_val)?SUCCESS:ERROR;
    return ret;
}

int32_t max77658_pm_set_CHG_EN(max77658_pm_t *ctx, uint8_t target_val)
{
    int32_t ret;
    uint8_t curr_data;
    uint8_t write_data[1];
    ret = max77658_pm_read_reg(ctx, MAX77658_CNFG_CHG_B, &curr_data);
    write_data[0] = (curr_data & 0xFE) | (target_val & 0x01);
    ret = max77658_pm_write_reg(ctx, MAX77658_CNFG_CHG_B, write_data);
    ret = (max77658_pm_get_CHG_EN(ctx) == target_val)?SUCCESS:ERROR;
    return ret;
}

int32_t max77658_pm_set_CHG_PQ(max77658_pm_t *ctx, uint8_t target_val)
{
    int32_t ret;
    uint8_t curr_data;
    uint8_t write_data[1];
    ret = max77658_pm_read_reg(ctx, MAX77658_CNFG_CHG_C, &curr_data);
    write_data[0] = (curr_data & 0x1F) | ((target_val & 0x07) << 5);
    ret = max77658_pm_write_reg(ctx, MAX77658_CNFG_CHG_C, write_data);
    ret = (max77658_pm_get_CHG_PQ(ctx) == target_val)?SUCCESS:ERROR;
    return ret;
}

int32_t max77658_pm_set_I_TERM(max77658_pm_t *ctx, uint8_t target_val)
{
    int32_t ret;
    uint8_t curr_data;
    uint8_t write_data[1];
    ret = max77658_pm_read_reg(ctx, MAX77658_CNFG_CHG_C, &curr_data);
    write_data[0] = (curr_data & 0xE7) | ((target_val & 0x03) << 3);
    ret = max77658_pm_write_reg(ctx, MAX77658_CNFG_CHG_C, write_data);
    ret = (max77658_pm_get_I_TERM(ctx) == target_val)?SUCCESS:ERROR;
    return ret;
}

int32_t max77658_pm_set_T_TOPOFF(max77658_pm_t *ctx, uint8_t target_val)
{
    int32_t ret;
    uint8_t curr_data;
    uint8_t write_data[1];
    ret = max77658_pm_read_reg(ctx, MAX77658_CNFG_CHG_C, &curr_data);
    write_data[0] = (curr_data & 0xF8) | (target_val & 0x07);
    ret = max77658_pm_write_reg(ctx, MAX77658_CNFG_CHG_C, write_data);
    ret = (max77658_pm_get_T_TOPOFF(ctx) == target_val)?SUCCESS:ERROR;
    return ret;
}

int32_t max77658_pm_set_TJ_REG(max77658_pm_t *ctx, uint8_t target_val)
{
    int32_t ret;
    uint8_t curr_data;
    uint8_t write_data[1];
    ret = max77658_pm_read_reg(ctx, MAX77658_CNFG_CHG_D, &curr_data);
    write_data[0] = (curr_data & 0x1F) | ((target_val & 0x07) << 5);
    ret = max77658_pm_write_reg(ctx, MAX77658_CNFG_CHG_D, write_data);
    ret = (max77658_pm_get_TJ_REG(ctx) == target_val)?SUCCESS:ERROR;
    return ret;
}

int32_t max77658_pm_set_VSYS_REG(max77658_pm_t *ctx, uint8_t target_val)
{
    int32_t ret;
    uint8_t curr_data;
    uint8_t write_data[1];
    ret = max77658_pm_read_reg(ctx, MAX77658_CNFG_CHG_D, &curr_data);
    write_data[0] = (curr_data & 0xF0) | (target_val & 0x0F);
    ret = max77658_pm_write_reg(ctx, MAX77658_CNFG_CHG_D, write_data);
    ret = (max77658_pm_get_VSYS_REG(ctx) == target_val)?SUCCESS:ERROR;
    return ret;
}

int32_t max77658_pm_set_CHG_CC(max77658_pm_t *ctx, uint8_t target_val)
{
    int32_t ret;
    uint8_t curr_data;
    uint8_t write_data[1];
    ret = max77658_pm_read_reg(ctx, MAX77658_CNFG_CHG_E, &curr_data);
    write_data[0] = (curr_data & 0x03) | ((target_val & 0x3F) << 2);
    ret = max77658_pm_write_reg(ctx, MAX77658_CNFG_CHG_E, write_data);
    ret = (max77658_pm_get_CHG_CC(ctx) == target_val)?SUCCESS:ERROR;
    return ret;
}

int32_t max77658_pm_set_T_FAST_CHG(max77658_pm_t *ctx, uint8_t target_val)
{
    int32_t ret;
    uint8_t curr_data;
    uint8_t write_data[1];
    ret = max77658_pm_read_reg(ctx, MAX77658_CNFG_CHG_E, &curr_data);
    write_data[0] = (curr_data & 0xFC) | (target_val & 0x03);
    ret = max77658_pm_write_reg(ctx, MAX77658_CNFG_CHG_E, write_data);
    ret = (max77658_pm_get_T_FAST_CHG(ctx) == target_val)?SUCCESS:ERROR;
    return ret;
}

int32_t max77658_pm_set_CHG_CC_JEITA(max77658_pm_t *ctx, uint8_t target_val)
{
    int32_t ret;
    uint8_t curr_data;
    uint8_t write_data[1];
    ret = max77658_pm_read_reg(ctx, MAX77658_CNFG_CHG_F, &curr_data);
    write_data[0] = (curr_data & 0x03) | ((target_val & 0x3F) << 2);
    ret = max77658_pm_write_reg(ctx, MAX77658_CNFG_CHG_F, write_data);
    ret = (max77658_pm_get_CHG_CC_JEITA(ctx) == target_val)?SUCCESS:ERROR;
    return ret;
}

int32_t max77658_pm_set_CHG_CV(max77658_pm_t *ctx, uint8_t target_val)
{
    int32_t ret;
    uint8_t curr_data;
    uint8_t write_data[1];
    ret = max77658_pm_read_reg(ctx, MAX77658_CNFG_CHG_G, &curr_data);
    write_data[0] = (curr_data & 0x03) | ((target_val & 0x3F) << 2);
    ret = max77658_pm_write_reg(ctx, MAX77658_CNFG_CHG_G, write_data);
    ret = (max77658_pm_get_CHG_CV(ctx) == target_val)?SUCCESS:ERROR;
    return ret;
}

int32_t max77658_pm_set_USBS(max77658_pm_t *ctx, uint8_t target_val)
{
    int32_t ret;
    uint8_t curr_data;
    uint8_t write_data[1];
    ret = max77658_pm_read_reg(ctx, MAX77658_CNFG_CHG_G, &curr_data);
    write_data[0] = (curr_data & 0xFD) | ((target_val & 0x01) << 1);
    ret = max77658_pm_write_reg(ctx, MAX77658_CNFG_CHG_G, write_data);
    ret = (max77658_pm_get_USBS(ctx) == target_val)?SUCCESS:ERROR;
    return ret;
}

int32_t max77658_pm_set_FUS_M(max77658_pm_t *ctx, uint8_t target_val)
{
    int32_t ret;
    uint8_t curr_data;
    uint8_t write_data[1];
    ret = max77658_pm_read_reg(ctx, MAX77658_CNFG_CHG_G, &curr_data);
    write_data[0] = (curr_data & 0xFE) | (target_val & 0x01);
    ret = max77658_pm_write_reg(ctx, MAX77658_CNFG_CHG_G, write_data);
    ret = (max77658_pm_get_FUS_M(ctx) == target_val)?SUCCESS:ERROR;
    return ret;
}

int32_t max77658_pm_set_CHG_CV_JEITA(max77658_pm_t *ctx, uint8_t target_val)
{
    int32_t ret;
    uint8_t curr_data;
    uint8_t write_data[1];
    ret = max77658_pm_read_reg(ctx, MAX77658_CNFG_CHG_H, &curr_data);
    write_data[0] = (curr_data & 0x03) | ((target_val & 0x3F) << 2);
    ret = max77658_pm_write_reg(ctx, MAX77658_CNFG_CHG_H, write_data);
    ret = (max77658_pm_get_CHG_CV_JEITA(ctx) == target_val)?SUCCESS:ERROR;
    return ret;
}

int32_t max77658_pm_set_SYS_BAT_PRT(max77658_pm_t *ctx, uint8_t target_val)
{
    int32_t ret;
    uint8_t curr_data;
    uint8_t write_data[1];
    ret = max77658_pm_read_reg(ctx, MAX77658_CNFG_CHG_H, &curr_data);
    write_data[0] = (curr_data & 0xFD) | ((target_val & 0x01) << 1);
    ret = max77658_pm_write_reg(ctx, MAX77658_CNFG_CHG_H, write_data);
    ret = (max77658_pm_get_SYS_BAT_PRT(ctx) == target_val)?SUCCESS:ERROR;
    return ret;
}

int32_t max77658_pm_set_CHR_TH_EN(max77658_pm_t *ctx, uint8_t target_val)
{
    int32_t ret;
    uint8_t curr_data;
    uint8_t write_data[1];
    ret = max77658_pm_read_reg(ctx, MAX77658_CNFG_CHG_H, &curr_data);
    write_data[0] = (curr_data & 0xFE) | (target_val & 0x01);
    ret = max77658_pm_write_reg(ctx, MAX77658_CNFG_CHG_H, write_data);
    ret = (max77658_pm_get_CHR_TH_EN(ctx) == target_val)?SUCCESS:ERROR;
    return ret;
}

int32_t max77658_pm_set_IMON_DISCHG_SCALE(max77658_pm_t *ctx, uint8_t target_val)
{
    int32_t ret;
    uint8_t curr_data;
    uint8_t write_data[1];
    ret = max77658_pm_read_reg(ctx, MAX77658_CNFG_CHG_I, &curr_data);
    write_data[0] = (curr_data & 0x0F) | ((target_val & 0x0F) << 4);
    ret = max77658_pm_write_reg(ctx, MAX77658_CNFG_CHG_I, write_data);
    ret = (max77658_pm_get_IMON_DISCHG_SCALE(ctx) == target_val)?SUCCESS:ERROR;
    return ret;
}

int32_t max77658_pm_set_MUX_SEL(max77658_pm_t *ctx, uint8_t target_val)
{
    int32_t ret;
    uint8_t curr_data;
    uint8_t write_data[1];
    ret = max77658_pm_read_reg(ctx, MAX77658_CNFG_CHG_I, &curr_data);
    write_data[0] = (curr_data & 0xF0) | (target_val & 0x0F);
    ret = max77658_pm_write_reg(ctx, MAX77658_CNFG_CHG_I, write_data);
    ret = (max77658_pm_get_MUX_SEL(ctx) == target_val)?SUCCESS:ERROR;
    return ret;
}

int32_t max77658_pm_set_DIS_LPM(max77658_pm_t *ctx, uint8_t target_val)
{
    int32_t ret;
    uint8_t curr_data;
    uint8_t write_data[1];
    ret = max77658_pm_read_reg(ctx, MAX77658_CNFG_SBB_TOP, &curr_data);
    write_data[0] = (curr_data & 0x7F) | ((target_val & 0x01) << 7);
    ret = max77658_pm_write_reg(ctx, MAX77658_CNFG_SBB_TOP, write_data);
    ret = (max77658_pm_get_DIS_LPM(ctx) == target_val)?SUCCESS:ERROR;
    return ret;
}

int32_t max77658_pm_set_IPK_1P5A(max77658_pm_t *ctx, uint8_t target_val)
{
    int32_t ret;
    uint8_t curr_data;
    uint8_t write_data[1];
    ret = max77658_pm_read_reg(ctx, MAX77658_CNFG_SBB_TOP, &curr_data);
    write_data[0] = (curr_data & 0xBF) | ((target_val & 0x01) << 6);
    ret = max77658_pm_write_reg(ctx, MAX77658_CNFG_SBB_TOP, write_data);
    ret = (max77658_pm_get_IPK_1P5A(ctx) == target_val)?SUCCESS:ERROR;
    return ret;
}

int32_t max77658_pm_set_DRV_SBB(max77658_pm_t *ctx, uint8_t target_val)
{
    int32_t ret;
    uint8_t curr_data;
    uint8_t write_data[1];
    ret = max77658_pm_read_reg(ctx, MAX77658_CNFG_SBB_TOP, &curr_data);
    write_data[0] = (curr_data & 0xFC) | (target_val & 0x03);
    ret = max77658_pm_write_reg(ctx, MAX77658_CNFG_SBB_TOP, write_data);
    ret = (max77658_pm_get_DRV_SBB(ctx) == target_val)?SUCCESS:ERROR;
    return ret;
}

int32_t max77658_pm_set_TV_SBB0(max77658_pm_t *ctx, uint8_t target_val)
{
    int32_t ret;
    uint8_t write_data[1];
    write_data[0] = target_val;
    ret = max77658_pm_write_reg(ctx, MAX77658_CNFG_SBB0_A, write_data);
    ret = (max77658_pm_get_TV_SBB0(ctx) == target_val)?SUCCESS:ERROR;
    return ret;
}

int32_t max77658_pm_set_OP_MODE(max77658_pm_t *ctx, uint8_t target_val)
{
    int32_t ret;
    uint8_t curr_data;
    uint8_t write_data[1];
    ret = max77658_pm_read_reg(ctx, MAX77658_CNFG_SBB0_B, &curr_data);
    write_data[0] = (curr_data & 0x3F) | ((target_val & 0x03) << 6);
    ret = max77658_pm_write_reg(ctx, MAX77658_CNFG_SBB0_B, write_data);
    ret = (max77658_pm_get_OP_MODE(ctx) == target_val)?SUCCESS:ERROR;
    return ret;
}

int32_t max77658_pm_set_IP_SBB0(max77658_pm_t *ctx, uint8_t target_val)
{
    int32_t ret;
    uint8_t curr_data;
    uint8_t write_data[1];
    ret = max77658_pm_read_reg(ctx, MAX77658_CNFG_SBB0_B, &curr_data);
    write_data[0] = (curr_data & 0xCF) | ((target_val & 0x03) << 4);
    ret = max77658_pm_write_reg(ctx, MAX77658_CNFG_SBB0_B, write_data);
    ret = (max77658_pm_get_IP_SBB0(ctx) == target_val)?SUCCESS:ERROR;
    return ret;
}

int32_t max77658_pm_set_ADE_SBB0(max77658_pm_t *ctx, uint8_t target_val)
{
    int32_t ret;
    uint8_t curr_data;
    uint8_t write_data[1];
    ret = max77658_pm_read_reg(ctx, MAX77658_CNFG_SBB0_B, &curr_data);
    write_data[0] = (curr_data & 0xF7) | ((target_val & 0x01) << 3);
    ret = max77658_pm_write_reg(ctx, MAX77658_CNFG_SBB0_B, write_data);
    ret = (max77658_pm_get_ADE_SBB0(ctx) == target_val)?SUCCESS:ERROR;
    return ret;
}

int32_t max77658_pm_set_EN_SBB0(max77658_pm_t *ctx, uint8_t target_val)
{
    int32_t ret;
    uint8_t curr_data;
    uint8_t write_data[1];
    ret = max77658_pm_read_reg(ctx, MAX77658_CNFG_SBB0_B, &curr_data);
    write_data[0] = (curr_data & 0xF8) | (target_val & 0x07);
    ret = max77658_pm_write_reg(ctx, MAX77658_CNFG_SBB0_B, write_data);
    ret = (max77658_pm_get_EN_SBB0(ctx) == target_val)?SUCCESS:ERROR;
    return ret;
}

int32_t max77658_pm_set_TV_SBB1(max77658_pm_t *ctx, uint8_t target_val)
{
    int32_t ret;
    uint8_t write_data[1];
    write_data[0] = target_val;
    ret = max77658_pm_write_reg(ctx, MAX77658_CNFG_SBB1_A, write_data);
    ret = (max77658_pm_get_TV_SBB1(ctx) == target_val)?SUCCESS:ERROR;
    return ret;
}

int32_t max77658_pm_set_OP_MODE_1(max77658_pm_t *ctx, uint8_t target_val)
{
    int32_t ret;
    uint8_t curr_data;
    uint8_t write_data[1];
    ret = max77658_pm_read_reg(ctx, MAX77658_CNFG_SBB1_B, &curr_data);
    write_data[0] = (curr_data & 0x3F) | ((target_val & 0x03) << 6);
    ret = max77658_pm_write_reg(ctx, MAX77658_CNFG_SBB1_B, write_data);
    ret = (max77658_pm_get_OP_MODE_1(ctx) == target_val)?SUCCESS:ERROR;
    return ret;
}

int32_t max77658_pm_set_IP_SBB1(max77658_pm_t *ctx, uint8_t target_val)
{
    int32_t ret;
    uint8_t curr_data;
    uint8_t write_data[1];
    ret = max77658_pm_read_reg(ctx, MAX77658_CNFG_SBB1_B, &curr_data);
    write_data[0] = (curr_data & 0xCF) | ((target_val & 0x03) << 4);
    ret = max77658_pm_write_reg(ctx, MAX77658_CNFG_SBB1_B, write_data);
    ret = (max77658_pm_get_IP_SBB1(ctx) == target_val)?SUCCESS:ERROR;
    return ret;
}

int32_t max77658_pm_set_ADE_SBB1(max77658_pm_t *ctx, uint8_t target_val)
{
    int32_t ret;
    uint8_t curr_data;
    uint8_t write_data[1];
    ret = max77658_pm_read_reg(ctx, MAX77658_CNFG_SBB1_B, &curr_data);
    write_data[0] = (curr_data & 0xF7) | ((target_val & 0x01) << 3);
    ret = max77658_pm_write_reg(ctx, MAX77658_CNFG_SBB1_B, write_data);
    ret = (max77658_pm_get_ADE_SBB1(ctx) == target_val)?SUCCESS:ERROR;
    return ret;
}

int32_t max77658_pm_set_EN_SBB1(max77658_pm_t *ctx, uint8_t target_val)
{
    int32_t ret;
    uint8_t curr_data;
    uint8_t write_data[1];
    ret = max77658_pm_read_reg(ctx, MAX77658_CNFG_SBB1_B, &curr_data);
    write_data[0] = (curr_data & 0xF8) | (target_val & 0x07);
    ret = max77658_pm_write_reg(ctx, MAX77658_CNFG_SBB1_B, write_data);
    ret = (max77658_pm_get_EN_SBB1(ctx) == target_val)?SUCCESS:ERROR;
    return ret;
}

int32_t max77658_pm_set_TV_SBB2(max77658_pm_t *ctx, uint8_t target_val)
{
    int32_t ret;
    uint8_t write_data[1];
    write_data[0] = target_val;
    ret = max77658_pm_write_reg(ctx, MAX77658_CNFG_SBB2_A, write_data);
    ret = (max77658_pm_get_TV_SBB2(ctx) == target_val)?SUCCESS:ERROR;
    return ret;
}

int32_t max77658_pm_set_OP_MODE_2(max77658_pm_t *ctx, uint8_t target_val)
{
    int32_t ret;
    uint8_t curr_data;
    uint8_t write_data[1];
    ret = max77658_pm_read_reg(ctx, MAX77658_CNFG_SBB2_B, &curr_data);
    write_data[0] = (curr_data & 0x3F) | ((target_val & 0x03) << 6);
    ret = max77658_pm_write_reg(ctx, MAX77658_CNFG_SBB2_B, write_data);
    ret = (max77658_pm_get_OP_MODE_2(ctx) == target_val)?SUCCESS:ERROR;
    return ret;
}

int32_t max77658_pm_set_IP_SBB2(max77658_pm_t *ctx, uint8_t target_val)
{
    int32_t ret;
    uint8_t curr_data;
    uint8_t write_data[1];
    ret = max77658_pm_read_reg(ctx, MAX77658_CNFG_SBB2_B, &curr_data);
    write_data[0] = (curr_data & 0xCF) | ((target_val & 0x03) << 4);
    ret = max77658_pm_write_reg(ctx, MAX77658_CNFG_SBB2_B, write_data);
    ret = (max77658_pm_get_IP_SBB2(ctx) == target_val)?SUCCESS:ERROR;
    return ret;
}

int32_t max77658_pm_set_ADE_SBB2(max77658_pm_t *ctx, uint8_t target_val)
{
    int32_t ret;
    uint8_t curr_data;
    uint8_t write_data[1];
    ret = max77658_pm_read_reg(ctx, MAX77658_CNFG_SBB2_B, &curr_data);
    write_data[0] = (curr_data & 0xF7) | ((target_val & 0x01) << 3);
    ret = max77658_pm_write_reg(ctx, MAX77658_CNFG_SBB2_B, write_data);
    ret = (max77658_pm_get_ADE_SBB2(ctx) == target_val)?SUCCESS:ERROR;
    return ret;
}

int32_t max77658_pm_set_EN_SBB2(max77658_pm_t *ctx, uint8_t target_val)
{
    int32_t ret;
    uint8_t curr_data;
    uint8_t write_data[1];
    ret = max77658_pm_read_reg(ctx, MAX77658_CNFG_SBB2_B, &curr_data);
    write_data[0] = (curr_data & 0xF8) | (target_val & 0x07);
    ret = max77658_pm_write_reg(ctx, MAX77658_CNFG_SBB2_B, write_data);
    ret = (max77658_pm_get_EN_SBB2(ctx) == target_val)?SUCCESS:ERROR;
    return ret;
}

int32_t max77658_pm_set_TV_SBB0_DVS(max77658_pm_t *ctx, uint8_t target_val)
{
    int32_t ret;
    uint8_t write_data;

    /* Mask to valid 7-bit TV range */
    write_data = target_val & 0x7F;

    /* 1. Perform Write */
    ret = max77658_pm_write_reg(ctx, MAX77658_CNFG_DVS_SBB0_A, &write_data);
    if (ret < 0) {
        return ret; // Return I2C write error immediately
    }

    /* 2. Verify Write (Read Back) */
    ret = max77658_pm_get_TV_SBB0_DVS(ctx);
    if (ret < 0) {
        return ret; // Return I2C read error
    }

    /* 3. Logic Check */
    if ((uint8_t)ret != write_data) {
        return -1; // ERROR: Data mismatch
    }

    return 0; // SUCCESS
}

int32_t max77658_pm_set_TV_OFS_LDO0(max77658_pm_t *ctx, uint8_t target_val)
{
    int32_t ret;
    uint8_t curr_data;
    uint8_t write_data[1];
    ret = max77658_pm_read_reg(ctx, MAX77658_CNFG_LDO0_A, &curr_data);
    write_data[0] = (curr_data & 0x7F) | ((target_val & 0x01) << 7);
    ret = max77658_pm_write_reg(ctx, MAX77658_CNFG_LDO0_A, write_data);
    ret = (max77658_pm_get_TV_OFS_LDO0(ctx) == target_val)?SUCCESS:ERROR;
    return ret;
}

int32_t max77658_pm_set_TV_LDO0(max77658_pm_t *ctx, uint8_t target_val)
{
    int32_t ret;
    uint8_t curr_data;
    uint8_t write_data[1];
    ret = max77658_pm_read_reg(ctx, MAX77658_CNFG_LDO0_A, &curr_data);
    write_data[0] = (curr_data & 0x80) | (target_val & 0x7F);
    ret = max77658_pm_write_reg(ctx, MAX77658_CNFG_LDO0_A, write_data);
    ret = (max77658_pm_get_TV_LDO0(ctx) == target_val)?SUCCESS:ERROR;
    return ret;
}

int32_t max77658_pm_set_LDO0_MD(max77658_pm_t *ctx, uint8_t target_val)
{
    int32_t ret;
    uint8_t curr_data;
    uint8_t write_data[1];
    ret = max77658_pm_read_reg(ctx, MAX77658_CNFG_LDO0_B, &curr_data);
    write_data[0] = (curr_data & 0xEF) | ((target_val & 0x01) << 4);
    ret = max77658_pm_write_reg(ctx, MAX77658_CNFG_LDO0_B, write_data);
    ret = (max77658_pm_get_LDO0_MD(ctx) == target_val)?SUCCESS:ERROR;
    return ret;
}

int32_t max77658_pm_set_ADE_LDO0(max77658_pm_t *ctx, uint8_t target_val)
{
    int32_t ret;
    uint8_t curr_data;
    uint8_t write_data[1];
    ret = max77658_pm_read_reg(ctx, MAX77658_CNFG_LDO0_B, &curr_data);
    write_data[0] = (curr_data & 0xF7) | ((target_val & 0x01) << 3);
    ret = max77658_pm_write_reg(ctx, MAX77658_CNFG_LDO0_B, write_data);
    ret = (max77658_pm_get_ADE_LDO0(ctx) == target_val)?SUCCESS:ERROR;
    return ret;
}

int32_t max77658_pm_set_EN_LDO0(max77658_pm_t *ctx, uint8_t target_val)
{
    int32_t ret;
    uint8_t curr_data;
    uint8_t write_data[1];
    ret = max77658_pm_read_reg(ctx, MAX77658_CNFG_LDO0_B, &curr_data);
    write_data[0] = (curr_data & 0xF8) | (target_val & 0x07);
    ret = max77658_pm_write_reg(ctx, MAX77658_CNFG_LDO0_B, write_data);
    ret = (max77658_pm_get_EN_LDO0(ctx) == target_val)?SUCCESS:ERROR;
    return ret;
}

int32_t max77658_pm_set_TV_OFS_LDO1(max77658_pm_t *ctx, uint8_t target_val)
{
    int32_t ret;
    uint8_t curr_data;
    uint8_t write_data[1];
    ret = max77658_pm_read_reg(ctx, MAX77658_CNFG_LDO1_A, &curr_data);
    write_data[0] = (curr_data & 0x7F) | ((target_val & 0x01) << 7);
    ret = max77658_pm_write_reg(ctx, MAX77658_CNFG_LDO1_A, write_data);
    ret = (max77658_pm_get_TV_OFS_LDO1(ctx) == target_val)?SUCCESS:ERROR;
    return ret;
}

int32_t max77658_pm_set_TV_LDO1(max77658_pm_t *ctx, uint8_t target_val)
{
    int32_t ret;
    uint8_t curr_data;
    uint8_t write_data[1];
    ret = max77658_pm_read_reg(ctx, MAX77658_CNFG_LDO1_A, &curr_data);
    write_data[0] = (curr_data & 0x80) | (target_val & 0x7F);
    ret = max77658_pm_write_reg(ctx, MAX77658_CNFG_LDO1_A, write_data);
    ret = (max77658_pm_get_TV_LDO1(ctx) == target_val)?SUCCESS:ERROR;
    return ret;
}

int32_t max77658_pm_set_LDO1_MD(max77658_pm_t *ctx, uint8_t target_val)
{
    int32_t ret;
    uint8_t curr_data;
    uint8_t write_data[1];
    ret = max77658_pm_read_reg(ctx, MAX77658_CNFG_LDO1_B, &curr_data);
    write_data[0] = (curr_data & 0xEF) | ((target_val & 0x01) << 4);
    ret = max77658_pm_write_reg(ctx, MAX77658_CNFG_LDO1_B, write_data);
    ret = (max77658_pm_get_LDO1_MD(ctx) == target_val)?SUCCESS:ERROR;
    return ret;
}

int32_t max77658_pm_set_ADE_LDO1(max77658_pm_t *ctx, uint8_t target_val)
{
    int32_t ret;
    uint8_t curr_data;
    uint8_t write_data[1];
    ret = max77658_pm_read_reg(ctx, MAX77658_CNFG_LDO1_B, &curr_data);
    write_data[0] = (curr_data & 0xF7) | ((target_val & 0x01) << 3);
    ret = max77658_pm_write_reg(ctx, MAX77658_CNFG_LDO1_B, write_data);
    ret = (max77658_pm_get_ADE_LDO1(ctx) == target_val)?SUCCESS:ERROR;
    return ret;
}

int32_t max77658_pm_set_EN_LDO1(max77658_pm_t *ctx, uint8_t target_val)
{
    int32_t ret;
    uint8_t curr_data;
    uint8_t write_data[1];
    ret = max77658_pm_read_reg(ctx, MAX77658_CNFG_LDO1_B, &curr_data);
    write_data[0] = (curr_data & 0xF8) | (target_val & 0x07);
    ret = max77658_pm_write_reg(ctx, MAX77658_CNFG_LDO1_B, write_data);
    ret = (max77658_pm_get_EN_LDO1(ctx) == target_val)?SUCCESS:ERROR;
    return ret;
}