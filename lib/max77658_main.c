/*
 * max77658_main.c - Application Specific PMIC Logic
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <math.h>

#include "max77658_main.h"
#include "bsp.h"
#include "pmic_gpio.h"
#include "max77658_pm.h"
#include "max77658_fg.h"
#include "max77658_defines.h"
#include "app_i2c_lock.h"

LOG_MODULE_REGISTER(pmic_app, CONFIG_LOG_DEFAULT_LEVEL);

/* --- PMIC Thread Configuration --- */
#define PMIC_STACK_SIZE 1024
#define PMIC_PRIORITY   7

K_THREAD_STACK_DEFINE(pmic_stack_area, PMIC_STACK_SIZE);
struct k_thread pmic_thread_data;

/* --- Configuration Constants --- */
#define BAT_CAP_MAH         400
#define BAT_TERM_CURR_MA    20
#define BAT_V_EMPTY_MV      3000
#define BAT_V_CHARGE_MV     4200
#define BAT_RSENSE_MOHM     10

/* --- Data Structures --- */
static max77658_pm_t pm_ctx;
static max77658_fg_t fg_ctx;

static struct {
    bool fg_ready;
    volatile bool irq_pending;
} app_state = { false, false };

/* --- Internal Helpers --- */

/* Interrupt Callback (Runs in ISR context) */
static void internal_irq_handler(void *user_data)
{
    ARG_UNUSED(user_data);
    app_state.irq_pending = true;
}

static int sbb_reg_to_mv(uint8_t reg_val)
{
    return 500 + (reg_val * 25);
}

/* Log System Status (Voltage, Current, Errors) */
static void log_status(void)
{
    int32_t ret;
    
    LOG_INF("--- System Status Report ---");

    /* 1. Error Flags */
    ret = max77658_pm_get_ERCFLAG(&pm_ctx);
    if (ret > 0) {
        LOG_WRN("ERC Flags: 0x%02X", ret);
        if (ret & 0x04) LOG_ERR(" >> SYSUVLO DETECTED (Brownout!)");
    }

    /* 2. Charger Status */
    ret = max77658_pm_get_CHG_DTLS(&pm_ctx);
    if (ret >= 0) LOG_INF("Charger State: 0x%X", ret);

    /* 3. Fuel Gauge */
    if (app_state.fg_ready) {
        int soc = max77658_fg_get_SOC(&fg_ctx);
        int vcell = max77658_fg_get_Vcell(&fg_ctx); 
        float current = max77658_fg_get_Current(&fg_ctx);
        
        if (soc >= 0) {
            LOG_INF("Battery: %d%% | %d mV | %.2f mA", soc, vcell, (double)current);
        }
    } else {
        LOG_INF("Battery: FG Not Initialized");
    }

    /* 4. Rail Voltages */
    uint8_t sbb0 = max77658_pm_get_TV_SBB0(&pm_ctx);
    uint8_t sbb1 = max77658_pm_get_TV_SBB1(&pm_ctx);
    uint8_t sbb2 = max77658_pm_get_TV_SBB2(&pm_ctx);
    
    LOG_INF("Rails: SBB0=%dmV, SBB1=%dmV, SBB2=%dmV", 
            sbb_reg_to_mv(sbb0), sbb_reg_to_mv(sbb1), sbb_reg_to_mv(sbb2));
}

static void process_interrupt(void)
{
    int32_t ret;
    uint8_t int0, int1;

    LOG_INF(">> Processing Interrupt...");

    /* Read/Clear INT_GLBL0 */
    ret = max77658_pm_get_INT_GLBL0(&pm_ctx);
    if (ret >= 0) {
        int0 = (uint8_t)ret;
        if (int0 & 0x04) LOG_INF("   nEN Button Pressed");
        if (int0 & 0x08) LOG_INF("   nEN Button Released");
    }

    /* Read/Clear INT_GLBL1 */
    ret = max77658_pm_get_INT_GLBL1(&pm_ctx);
    if (ret >= 0) {
        int1 = (uint8_t)ret;
        if (int1 & 0x04) LOG_ERR("   SBB Regulator Timeout/Fault");
    }
}

/* -------------------------------------------------------------------------- */
/* Power Rail Configuration                                                   */
/* -------------------------------------------------------------------------- */

static int configure_pmic_regulators(void)
{
    int32_t ret;
    int errors = 0;

    LOG_INF("=== Configuring Power Rails (Soft-Start) ===");

    /* --------------------------------------------------------- */
    /* FIX: Ultra-Slow Ramp for SBB0 to prevent SYSUVLO          */
    /* --------------------------------------------------------- */
    
    /* 1. Start SBB0 at lowest safe voltage (3.0V) */
    ret = max77658_pm_set_TV_SBB0(&pm_ctx, 0x64); // 0x64 = 3.0V
    
    /* 2. Enable SBB0 and wait for stabilization */
    max77658_pm_set_EN_SBB0(&pm_ctx, 0x06); // Enable SBB0
    k_msleep(50); // WAIT 50ms (Increased from 20ms)

    /* 3. Step up to 3.6V */
    ret = max77658_pm_set_TV_SBB0(&pm_ctx, 0x7C); // 0x7C = 3.6V
    k_msleep(20); 

    /* 4. Step up to 4.2V */
    ret = max77658_pm_set_TV_SBB0(&pm_ctx, 0x94); // 0x94 = 4.2V
    k_msleep(20);

    /* 5. Finally set to Target 5.0V */
    ret = max77658_pm_set_TV_SBB0(&pm_ctx, 0xB4); // 0xB4 = 5.0V
    
    if (ret < 0) {
        LOG_ERR("Failed to set SBB0 to 5.0V");
        errors++;
    } else {
        LOG_INF("SBB0 Ramped to 5.0V Successfully");
    }

    /* --------------------------------------------------------- */
    /* Configure Other Rails (Standard)                          */
    /* --------------------------------------------------------- */

    /* Configure SBB1 = 1.8V */
    max77658_pm_set_TV_SBB1(&pm_ctx, 0x34);
    max77658_pm_set_EN_SBB1(&pm_ctx, 0x06);

    /* Configure SBB2 = 3.3V */
    max77658_pm_set_TV_SBB2(&pm_ctx, 0x70);
    max77658_pm_set_EN_SBB2(&pm_ctx, 0x06);

    /* Disable LDOs */
    max77658_pm_set_EN_LDO0(&pm_ctx, 0x00);
    max77658_pm_set_EN_LDO1(&pm_ctx, 0x00);

    return (errors == 0) ? 0 : -1;
}

/* -------------------------------------------------------------------------- */
/* Main Initialization Logic                                                  */
/* -------------------------------------------------------------------------- */

int max77658_app_init(void)
{
    int ret;

    LOG_INF("==============================================");
    LOG_INF("   Initializing MAX77658 Subsystem");
    LOG_INF("==============================================");

    /* 1. Low Level BSP (I2C) */
    if (bsp_init() != 0) {
        LOG_ERR("BSP Init Failed");
        return -1;
    }

    /* 2. GPIO & Hardware Reset */
    if (pmic_gpio_init() == 0) {
        LOG_INF("Performing Hardware Reset...");
        pmic_hardware_reset(10, 50);
        
        /* Register IRQ */
        pmic_irq_register_callback(internal_irq_handler, NULL);
        pmic_irq_enable();
    } else {
        LOG_WRN("PMIC GPIO Init Failed (Running without IRQ/Reset)");
    }

    /* 3. Setup Contexts */
    pm_ctx.device_address = MAX77658_PM_ADDR;
    pm_ctx.read_reg = bsp_i2c_reg_read;
    pm_ctx.write_reg = bsp_i2c_reg_write;

    fg_ctx.device_address = 0x36;
    fg_ctx.read_reg = bsp_i2c_reg_read;
    fg_ctx.write_reg = bsp_i2c_reg_write;

    /* 4. Verify Chip ID */
    ret = max77658_pm_get_CID(&pm_ctx);
    if (ret < 0) {
        LOG_ERR("PMIC Not Found (I2C Error)");
        return -1;
    }
    LOG_INF("PMIC Connected (ID: 0x%02X)", ret);

    /* 5. Global Settings */
    max77658_pm_set_nEN_MODE(&pm_ctx, 0x00); // Push-Button Mode

    /* 6. Configure Power Rails */
    ret = configure_pmic_regulators();
    if (ret != 0) {
        LOG_WRN("Regulator configuration had errors (continuing)");
    }

    /* 7. Configure Charger */
    // 200mA Fast Charge, 4.2V, 22.5mA Term
    max77658_pm_set_CHG_CC(&pm_ctx, 0x19);
    max77658_pm_set_CHG_CV(&pm_ctx, 0x18);
    max77658_pm_set_I_TERM(&pm_ctx, 0x02);
    max77658_pm_set_CHG_EN(&pm_ctx, 0x01);

    /* 8. Initialize Fuel Gauge */
    LOG_INF("Initializing Fuel Gauge...");
    fg_ctx.pdata.designcap = BAT_CAP_MAH;
    fg_ctx.pdata.ichgterm = BAT_TERM_CURR_MA;
    fg_ctx.pdata.vempty = BAT_V_EMPTY_MV;
    fg_ctx.pdata.vcharge = BAT_V_CHARGE_MV;
    fg_ctx.pdata.rsense = BAT_RSENSE_MOHM;

    ret = max77658_fg_init(&fg_ctx);
    if (ret != 0) {
        LOG_WRN("Fuel Gauge Init Warning (%d)", ret);
        app_state.fg_ready = false;
    } else {
        app_state.fg_ready = true;
    }

    LOG_INF("Initialization Complete");
    return 0;
}

void max77658_app_process_events(void)
{
    /* Handle pending IRQs */
    if (app_state.irq_pending) {
        app_state.irq_pending = false;
        process_interrupt();
    }
    
    /* Log system health */
    log_status();
}

/* -------------------------------------------------------------------------- */
/* PMIC Thread Implementation                                                 */
/* -------------------------------------------------------------------------- */

void pmic_thread_entry(void *p1, void *p2, void *p3)
{
    ARG_UNUSED(p1);
    ARG_UNUSED(p2);
    ARG_UNUSED(p3);

    LOG_INF("PMIC Thread Started");

    while (1) {
        /* Lock I2C bus before talking to hardware */
        k_mutex_lock(&i2c_lock, K_FOREVER);

        /* Check Interrupts */
        if (app_state.irq_pending) {
            app_state.irq_pending = false;
            process_interrupt();
        }
        
        /* Log Status */
        log_status();
        
        k_mutex_unlock(&i2c_lock);

        /* Sleep: Update PMIC stats every 2 seconds (0.5Hz) */
        k_msleep(2000);
    }
}

void max77658_app_start(void)
{
    k_thread_create(&pmic_thread_data, pmic_stack_area,
                    K_THREAD_STACK_SIZEOF(pmic_stack_area),
                    pmic_thread_entry,
                    NULL, NULL, NULL,
                    PMIC_PRIORITY, 0, K_NO_WAIT);
    k_thread_name_set(&pmic_thread_data, "pmic_thread");
    LOG_INF("PMIC Thread Created (Priority: %d)", PMIC_PRIORITY);
}

void max77658_enter_ship_mode(void)
{
    LOG_INF("Entering Ship Mode (Shutdown)...");
    k_sleep(K_MSEC(100)); // Flush logs
    max77658_pm_set_SFT_CTRL(&pm_ctx, 0x02); // 0x02 = Software Off
    while(1) k_sleep(K_MSEC(100));
}
