/*
 * max77658_main.c - Application Specific PMIC Logic
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/atomic.h>
#include <math.h>

#include "max77658_main.h"
#include "bsp.h"
#include "pmic_gpio.h"
#include "max77658_pm.h"
#include "max77658_fg.h"
#include "max77658_defines.h"
#include "app_i2c_lock.h"

LOG_MODULE_REGISTER(pmic_app, CONFIG_LOG_DEFAULT_LEVEL);

/* --- Shutdown Request Coordination --- */
static atomic_t g_shutdown_req = ATOMIC_INIT(0);
static atomic_t g_shutdown_in_progress = ATOMIC_INIT(0);

bool max77658_shutdown_requested(void)
{
    return atomic_get(&g_shutdown_req) != 0;
}

void max77658_request_software_off(const char *reason)
{
    if (!atomic_cas(&g_shutdown_req, 0, 1)) {
        return; // already requested
    }
    LOG_WRN("PMIC shutdown requested: %s", reason ? reason : "(no reason)");
}

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
            /* Convert uV to mV for cleaner logs */
            LOG_INF("Battery: %d%% | %d mV | %.2f mA", soc, vcell / 1000, (double)current);
        }
    } else {
        LOG_INF("Battery: FG Not Initialized");
    }

    /* 4. Rail Voltages */
    int32_t sbb0_ret = max77658_pm_get_TV_SBB0(&pm_ctx);
    int32_t sbb1_ret = max77658_pm_get_TV_SBB1(&pm_ctx);
    int32_t sbb2_ret = max77658_pm_get_TV_SBB2(&pm_ctx);
    
    if (sbb0_ret >= 0 && sbb1_ret >= 0 && sbb2_ret >= 0) {
        LOG_INF("Rails: SBB0=%dmV, SBB1=%dmV, SBB2=%dmV", 
                sbb_reg_to_mv((uint8_t)sbb0_ret), 
                sbb_reg_to_mv((uint8_t)sbb1_ret), 
                sbb_reg_to_mv((uint8_t)sbb2_ret));
    } else {
        LOG_ERR("Failed to read rail voltages (SBB0=%d, SBB1=%d, SBB2=%d)", 
                sbb0_ret, sbb1_ret, sbb2_ret);
    }
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

#define REG_CNFG_SBB0_B  0x3A

static int configure_pmic_regulators(void)
{
    int32_t ret;
    uint8_t val;
    int errors = 0;

    LOG_INF("=== Configuring Power Rails (SBB0=1.8V, SBB1=3.3V, SBB2=5.0V) ===");

    /* --------------------------------------------------------- */
    /* GLOBAL SETTINGS: Enable 1.5A Peak Current                 */
    /* --------------------------------------------------------- */
    ret = max77658_pm_set_IPK_1P5A(&pm_ctx, 1);
    if (ret < 0) LOG_ERR("Failed to set 1.5A Limit bit");

    /* --------------------------------------------------------- */
    /* 1. Configure SBB0 (1.8V) - KEEPING YOUR WORKING FIX       */
    /* --------------------------------------------------------- */
    /* A. Set Main Target to 1.8V (0x34) */
    ret = max77658_pm_set_TV_SBB0(&pm_ctx, 0x34);
    if (ret < 0) errors++;

    /* B. Set DVS Target to 1.8V (0x34) */
    ret = max77658_pm_set_TV_SBB0_DVS(&pm_ctx, 0x34);
    if (ret < 0) errors++;

    /* C. Force Buck Mode (01) & Force Enable (110) */
    ret = bsp_i2c_reg_read(MAX77658_PM_ADDR, REG_CNFG_SBB0_B, &val, 1);
    if (ret == 0) {
        val &= ~(0xF7);         // Clear bits
        val |= (0x01 << 6);     // OP_MODE = Buck
        val |= (0x00 << 4);     // IP = 1.0A
        val |= 0x06;            // EN = Force On
        bsp_i2c_reg_write(MAX77658_PM_ADDR, REG_CNFG_SBB0_B, &val, 1);
    } else { errors++; }

    /* --------------------------------------------------------- */
    /* 2. Configure SBB1 (3.3V)                                  */
    /* --------------------------------------------------------- */
    max77658_pm_set_IP_SBB1(&pm_ctx, 0); 
    max77658_pm_set_TV_SBB1(&pm_ctx, 0x70); 
    max77658_pm_set_EN_SBB1(&pm_ctx, 0x06); 

    /* --------------------------------------------------------- */
    /* 3. Configure SBB2 (5.0V) - THE FIX                        */
    /* --------------------------------------------------------- */
    /* Set Peak Current to Max (Combined with Global 1.5A) */
    max77658_pm_set_IP_SBB2(&pm_ctx, 0); // 0b00

    /* Ramp Sequence to 5.0V */
    /* 1. Start at 3.3V (0x70) */
    max77658_pm_set_TV_SBB2(&pm_ctx, 0x70); 
    max77658_pm_set_EN_SBB2(&pm_ctx, 0x0E); // Force Enable (preserve defaults)
    k_msleep(20);

    /* 2. Ramp to 4.2V (0x94) */
    max77658_pm_set_TV_SBB2(&pm_ctx, 0x94); 
    k_msleep(20);

    /* 3. Target 5.0V (0xB4) - CHANGED FROM 0x7A */
    /* Formula: (5.0 - 0.5) / 0.025 = 180 = 0xB4 */
    ret = max77658_pm_set_TV_SBB2(&pm_ctx, 0xB4); 
    
    if (ret < 0) {
        LOG_ERR("Failed to set SBB2 to 5.0V");
        errors++;
    } else {
        LOG_INF("SBB2 Configured: 5.0V @ 1.5A Peak Limit");
    }

    /* --------------------------------------------------------- */
    /* 4. Disable LDOs                                           */
    /* --------------------------------------------------------- */
    max77658_pm_set_EN_LDO0(&pm_ctx, 0x04);
    max77658_pm_set_EN_LDO1(&pm_ctx, 0x04);

    return (errors == 0) ? 0 : -1;
}

static void debug_dump_all_rail_registers(void)
{
    uint8_t sbb0_tv, sbb0_dvs, sbb0_cfg, sbb1_tv, sbb1_cfg, sbb2_tv, sbb2_cfg;
    
    /* Read SBB0 registers */
    bsp_i2c_reg_read(MAX77658_PM_ADDR, 0x39, &sbb0_tv, 1);   // TV_SBB0
    bsp_i2c_reg_read(MAX77658_PM_ADDR, 0x3F, &sbb0_dvs, 1);  // DVS_SBB0
    bsp_i2c_reg_read(MAX77658_PM_ADDR, 0x3A, &sbb0_cfg, 1);  // CNFG_SBB0_B
    
    /* Read SBB1 registers */
    bsp_i2c_reg_read(MAX77658_PM_ADDR, 0x3B, &sbb1_tv, 1);   // TV_SBB1
    bsp_i2c_reg_read(MAX77658_PM_ADDR, 0x3C, &sbb1_cfg, 1);  // CNFG_SBB1_B
    
    /* Read SBB2 registers */
    bsp_i2c_reg_read(MAX77658_PM_ADDR, 0x3D, &sbb2_tv, 1);   // TV_SBB2
    bsp_i2c_reg_read(MAX77658_PM_ADDR, 0x3E, &sbb2_cfg, 1);  // CNFG_SBB2_B

    LOG_INF("=== VOLTAGE REGISTER VERIFICATION ===");
    LOG_INF("SBB0 (Target: 1.8V = 0x34):");
    LOG_INF("  TV_SBB0 [0x39]:  0x%02X = %d mV", sbb0_tv, sbb_reg_to_mv(sbb0_tv));
    LOG_INF("  DVS_SBB0 [0x3F]: 0x%02X = %d mV", sbb0_dvs, sbb_reg_to_mv(sbb0_dvs));
    LOG_INF("  CNFG [0x3A]:     0x%02X (Mode bits: 0x%02X)", sbb0_cfg, (sbb0_cfg >> 6) & 0x03);
    
    LOG_INF("SBB1 (Target: 3.3V = 0x70):");
    LOG_INF("  TV_SBB1 [0x3B]:  0x%02X = %d mV", sbb1_tv, sbb_reg_to_mv(sbb1_tv));
    LOG_INF("  CNFG [0x3C]:     0x%02X", sbb1_cfg);
    
    LOG_INF("SBB2 (Target: 5.0V = 0xB4):");
    LOG_INF("  TV_SBB2 [0x3D]:  0x%02X = %d mV", sbb2_tv, sbb_reg_to_mv(sbb2_tv));
    LOG_INF("  CNFG [0x3E]:     0x%02X", sbb2_cfg);
    LOG_INF("=====================================");
}

/* -------------------------------------------------------------------------- */
/* Main Initialization Logic                                                  */
/* -------------------------------------------------------------------------- */

int max77658_app_init(void)
{
    int ret;
    uint8_t val;

    LOG_INF("==============================================");
    LOG_INF("   Initializing MAX77658 Subsystem");
    LOG_INF("==============================================");

    /* 1. Low Level BSP (I2C) */
    if (bsp_init() != 0) {
        LOG_ERR("BSP Init Failed");
        return -1;
    }

    /* Setup PMIC context early for diagnostics */
    pm_ctx.device_address = MAX77658_PM_ADDR;
    pm_ctx.read_reg = bsp_i2c_reg_read;
    pm_ctx.write_reg = bsp_i2c_reg_write;

    /* --------------------------------------------------------------- */
    /* PATCH: Disable LDO1 Active Discharge ASAP (ADE_LDO1 = bit3)      */
    /* CNFG_LDO1_B = 0x4B. This prevents the ~100 ohm discharge burn.   */
    /* --------------------------------------------------------------- */
    ret = bsp_i2c_reg_read(MAX77658_PM_ADDR, 0x4B, &val, 1);
    if (ret == 0) {
        val &= ~(1U << 3); /* ADE_LDO1 = 0 */
        (void)bsp_i2c_reg_write(MAX77658_PM_ADDR, 0x4B, &val, 1);
        LOG_INF("LDO1 Active Discharge Disabled (ADE_LDO1=0)");
    } else {
        LOG_WRN("Failed to disable ADE_LDO1 (ret=%d)", ret);
    }

    /* 2. GPIO & Hardware Reset */
    if (pmic_gpio_init() == 0) {
        /* Diagnostic: Read raw GPIO pin states */
        LOG_INF("GPIO raw: nEN=%d nRST=%d nIRQ=%d",
                pmic_gpio_get_nen(),
                pmic_gpio_get_nrst(),
                pmic_gpio_get_nirq());

        pmic_irq_register_callback(internal_irq_handler, NULL);
        pmic_irq_enable();

        /* CHARGER-ONLY BOOT POLICY: Check CHGIN validity FIRST */
        /* Wait for PMIC to wake, but limit attempts to prevent infinite loop */
        LOG_INF("Waiting for PMIC to wake (charger insertion required)...");
        int wake_attempts = 0;
        while (wake_attempts < 50) {  /* 50 * 200ms = 10 second timeout */
            int cid = max77658_pm_get_CID(&pm_ctx);
            if (cid >= 0) {
                LOG_INF("PMIC woke up! (CID=0x%02X)", cid);
                break;
            }
            LOG_WRN("PMIC asleep. Plug charger (CHGIN)...");
            k_msleep(200);
            wake_attempts++;
        }

        /* Early CHGIN validity check - do this BEFORE full init */
        uint8_t stat_chg_b_early;
        ret = bsp_i2c_reg_read(MAX77658_PM_ADDR, MAX77658_STAT_CHG_B, &stat_chg_b_early, 1);
        if (ret == 0) {
            uint8_t chgin_dtls_early = (stat_chg_b_early >> 2) & 0x03;
            LOG_INF("Early CHGIN check: STAT_CHG_B=0x%02X, CHGIN_DTLS=%u", stat_chg_b_early, chgin_dtls_early);
            
            if (chgin_dtls_early != 3) {
                LOG_WRN("Boot without valid charger: entering SHIP MODE immediately (prevent boot loop)");
                pmic_irq_disable();
                pmic_nen_release_hiz();
                max77658_pm_set_SFT_CTRL_novfy(&pm_ctx, 0x03);
                k_msleep(200);
                while (1) { k_msleep(1000); }
            }
            LOG_INF("CHGIN valid - continuing boot");
        } else {
            LOG_ERR("Failed to read STAT_CHG_B early check (I2C error)");
        }

        /* Verify PMIC is accessible and ready for configuration */
        ret = max77658_pm_get_CID(&pm_ctx);
        if (ret < 0) {
            LOG_ERR("PMIC Not Found (I2C Error)");
            return -1;
        }
        LOG_INF("PMIC Connected (ID: 0x%02X)", ret);

        /* nRST is only a debug signal - log state but don't gate init on it */
        LOG_WRN("nRST raw=%d (ignored for readiness)", pmic_gpio_get_nrst());
    } else {
        LOG_WRN("PMIC GPIO Init Failed (Running without IRQ)");
    }

    /* 3. Setup Fuel Gauge Context */
    fg_ctx.device_address = 0x36;
    fg_ctx.read_reg = bsp_i2c_reg_read;
    fg_ctx.write_reg = bsp_i2c_reg_write;

    /* --- BOOT WAKE SOURCE DETECTION --- */
    uint8_t stat_chg_b, stat_glbl, ercflag;
    bsp_i2c_reg_read(MAX77658_PM_ADDR, MAX77658_STAT_CHG_B, &stat_chg_b, 1);
    bsp_i2c_reg_read(MAX77658_PM_ADDR, MAX77658_STAT_GLBL, &stat_glbl, 1);
    
    ret = max77658_pm_get_ERCFLAG(&pm_ctx);
    ercflag = (ret >= 0) ? (uint8_t)ret : 0;
    
    uint8_t chgin_dtls = (stat_chg_b >> 2) & 0x03;
    
    LOG_INF("=== BOOT WAKE SOURCE DETECTION ===");
    LOG_INF("STAT_CHG_B = 0x%02X (CHGIN_DTLS=%u)", stat_chg_b, chgin_dtls);
    LOG_INF("STAT_GLBL  = 0x%02X", stat_glbl);
    LOG_INF("ERCFLAG    = 0x%02X", ercflag);
    
    if (chgin_dtls == 3) {
        LOG_INF(">>> CHGIN valid (charger present)");
    } else {
        LOG_WRN(">>> CHGIN not valid (dtls=%u) - should not reach here!", chgin_dtls);
    }
    LOG_INF("===================================");

    /* 5. Global Settings */
    max77658_pm_set_nEN_MODE(&pm_ctx, 0x00); // Push-Button Mode
    max77658_pm_set_DBEN_nEN(&pm_ctx, 0x01); // 30ms debounce

    /* 6. Configure Power Rails */
    ret = configure_pmic_regulators();
    if (ret != 0) {
        LOG_WRN("Regulator configuration had errors (continuing)");
    }
    
    /* VERIFY: Dump ALL rail registers to confirm exact voltages */
    debug_dump_all_rail_registers();

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
    k_mutex_lock(&i2c_lock, K_FOREVER);

    /* Handle pending IRQs */
    if (app_state.irq_pending) {
        app_state.irq_pending = false;
        process_interrupt();
    }
    
    /* Log system health */
    log_status();
    
    k_mutex_unlock(&i2c_lock);
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
        /* If shutdown requested, do it once, cleanly */
        if (atomic_get(&g_shutdown_req) && atomic_cas(&g_shutdown_in_progress, 0, 1)) {

            LOG_WRN(">>> Entering FACTORY SHIP MODE now (wake: CHGIN only)");

            /* 1) Stop IRQ + stop future activity */
            pmic_irq_disable();

            /* 2) Take I2C lock and do final PMIC ops */
            k_mutex_lock(&i2c_lock, K_FOREVER);

            /* Optional: turn off heavy rails BEFORE ship mode (reduces glitches) */
            (void)max77658_pm_set_EN_SBB2(&pm_ctx, 0x04); // disable SBB2
            k_msleep(10);

            /* Keep nEN in Hi-Z (no pull-up) */
            pmic_nen_release_hiz();

            /* 3) Enter ship mode (write-only) */
            max77658_pm_set_SFT_CTRL_novfy(&pm_ctx, 0x03);

            /* not expected to return */
            k_msleep(200);
            while (1) { k_msleep(1000); }
        }

        /* Normal periodic work - Lock I2C bus before talking to hardware */
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

void max77658_enter_software_off(void)
{
    LOG_WRN("Entering SOFTWARE OFF (SFT_OFF=0x02). Wake via CHGIN or nEN per config.");

    k_mutex_lock(&i2c_lock, K_FOREVER);

    pmic_nen_release_hiz();

    // WRITE-ONLY (NO VERIFY!)
    max77658_pm_set_SFT_CTRL_novfy(&pm_ctx, 0x02);

    k_mutex_unlock(&i2c_lock);

    k_msleep(200);
    while (1) { k_msleep(1000); }
}

void max77658_enter_software_off_nolock(void)
{
    LOG_WRN("Entering SOFTWARE OFF (SFT_OFF=0x02). Wake via CHGIN or nEN per config.");

    /* Assumes i2c_lock is already held by caller */
    pmic_nen_release_hiz();

    // WRITE-ONLY (NO VERIFY!)
    max77658_pm_set_SFT_CTRL_novfy(&pm_ctx, 0x02);

    k_msleep(200);
    while (1) { k_msleep(1000); }
}

void max77658_enter_ship_mode(void)
{
    LOG_WRN("Entering FACTORY SHIP MODE (FSM). Wake via CHGIN (charger insertion).");

    /* Stop PMIC IRQ so we don't service anything during collapse */
    pmic_irq_disable();

    /* Take I2C lock for final transactions */
    k_mutex_lock(&i2c_lock, K_FOREVER);

    /* Optional: explicitly disable rails before ship mode (not required, but ok) */
    (void)max77658_pm_set_EN_SBB2(&pm_ctx, 0x04); /* disable */
    (void)max77658_pm_set_EN_SBB1(&pm_ctx, 0x04); /* disable */
    (void)max77658_pm_set_EN_SBB0(&pm_ctx, 0x04); /* disable */
    k_msleep(10);

    /* IMPORTANT: make sure nEN is NOT being driven low by MCU */
    pmic_nen_release_hiz();

    /* Enter ship mode: CNFG_GLBL.SFT_CTRL = 0x03 */
    (void)max77658_pm_set_SFT_CTRL_novfy(&pm_ctx, 0x03);

    k_mutex_unlock(&i2c_lock);

    /* Not expected to return */
    k_msleep(200);
    while (1) { k_msleep(1000); }
}
