/*
 * main.c - MAX77658 PMIC Demo Application for Zephyr
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <math.h>

#include "bsp.h"
#include "pmic_gpio.h"
#include "max77658_pm.h"
#include "max77658_fg.h"
#include "max77658_defines.h"

LOG_MODULE_REGISTER(main, CONFIG_LOG_DEFAULT_LEVEL);

/* PMIC context structures */
static max77658_pm_t pm_ctx;
static max77658_fg_t fg_ctx;

/* Fuel gauge initialization status */
static bool fg_initialized = false;

/* PMIC interrupt flag */
static volatile bool pmic_irq_pending = false;

/*
 * PMIC Interrupt Callback
 * Called from ISR context when nIRQ pin goes LOW
 */
static void pmic_irq_handler(void *user_data)
{
    ARG_UNUSED(user_data);
    pmic_irq_pending = true;
}

/*
 * Initialize PMIC contexts with BSP I2C functions
 */
static void init_pmic_contexts(void)
{
    /* Power Management IC context (I2C address 0x48) */
    pm_ctx.device_address = MAX77658_PM_ADDR;
    pm_ctx.read_reg = bsp_i2c_reg_read;
    pm_ctx.write_reg = bsp_i2c_reg_write;

    /* Fuel Gauge context (I2C address 0x36) */
    fg_ctx.device_address = 0x36;
    fg_ctx.read_reg = bsp_i2c_reg_read;
    fg_ctx.write_reg = bsp_i2c_reg_write;

    LOG_INF("PMIC contexts initialized");
    LOG_INF("  PM address:  0x%02X", pm_ctx.device_address);
    LOG_INF("  FG address:  0x%02X", fg_ctx.device_address);
}

/*
 * Initialize Fuel Gauge with battery parameters
 * CRITICAL: You must set the battery parameters here before calling init
 */
static int init_fuel_gauge(void)
{
    int ret;

    LOG_INF("=== Initializing Fuel Gauge ===");

    /* * 1. Configure Battery Parameters 
     * Adjust these values to match your specific battery!
     * Example below is for a 400mAh 4.2V LiPo battery.
     */
    fg_ctx.pdata.designcap = 400;      // Capacity in mAh (e.g., 400mAh)
    fg_ctx.pdata.ichgterm = 20;        // Termination current in mA
    fg_ctx.pdata.vempty = 3000;        // Empty voltage in mV (3.0V)
    fg_ctx.pdata.vcharge = 4200;       // Charge voltage in mV (4.2V)
    fg_ctx.pdata.rsense = 10;          // Sense resistor in mOhms (usually 10 on EV kits)

    /* 2. Initialize the Fuel Gauge Driver */
    ret = max77658_fg_init(&fg_ctx);
    if (ret != 0) {
        LOG_ERR("Fuel gauge init failed: %d", ret);
        return ret;
    }

    LOG_INF("Fuel gauge initialized successfully");
    fg_initialized = true;
    return 0;
}

/*
 * Read and display PMIC Power Management status
 */
static void read_pm_status(void)
{
    int32_t ret;

    LOG_INF("=== MAX77658 Power Management Status ===");

    ret = max77658_pm_get_CID(&pm_ctx);
    if (ret >= 0) {
        LOG_INF("Chip ID (CID): 0x%02X", ret);
    } else {
        LOG_ERR("Failed to read Chip ID");
    }

    ret = max77658_pm_get_DIDM(&pm_ctx);
    if (ret >= 0) {
        LOG_INF("Device ID (DIDM): %d %s", ret, (ret == 0) ? "(MAX77658)" : "(Reserved)");
    }

    ret = max77658_pm_get_BOK(&pm_ctx);
    if (ret >= 0) {
        LOG_INF("Main Bias OK (BOK): %d %s", ret, ret ? "Ready" : "Not Ready");
    }

    ret = max77658_pm_get_INT_GLBL0(&pm_ctx);
    if (ret >= 0) {
        LOG_INF("INT_GLBL0: 0x%02X", ret);
    }

    ret = max77658_pm_get_INT_GLBL1(&pm_ctx);
    if (ret >= 0) {
        LOG_INF("INT_GLBL1: 0x%02X", ret);
    }

    ret = max77658_pm_get_ERCFLAG(&pm_ctx);
    if (ret >= 0) {
        LOG_INF("Error Flags (ERCFLAG): 0x%02X", ret);
        if (ret & 0x01) LOG_INF("  - TOVLD: Thermal overload");
        if (ret & 0x02) LOG_INF("  - SYSOVLO: Sys overvoltage lockout");
        if (ret & 0x04) LOG_INF("  - SYSUVLO: Sys undervoltage lockout");
        if (ret & 0x08) LOG_INF("  - MRST: Manual reset timer");
        if (ret & 0x10) LOG_INF("  - SFT_OFF_F: Software off flag");
        if (ret & 0x20) LOG_INF("  - SFT_CRST_F: Software cold reset");
        if (ret & 0x40) LOG_INF("  - WDT_OFF: Watchdog timer off");
        if (ret & 0x80) LOG_INF("  - WDT_RST: Watchdog timer reset");
    }

    ret = max77658_pm_get_STAT_IRQ(&pm_ctx);
    if (ret >= 0) {
        LOG_INF("IRQ Status: %d", ret);
    }
}

/*
 * Read and display Charger status
 */
static void read_charger_status(void)
{
    int32_t ret;

    LOG_INF("=== MAX77658 Charger Status ===");

    ret = max77658_pm_get_CHG(&pm_ctx);
    if (ret >= 0) {
        LOG_INF("Charger Status (CHG): %d", ret);
    }

    ret = max77658_pm_get_CHG_DTLS(&pm_ctx);
    if (ret >= 0) {
        const char *chg_state;
        switch (ret) {
            case 0x0: chg_state = "Off"; break;
            case 0x1: chg_state = "Prequalification"; break;
            case 0x2: chg_state = "Fast-charge CC"; break;
            case 0x3: chg_state = "JEITA CC"; break;
            case 0x4: chg_state = "Fast-charge CV"; break;
            case 0x5: chg_state = "JEITA CV"; break;
            case 0x6: chg_state = "Top-off"; break;
            case 0x7: chg_state = "JEITA Top-off"; break;
            case 0x8: chg_state = "Done"; break;
            case 0x9: chg_state = "JEITA Done"; break;
            case 0xA: chg_state = "Prequal Timer Fault"; break;
            case 0xB: chg_state = "Fast-charge Timer Fault"; break;
            case 0xC: chg_state = "Battery Temp Fault"; break;
            default:  chg_state = "Unknown"; break;
        }
        LOG_INF("Charger Details: 0x%X (%s)", ret, chg_state);
    }

    ret = max77658_pm_get_CHGIN_DTLS(&pm_ctx);
    if (ret >= 0) {
        const char *chgin_state;
        switch (ret) {
            case 0x0: chgin_state = "Below UVLO"; break;
            case 0x1: chgin_state = "Above UVLO, Below OVP"; break;
            case 0x2: chgin_state = "Above OVP"; break;
            case 0x3: chgin_state = "Being debounced"; break;
            default:  chgin_state = "Unknown"; break;
        }
        LOG_INF("CHGIN Details: 0x%X (%s)", ret, chgin_state);
    }
}

/*
 * Read and display Fuel Gauge status
 * Only reads safe values that don't require rsense initialization
 */
static void read_fuel_gauge_status(void)
{
    int ret;
    uint16_t reg_val;

    LOG_INF("=== MAX17055 Fuel Gauge Status ===");

    /* Check if fuel gauge responds */
    ret = max77658_fg_read_reg(&fg_ctx, VERSION_REG, &reg_val);
    if (ret != 0) {
        LOG_WRN("Fuel Gauge not responding");
        return;
    }
    LOG_INF("FG Version: 0x%04X", reg_val);

    /* Read SOC - always safe */
    ret = max77658_fg_get_SOC(&fg_ctx);
    if (ret >= 0) {
        LOG_INF("State of Charge (SOC): %d%%", ret);
    }

    /* Read Vcell directly from register to avoid conversion issues */
    ret = max77658_fg_read_reg(&fg_ctx, VCELL_REG, &reg_val);
    if (ret == 0) {
        /* Vcell LSB = 78.125uV, convert to mV */
        uint32_t vcell_mv = (uint32_t)reg_val * 78125 / 1000000;
        LOG_INF("Cell Voltage: %d mV", vcell_mv);
    }

    /* Read temperature - always safe */
    ret = max77658_fg_get_temperature(&fg_ctx);
    if (ret > -100) {
        LOG_INF("Temperature: %d C", ret);
    }

    /* Only read current-related values if FG is properly initialized */
    if (fg_initialized) {
        float current = max77658_fg_get_Current(&fg_ctx);
        /* Check for valid value (not inf or nan) */
        if (isfinite(current)) {
            LOG_INF("Current: %.2f mA", (double)current);
        } else {
            LOG_INF("Current: N/A");
        }

        float avg_current = max77658_fg_get_AvgCurrent(&fg_ctx);
        if (isfinite(avg_current)) {
            LOG_INF("Avg Current: %.2f mA", (double)avg_current);
        }

        float tte = max77658_fg_get_TTE(&fg_ctx);
        if (isfinite(tte) && tte > 0) {
            LOG_INF("Time to Empty: %.1f min", (double)tte);
        }

        float ttf = max77658_fg_get_TTF(&fg_ctx);
        if (isfinite(ttf) && ttf > 0) {
            LOG_INF("Time to Full: %.1f min", (double)ttf);
        }
    } else {
        LOG_INF("Current/TTE/TTF: N/A (FG not initialized)");
    }

    /* Read battery capacity */
    ret = max77658_fg_get_battCAP(&fg_ctx);
    if (ret >= 0) {
        LOG_INF("Battery Capacity: %d mAh", ret);
    }
}

/*
 * Read and display Regulator status
 */
static void read_regulator_status(void)
{
    int32_t ret;
    uint8_t reg_val;
    int voltage_mv;

    LOG_INF("=== MAX77658 Regulator Status ===");

    /* SBB0 - SIMO Buck-Boost 0 */
    ret = max77658_pm_get_EN_SBB0(&pm_ctx);
    if (ret >= 0) {
        LOG_INF("SBB0 Enable: %d", ret);
        reg_val = max77658_pm_get_TV_SBB0(&pm_ctx);
        voltage_mv = max77658_pm_sbb_reg_to_mv(reg_val);
        LOG_INF("  SBB0 Voltage: %d mV (reg: 0x%02X)", voltage_mv, reg_val);
    }

    /* SBB1 - SIMO Buck-Boost 1 */
    ret = max77658_pm_get_EN_SBB1(&pm_ctx);
    if (ret >= 0) {
        LOG_INF("SBB1 Enable: %d", ret);
        reg_val = max77658_pm_get_TV_SBB1(&pm_ctx);
        voltage_mv = max77658_pm_sbb_reg_to_mv(reg_val);
        LOG_INF("  SBB1 Voltage: %d mV (reg: 0x%02X)", voltage_mv, reg_val);
    }

    /* SBB2 - SIMO Buck-Boost 2 */
    ret = max77658_pm_get_EN_SBB2(&pm_ctx);
    if (ret >= 0) {
        LOG_INF("SBB2 Enable: %d", ret);
        reg_val = max77658_pm_get_TV_SBB2(&pm_ctx);
        voltage_mv = max77658_pm_sbb_reg_to_mv(reg_val);
        LOG_INF("  SBB2 Voltage: %d mV (reg: 0x%02X)", voltage_mv, reg_val);
    }

    /* LDO0 - Low Dropout Regulator 0 */
    ret = max77658_pm_get_EN_LDO0(&pm_ctx);
    if (ret >= 0) {
        LOG_INF("LDO0 Enable: %d", ret);
        reg_val = max77658_pm_get_TV_LDO0(&pm_ctx);
        /* LDO uses different formula: Base voltage + (reg_val * step)
         * For MAX77658: 0.4875V to 3.9375V, step = 12.5mV for TV_LDO < 0x40
         * Simplified: voltage_mv = 487.5 + (reg_val * 12.5) for TV_OFS=0 */
        voltage_mv = 487 + (reg_val * 12); // Approximate mV
        LOG_INF("  LDO0 Voltage: ~%d mV (reg: 0x%02X)", voltage_mv, reg_val);
    }

    /* LDO1 - Low Dropout Regulator 1 */
    ret = max77658_pm_get_EN_LDO1(&pm_ctx);
    if (ret >= 0) {
        LOG_INF("LDO1 Enable: %d", ret);
        reg_val = max77658_pm_get_TV_LDO1(&pm_ctx);
        /* Same formula as LDO0 */
        voltage_mv = 487 + (reg_val * 12); // Approximate mV
        LOG_INF("  LDO1 Voltage: ~%d mV (reg: 0x%02X)", voltage_mv, reg_val);
    }
}

/*
 * Configure Battery Charging
 * Battery: 400mAh Li-Po
 * Charge Current: 200mA (0.5C rate)
 * Charge Voltage: 4.2V
 * Termination Current: 20mA (5% of capacity)
 * Prequalification Current: 10% of fast charge
 */
static int configure_pmic_charging(void)
{
    int32_t ret;
    int errors = 0;

    LOG_INF("=== Configuring Battery Charging ===");

    /* Set charge current: CHG_CC = (200mA - 7.5) / 7.5 = 25.67 -> 26 (0x1A) = 202.5mA
     * Or use 25 (0x19) = 195mA - closer to 200mA */
    ret = max77658_pm_set_CHG_CC(&pm_ctx, 0x19);
    if (ret < 0) {
        LOG_ERR("Failed to set CHG_CC: %d", ret);
        errors++;
    } else {
        LOG_INF("CHG_CC set to 0x19 (~195mA)");
    }

    /* Set charge voltage: CHG_CV = (4.2V - 3.6V) / 0.025V = 24 (0x18) = 4.2V */
    ret = max77658_pm_set_CHG_CV(&pm_ctx, 0x18);
    if (ret < 0) {
        LOG_ERR("Failed to set CHG_CV: %d", ret);
        errors++;
    } else {
        LOG_INF("CHG_CV set to 0x18 (4.2V)");
    }

    /* Set termination current: I_TERM
     * 0 = 7.5mA, 1 = 15mA, 2 = 22.5mA, 3 = 30mA
     * For 20mA target, use 2 (22.5mA) */
    ret = max77658_pm_set_I_TERM(&pm_ctx, 0x02);
    if (ret < 0) {
        LOG_ERR("Failed to set I_TERM: %d", ret);
        errors++;
    } else {
        LOG_INF("I_TERM set to 0x02 (22.5mA)");
    }

    /* Set prequalification current: I_PQ
     * 0 = 10%, 1 = 20% of fast charge current
     * Use 10% = ~20mA */
    ret = max77658_pm_set_I_PQ(&pm_ctx, 0x00);
    if (ret < 0) {
        LOG_ERR("Failed to set I_PQ: %d", ret);
        errors++;
    } else {
        LOG_INF("I_PQ set to 0x00 (10%% of CHG_CC)");
    }

    /* Enable charging */
    ret = max77658_pm_set_CHG_EN(&pm_ctx, 0x01);
    if (ret < 0) {
        LOG_ERR("Failed to enable charging: %d", ret);
        errors++;
    } else {
        LOG_INF("Charging ENABLED");
    }

    if (errors == 0) {
        LOG_INF("Battery charging configured successfully");
    } else {
        LOG_ERR("Battery charging config had %d errors", errors);
    }

    return (errors == 0) ? 0 : -1;
}

/*
 * Configure Power Rails (SBBs and LDOs)
 * SBB0: 5.0V (enabled) - for external devices
 * SBB1: 1.8V (enabled) - for low-voltage peripherals
 * SBB2: 3.3V (enabled) - for general 3.3V devices
 * LDO0: 1.8V (disabled)
 * LDO1: 3.0V (disabled)
 */
static int configure_pmic_regulators(void)
{
    int32_t ret;
    int errors = 0;

    LOG_INF("=== Configuring Power Rails ===");

    /* Configure SBB0 = 5.0V */
    // Calculation: (5000mV - 500mV) / 25mV = 180 = 0xB4
    ret = max77658_pm_set_TV_SBB0(&pm_ctx, 0xB4); 
    if (ret < 0) errors++;

    /* Configure SBB1 = 1.8V */
    // Calculation: (1800mV - 500mV) / 25mV = 52 = 0x34
    ret = max77658_pm_set_TV_SBB1(&pm_ctx, 0x34);
    if (ret < 0) errors++;

    /* Configure SBB2 = 3.3V */
    // Calculation: (3300mV - 500mV) / 25mV = 112 = 0x70
    ret = max77658_pm_set_TV_SBB2(&pm_ctx, 0x70);
    if (ret < 0) errors++;

    /* Enable SBBs */
    max77658_pm_set_EN_SBB0(&pm_ctx, 0x06); // Enable SBB0
    max77658_pm_set_EN_SBB1(&pm_ctx, 0x06); // Enable SBB1
    max77658_pm_set_EN_SBB2(&pm_ctx, 0x06); // Enable SBB2

    /* Disable LDOs */
    max77658_pm_set_EN_LDO0(&pm_ctx, 0x00); // Disable LDO0
    max77658_pm_set_EN_LDO1(&pm_ctx, 0x00); // Disable LDO1

    return (errors == 0) ? 0 : -1;
}

/*
 * Enter Ship Mode (Software Off / Shutdown)
 * 
 * Puts the PMIC into ultra-low power mode (~1µA). All regulators turn off.
 * MCU will power down and cannot wake itself via software.
 * 
 * Wake-up requires hardware action:
 *   - Press nEN button (>30ms hold)
 *   - Connect charger (CHGIN > 4.0V)
 * 
 * Call this function when you want to shut down the system.
 */
static void enter_ship_mode(void)
{
    LOG_INF("Entering Ship Mode (Power Off)...");

    /*
     * Command: Software Off
     * Register: CNFG_GLBL (0x10)
     * Field: SFT_CTRL (Bits 1:0)
     * Value: 0x02 (binary 0b10)
     */
    int32_t ret = max77658_pm_set_SFT_CTRL(&pm_ctx, 0x02);

    if (ret != 0) {
        LOG_ERR("Failed to enter Ship Mode");
        return;
    }

    LOG_INF("Ship Mode command sent. Powering down in ~60ms...");

    /*
     * The PMIC will wait ~60ms before cutting power.
     * We enter an infinite loop here to prevent the MCU from
     * doing anything else while waiting to die.
     */
    while (1) {
        k_sleep(K_MSEC(100));
    }
}

/*
 * Handle PMIC interrupt
 */
static void handle_pmic_interrupt(void)
{
    int32_t ret;

    LOG_INF("=== PMIC Interrupt Detected ===");

    ret = max77658_pm_get_INT_GLBL0(&pm_ctx);
    if (ret >= 0) {
        LOG_INF("INT_GLBL0: 0x%02X", ret);
        if (ret & 0x04) LOG_INF("  - nEN falling edge");
        if (ret & 0x08) LOG_INF("  - nEN rising edge");
        if (ret & 0x10) LOG_INF("  - Thermal alarm 1");
        if (ret & 0x20) LOG_INF("  - Thermal alarm 2");
    }

    ret = max77658_pm_get_INT_GLBL1(&pm_ctx);
    if (ret >= 0) {
        LOG_INF("INT_GLBL1: 0x%02X", ret);
        if (ret & 0x04) LOG_INF("  - SBB timeout");
        if (ret & 0x08) LOG_INF("  - LDO fault");
    }
}

/*
 * Main application entry point
 */
int main(void)
{
    int ret;

    LOG_INF("==============================================");
    LOG_INF("MAX77658 PMIC Demo Application");
    LOG_INF("==============================================");

    /* Initialize BSP (I2C, delays) */
    ret = bsp_init();
    if (ret != 0) {
        LOG_ERR("BSP init failed: %d", ret);
        return ret;
    }

    /* Initialize PMIC GPIO pins (nRST, nIRQ) */
    ret = pmic_gpio_init();
    if (ret != 0) {
        LOG_WRN("PMIC GPIO init failed (continuing): %d", ret);
    } else {
        /* Perform hardware reset */
        LOG_INF("Performing PMIC hardware reset...");
        pmic_hardware_reset(10, 50);

        /* Register interrupt callback */
        ret = pmic_irq_register_callback(pmic_irq_handler, NULL);
        if (ret == 0) {
            pmic_irq_enable();
            LOG_INF("PMIC interrupt handler registered");
        }
    }

    /* Scan I2C bus */
    bsp_i2c_scan();

    /* Initialize PMIC contexts */
    init_pmic_contexts();

    /* Configure nEN pin as Push-Button mode (default, but set explicitly) */
    ret = max77658_pm_set_nEN_MODE(&pm_ctx, 0x00);
    if (ret < 0) {
        LOG_WRN("Failed to set nEN mode: %d", ret);
    } else {
        LOG_INF("nEN configured as Push-Button mode");
    }

    /* Initialize Fuel Gauge (sets up rsense for current calculations) */
    ret = init_fuel_gauge();
    if (ret != 0) {
        LOG_WRN("Fuel gauge init failed - current readings will be unavailable");
    }

    /* Configure PMIC charging (400mAh battery, 200mA charge current) */
    ret = configure_pmic_charging();
    if (ret != 0) {
        LOG_WRN("Charging configuration failed (continuing)");
    }

    /* Configure power rails (SBBs and LDOs) */
    ret = configure_pmic_regulators();
    if (ret != 0) {
        LOG_WRN("Regulator configuration failed (continuing)");
    }

    /* Check for pending interrupt at startup */
    if (pmic_irq_is_active()) {
        LOG_INF("PMIC interrupt pending at startup");
        handle_pmic_interrupt();
    }

    /* Main loop */
    while (1) {
        /* Handle PMIC interrupt */
        if (pmic_irq_pending) {
            pmic_irq_pending = false;
            handle_pmic_interrupt();
        }

        LOG_INF("----------------------------------------------");

        read_pm_status();
        read_charger_status();
        read_regulator_status();
        read_fuel_gauge_status();

        LOG_INF("----------------------------------------------");

        k_sleep(K_SECONDS(5));
    }

    return 0;
}