# MAX77658 PMIC Ship Mode & Power Management Guide

## Overview

This project implements robust power management for the MAX77658 PMIC with Nordic nRF54L15, including proper factory ship mode support and charger-only boot policy.

## Power States

### 1. Normal Operation
- All rails active (SBB0=1.8V, SBB1=3.3V, SBB2=5.0V)
- System fully functional
- PMIC thread monitors health every 2 seconds

### 2. Software Off (SFT_CTRL = 0x02)
- **Wake Sources**: CHGIN insertion OR nEN button press (per configuration)
- **Use Case**: Normal user shutdowns, idle timeout
- **Function**: `max77658_enter_software_off()`
- **Current Draw**: ~10-50µA (depending on configuration)

### 3. Factory Ship Mode (SFT_CTRL = 0x03)
- **Wake Source**: CHGIN insertion ONLY (charger plugged in)
- **nEN button**: DISABLED - will NOT wake device
- **Use Case**: Factory shipping, long-term storage, RMA returns
- **Function**: `max77658_enter_ship_mode()`
- **Current Draw**: <5µA (deepest shutdown state)

## Charger-Only Boot Policy

The firmware enforces **charger insertion as the only valid boot source**:

### Boot Sequence
```
1. MCU boots (from USB debugger or residual power)
2. I2C initialized, PMIC context setup
3. GPIO configured (nEN = true Hi-Z, no pull-up)
4. PMIC CID poll (10-second timeout, 50 attempts × 200ms)
5. **EARLY CHGIN CHECK** - Read STAT_CHG_B immediately
6. If CHGIN_DTLS != 3 → Enter ship mode immediately
7. If CHGIN_DTLS == 3 → Continue full initialization
```

### Why This Prevents Boot Loops
Without early CHGIN check, the following can happen:
- PMIC powered from battery/backfeed
- I2C responds (CID read succeeds)
- Init completes, discovers no charger
- Enters ship mode
- Debugger backfeed/WDT/noise wakes PMIC again
- **LOOP**: repeats endlessly

**Solution**: Check CHGIN validity **before** full init. If invalid, enter ship mode immediately with minimal activity.

## GPIO Configuration

### nEN (P1.07) - Power Enable
- **Init State**: `GPIO_INPUT` (true Hi-Z, no internal pull)
- **Active**: Drive LOW to wake PMIC from ship mode
- **Released**: Hi-Z (input, no pull-up/down)
- **Critical**: Internal pull-up can bias nEN HIGH and prevent proper shutdown

### nRST (P1.05) - Reset Monitor
- **Direction**: INPUT (PMIC drives this pin)
- **Purpose**: Debug signal only, NOT used for readiness gating
- **Note**: Init succeeds even if nRST unmapped/wrong

### nIRQ (P1.06) - Interrupt
- **Configuration**: `GPIO_INPUT | GPIO_PULL_UP`
- **Edge**: Falling edge interrupt
- **Purpose**: PMIC event notification (button press, charger events)

## API Reference

### Shutdown Functions

#### `max77658_enter_software_off()`
Normal shutdown with configurable wake sources.
```c
void max77658_enter_software_off(void);
```
- Acquires I2C lock
- Releases nEN to Hi-Z
- Writes SFT_CTRL = 0x02
- Infinite loop (does not return)

#### `max77658_enter_ship_mode()`
Factory ship mode - charger wake only.
```c
void max77658_enter_ship_mode(void);
```
- Disables PMIC IRQ
- Disables all rails (SBB0, SBB1, SBB2)
- Releases nEN to Hi-Z
- Writes SFT_CTRL = 0x03
- Infinite loop (does not return)

### Shutdown Coordination

#### `max77658_request_software_off(const char *reason)`
Thread-safe shutdown request (atomic, single-shot).
```c
max77658_request_software_off("10s timeout");
```
- Used by sensor thread, main thread, or any worker
- PMIC thread detects request and executes ship mode
- Prevents I2C deadlock by coordinating through PMIC thread

#### `max77658_shutdown_requested()`
Check if shutdown was requested.
```c
if (max77658_shutdown_requested()) {
    // Skip remaining work
}
```

## Register Details

### CNFG_GLBL (0x10) - Global Configuration
| Bits | Field     | Value | Description |
|------|-----------|-------|-------------|
| 1:0  | SFT_CTRL  | 0x00  | Normal operation |
| 1:0  | SFT_CTRL  | 0x02  | Software off (wake: CHGIN or nEN) |
| 1:0  | SFT_CTRL  | 0x03  | Factory ship mode (wake: CHGIN only) |

**Critical**: SFT_CTRL writes cannot be verified - PMIC cuts power during write, causing I2C hang. Use `max77658_pm_set_SFT_CTRL_novfy()` (no verification).

### STAT_CHG_B (0x03) - Charger Status
| Bits | Field      | Value | Description |
|------|------------|-------|-------------|
| 3:2  | CHGIN_DTLS | 0x00  | No input |
| 3:2  | CHGIN_DTLS | 0x01  | Input voltage below UVLO |
| 3:2  | CHGIN_DTLS | 0x02  | Input voltage above OVLO |
| 3:2  | CHGIN_DTLS | 0x03  | Input valid (charger present) |

**Boot Policy**: Only `CHGIN_DTLS == 3` allows boot to continue.

## Power Consumption

| State | Typical Current | Notes |
|-------|----------------|-------|
| Active (all rails) | 50-200mA | Depends on sensor/BLE activity |
| Software Off (0x02) | 10-50µA | Varies with nEN config |
| Ship Mode (0x03) | <5µA | Deepest shutdown, charger-only wake |

## Debugging

### Boot Diagnostics
```
[00:00:00.003] <inf> pmic_gpio: nEN: P1.07 configured (Hi-Z, released, no-pull)
[00:00:00.003] <inf> pmic_app: GPIO raw: nEN=1 nRST=0 nIRQ=1
[00:00:00.003] <inf> pmic_app: PMIC CID read = 11 (0x0B)
[00:00:00.004] <inf> pmic_app: Early CHGIN check: STAT_CHG_B=0x0C, CHGIN_DTLS=3
[00:00:00.004] <inf> pmic_app: CHGIN valid - continuing boot
```

### Expected at Power-On
1. **With Charger**: Boot completes, system runs normally
2. **Without Charger**: 
   ```
   [00:00:00.500] <wrn> pmic_app: Boot without valid charger: entering SHIP MODE immediately
   (system halts)
   ```

### Common Issues

#### Device won't stay off
- **Cause**: nEN has internal pull-up enabled
- **Fix**: Ensure `pmic_nen_release_hiz()` uses `GPIO_INPUT` (no pull)

#### Boot loops on battery power
- **Cause**: CHGIN check happens too late
- **Fix**: Early CHGIN check implemented (see boot sequence above)

#### nRST stuck low prevents boot
- **Cause**: Old code gated init on nRST
- **Fix**: nRST now debug-only, init succeeds without it

## Testing Procedure

### 1. Ship Mode Entry
```bash
# Flash firmware with charger connected
nrfjprog --program build/zephyr/zephyr.hex --chiperase --verify --reset

# Observe: Device boots, runs normally
# Disconnect charger
# Expected: Enters ship mode within 200ms
# Verify: Current draw <5µA
```

### 2. Charger Wake
```bash
# With device in ship mode (off)
# Connect charger
# Expected: Device boots immediately
# Log shows: "CHGIN valid - continuing boot"
```

### 3. Button Wake Test
```bash
# Device in ship mode
# Press nEN button
# Expected: NO WAKE (button disabled in ship mode 0x03)
# Verify: Device remains off
```

## Build & Flash

```powershell
cd C:\ncs\v3.1.0
& "C:\ncs\toolchains\c1a76fddb2\opt\bin\python.exe" -m west build -b nrf52840dk/nrf52840 "C:\nrf_workspace\pmic_2512_nrf54" --build-dir "C:\nrf_workspace\pmic_2512_nrf54\build" --no-sysbuild

& "C:\ncs\toolchains\c1a76fddb2\opt\bin\python.exe" -m west flash --build-dir "C:\nrf_workspace\pmic_2512_nrf54\build"
```

## Files Modified

- `lib/max77658_main.c` - Power management logic, boot policy
- `lib/max77658_main.h` - API declarations, documentation
- `driver/pmic/max77658_pm.c` - Added `max77658_pm_set_SFT_CTRL_novfy()` (no-verify write)
- `driver/pmic/max77658_pm.h` - Function prototypes
- `src/pmic_gpio.c` - True Hi-Z nEN configuration
- `src/pmic_gpio.h` - GPIO getter functions
- `src/main.c` - PMIC init failure handling, shutdown deadlock prevention

## References

- [MAX77658 Datasheet](https://www.analog.com/media/en/technical-documentation/data-sheets/MAX77658.pdf)
- Section 5.1.14: SFT_CTRL Field (Ship Mode Configuration)
- Section 5.1.3: STAT_CHG_B Register (CHGIN_DTLS Status)
