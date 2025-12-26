# PMIC_2512 Project Build Rules
## nRF Connect SDK v3.1.0 + Zephyr Build Configuration

---

## 1. Environment Setup

### Required Software
- **nRF Connect SDK**: v3.1.0 at `C:\ncs\v3.1.0`
- **Toolchain**: `C:\ncs\toolchains\c1a76fddb2`
- **Target Board**: nrf52840dk/nrf52840

### Environment Variables
```powershell
# Not required if building from C:\ncs\v3.1.0 directory
# ZEPHYR_BASE is automatically set by west
```

---

## 2. Build Commands

### Standard Build (Recommended)
```powershell
cd C:\ncs\v3.1.0
& "C:\ncs\toolchains\c1a76fddb2\opt\bin\python.exe" -m west build -b nrf52840dk/nrf52840 "C:\Users\Ramprakash\workspace_5\pmic_2512" --build-dir "C:\Users\Ramprakash\workspace_5\pmic_2512\build" --no-sysbuild
```

### Clean Build (Pristine)
```powershell
cd C:\ncs\v3.1.0
Remove-Item -Recurse -Force "C:\Users\Ramprakash\workspace_5\pmic_2512\build" -ErrorAction SilentlyContinue
& "C:\ncs\toolchains\c1a76fddb2\opt\bin\python.exe" -m west build -b nrf52840dk/nrf52840 "C:\Users\Ramprakash\workspace_5\pmic_2512" --build-dir "C:\Users\Ramprakash\workspace_5\pmic_2512\build" --no-sysbuild
```

### Flash to Device
```powershell
cd C:\ncs\v3.1.0
& "C:\ncs\toolchains\c1a76fddb2\opt\bin\python.exe" -m west flash --build-dir "C:\Users\Ramprakash\workspace_5\pmic_2512\build"
```

---

## 3. Critical Build Flags

| Flag | Purpose |
|------|---------|
| `--no-sysbuild` | **REQUIRED** - Disables sysbuild which causes bootloader build errors |
| `-b nrf52840dk/nrf52840` | Target board specification |
| `--build-dir` | Output directory for build artifacts |

---

## 4. Project Structure

```
pmic_2512/
├── CMakeLists.txt          # Main build file - adds sources to 'app' target
├── Kconfig                 # Root Kconfig (includes pmic/Kconfig)
├── prj.conf                # Project configuration
├── src/
│   └── main.c              # Application entry point
└── pmic/
    ├── bsp.h               # HAL abstraction (function pointers)
    ├── bsp.c               # HAL instance definition
    ├── max77658.c          # Main PMIC driver
    ├── max77658_pm.c       # Power Management functions
    ├── max77658_fg.c       # Fuel Gauge functions
    ├── max77658_defines.h  # Register definitions
    └── Kconfig             # PMIC configuration options
```

---

## 5. Configuration (prj.conf)

### Required Settings
```ini
# I2C for PMIC communication
CONFIG_I2C=y

# Logging
CONFIG_LOG=y
CONFIG_LOG_DEFAULT_LEVEL=3

# PMIC Driver
CONFIG_PMIC_MAX77658=y
CONFIG_PMIC_MAX77658_PM=y
CONFIG_PMIC_MAX77658_FG=y

# nRF52840 Clock (internal RC)
CONFIG_CLOCK_CONTROL_NRF_K32SRC_RC=y
```

---

## 6. PMIC Library Notes

### I2C Addresses
- **MAX77658 PM**: 0x48 (Power Management)
- **MAX17055 FG**: 0x36 (Fuel Gauge)

### HAL Abstraction
The PMIC library uses a HAL abstraction layer (`bsp.h`) with function pointers:
- `bsp_delay_ms()` → Maps to `k_msleep()`
- `bsp_delay_us()` → Maps to `k_usleep()`

Initialize in `main()`:
```c
g_bsp_hal.delay_ms = bsp_delay_ms_impl;
g_bsp_hal.delay_us = bsp_delay_us_impl;
```

---

## 7. Build Output

### Expected Output Location
- ELF: `build/zephyr/zephyr.elf`
- HEX: `build/zephyr/zephyr.hex`
- BIN: `build/zephyr/zephyr.bin`

### Expected Size (approximate)
- FLASH: ~56 KB (5-6%)
- RAM: ~31 KB (11-12%)

---

## 8. Troubleshooting

### Error: "west: unknown command 'build'"
**Solution**: Run from `C:\ncs\v3.1.0` directory, not from project directory.

### Error: "Kconfig warnings - undefined symbol PMIC_MAX77658"
**Solution**: Ensure `Kconfig` file exists in project root with:
```kconfig
mainmenu "PMIC_2512 Application"
rsource "pmic/Kconfig"
source "Kconfig.zephyr"
```

### Error: "pm_config.h not found" (sysbuild error)
**Solution**: Add `--no-sysbuild` flag to west build command.

### Error: "undefined reference to max77658_pm_*"
**Solution**: Ensure CMakeLists.txt adds PMIC sources directly to app:
```cmake
target_sources(app PRIVATE 
    src/main.c
    pmic/max77658.c
    pmic/bsp.c
    pmic/max77658_pm.c
    pmic/max77658_fg.c
)
```

---

## 9. Version History

| Date | Change | Author |
|------|--------|--------|
| 2025-12-25 | Initial Zephyr port from ESP-IDF | Copilot |
| 2025-12-25 | Added HAL abstraction layer | Copilot |
| 2025-12-25 | Fixed linking - sources added to app | Copilot |

---

## 10. DO NOT MODIFY

The following files contain critical PMIC register sequences - do not modify without explicit user request:
- `pmic/max77658_pm.c` - Power management register operations
- `pmic/max77658_fg.c` - Fuel gauge register operations
- `pmic/max77658_defines.h` - Register address definitions
