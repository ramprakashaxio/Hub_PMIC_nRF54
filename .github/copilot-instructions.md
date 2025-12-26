---
applyTo: '**/*.c,**/*.h,**/CMakeLists.txt,**/Kconfig,**/prj.conf'
---

# PMIC_2512 Project Build Instructions

## Build Environment
- **SDK**: nRF Connect SDK v3.1.0 at `C:\ncs\v3.1.0`
- **Toolchain**: `C:\ncs\toolchains\c1a76fddb2`
- **Board**: nrf52840dk/nrf52840

## Build Command (ALWAYS USE)
```powershell
cd C:\ncs\v3.1.0
& "C:\ncs\toolchains\c1a76fddb2\opt\bin\python.exe" -m west build -b nrf52840dk/nrf52840 "C:\Users\Ramprakash\workspace_5\pmic_2512" --build-dir "C:\Users\Ramprakash\workspace_5\pmic_2512\build" --no-sysbuild
```

## Flash Command
```powershell
cd C:\ncs\v3.1.0
& "C:\ncs\toolchains\c1a76fddb2\opt\bin\python.exe" -m west flash --build-dir "C:\Users\Ramprakash\workspace_5\pmic_2512\build"
```

## Critical Rules
1. MUST run west from `C:\ncs\v3.1.0` directory
2. MUST include `--no-sysbuild` flag
3. DO NOT modify PMIC register sequences in max77658_pm.c or max77658_fg.c without explicit request
4. DO NOT change I2C addresses (PM: 0x48, FG: 0x36)

## PMIC Library Files (Protected)
- `pmic/max77658_pm.c` - Power management (4700+ lines)
- `pmic/max77658_fg.c` - Fuel gauge (1000+ lines)
- `pmic/max77658_defines.h` - Register definitions
