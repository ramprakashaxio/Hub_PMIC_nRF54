# MAX77658 PMIC - Power Management Summary

## Quick Reference

### Power States
| Mode | SFT_CTRL | Wake Sources | Current | Use Case |
|------|----------|--------------|---------|----------|
| Active | 0x00 | N/A | 50-200mA | Normal operation |
| Software Off | 0x02 | CHGIN + nEN | 10-50µA | User shutdown |
| **Ship Mode** | **0x03** | **CHGIN only** | **<5µA** | **Factory/Storage** |

### API Functions

```c
// Normal shutdown (wake: CHGIN or nEN)
void max77658_enter_software_off(void);

// Factory ship mode (wake: CHGIN only)
void max77658_enter_ship_mode(void);

// Thread-safe shutdown request
void max77658_request_software_off(const char *reason);

// Check if shutdown requested
bool max77658_shutdown_requested(void);
```

### Boot Policy

✅ **Only boots when charger connected**
- Early CHGIN_DTLS check (prevents boot loops)
- If `CHGIN_DTLS != 3` → Enter ship mode immediately
- 10-second CID poll timeout (no infinite wait)

### GPIO Configuration

```c
nEN  (P1.07): GPIO_INPUT (true Hi-Z, no pull-up)
nRST (P1.05): GPIO_INPUT (monitor only, not gated)
nIRQ (P1.06): GPIO_INPUT | GPIO_PULL_UP (interrupt)
```

### Key Registers

**CNFG_GLBL (0x10)** - Bits [1:0] = SFT_CTRL
- `0x02` = Software Off (CHGIN + nEN wake)
- `0x03` = Ship Mode (CHGIN only wake)

**STAT_CHG_B (0x03)** - Bits [3:2] = CHGIN_DTLS
- `0x03` = Charger valid (required for boot)

### Testing

```bash
# Enter ship mode
# Disconnect charger → Device enters ship mode

# Wake from ship mode
# Connect charger → Device boots immediately

# Button wake test (should NOT wake)
# Press nEN button → Device stays off
```

### Common Issues

❌ **Device won't stay off**
- Check: nEN must be `GPIO_INPUT` (no pull-up)

❌ **Boot loops on battery**
- Check: Early CHGIN validation enabled

❌ **nRST prevents boot**
- Fixed: nRST is debug-only, not gated

---

📖 **Full Documentation**: See [SHIP_MODE_GUIDE.md](SHIP_MODE_GUIDE.md)

🔧 **Build Instructions**: See [.github/copilot-instructions.md](.github/copilot-instructions.md)
