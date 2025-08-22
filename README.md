# BQ25180YBGR Driver – Practical Firmware Guide

This repository contains a minimal, portable driver for the TI BQ25180YBGR 1‑A linear charger with I2C control.

## 1) What this IC does (mental model)
The BQ25180 is a single‑cell linear charger with a regulated power path. It autonomously runs a multi‑phase charge:
- Precharge: for deeply discharged cells, a small current raises voltage safely.
- Fast‑charge (CC): constant current `ICHG`.
- Regulation (CV): constant voltage `VREG` as current tapers.
- Termination: charge stops when current reaches `ITERM` (a percent of ICHG). Precharge ratio can be 2× or 1× ITERM.

Core safety/controls you’ll configure:
- `IINLIM` (input current limit) to protect your adapter and manage thermals.
- Safety timer (3h/6h/12h/disable) to bound charge duration.
- Watchdog (optional) if your host can service it.
- TS (thermistor) HOT/COLD limits and WARM/COOL derates (lower ICHG and VREG in warm).

Default 7‑bit I2C address: `0x6A` (check ADR strap).

## 2) Pins (BQ25180YBGR)
Consult the TI datasheet for your exact board, but typical functional pins include:
- I2C: `SCL`, `SDA`
- Enable: `CE` (charge enable) — logic input
- Status/Interrupt: `STAT` and/or `INT` (if populated)
- Thermistor input: `TS` (optional, for NTC)
- Power/battery nodes: `IN`, `SYS`, `BAT`, `PMID` (board‑level power nets; not MCU pins)

I2C requires pull‑up resistors on `SCL` and `SDA` (commonly 2.2–10 kΩ; 4.7 kΩ typical) to the I/O voltage domain.

## 3) STM32 Wiring (example)
- Connect `SCL` → STM32 I2C SCL pin (e.g., `PB6` for `I2C1_SCL`)
- Connect `SDA` → STM32 I2C SDA pin (e.g., `PB7` for `I2C1_SDA`)
- Connect `CE` → any STM32 GPIO output (e.g., `PA8`) if you want MCU control; otherwise strap per design
- Optionally connect `STAT`/`INT` → STM32 GPIO input (e.g., `PA9`) with EXTI if you need interrupts
- Connect `TS` per datasheet (NTC divider). If unused, configure per datasheet recommendation

Note: Verify the BQ25180 7‑bit I2C address based on `ADR` pin strap. Default in this driver is `0x6A` via `BQ25180_I2C_ADDR`.

## 4) STM32 HAL Adapter
Create a thin adapter that plugs STM32 HAL into `bq25180.h` I2C hooks. Example for `I2C1`:
```c
#include "stm32xx_hal.h"
#include "bq25180.h"

extern I2C_HandleTypeDef hi2c1;

static int bq_i2c_write(void *user, uint8_t addr7, uint8_t reg, uint8_t value) {
	uint8_t buf[2] = { reg, value };
	if (HAL_I2C_Master_Transmit(&hi2c1, (uint16_t)(addr7 << 1), buf, 2, 100) == HAL_OK) return 0;
	return -1;
}

static int bq_i2c_read(void *user, uint8_t addr7, uint8_t reg, uint8_t *value) {
	if (HAL_I2C_Master_Transmit(&hi2c1, (uint16_t)(addr7 << 1), &reg, 1, 100) != HAL_OK) return -1;
	if (HAL_I2C_Master_Receive(&hi2c1, (uint16_t)(addr7 << 1), value, 1, 100) != HAL_OK) return -1;
	return 0;
}

static void bq_delay(void *user, uint32_t ms) {
	HAL_Delay(ms);
}

static bq25180_ctx_t bq;

void BQ25180_BringUp(void) {
	bq25180_init(&bq, BQ25180_I2C_ADDR, NULL, bq_i2c_write, bq_i2c_read, bq_delay);

	bq25180_config_t cfg = {
		.vreg_mv = 4200,
		.ichg_ma = 500,
		.ipre_ma = 0,
		.iterm_ma = 0,
		.chg_enable = 1,
	};
	bq25180_apply_config(&bq, &cfg);
}
```

## 5) STM32 HAL I2C Init (CubeMX typical)
Ensure I2C and GPIO clocks are enabled and pins set to AF Open‑Drain with pull‑ups. Example for `I2C1`:
```c
I2C_HandleTypeDef hi2c1;

void MX_I2C1_Init(void) {
	hi2c1.Instance = I2C1;
	hi2c1.Init.Timing = 0x2000090E; // for 100 kHz @ 80 MHz; use CubeMX to compute
	hi2c1.Init.OwnAddress1 = 0x00;
	hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c1.Init.OwnAddress2 = 0xFF;
	hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
	hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	if (HAL_I2C_Init(&hi2c1) != HAL_OK) { Error_Handler(); }

	// Configure analog/digital filters as needed
	if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK) { Error_Handler(); }
	if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK) { Error_Handler(); }
}
```

GPIO example (CubeMX will generate this), for `PB6`/`PB7` AF4 Open‑Drain with Pull‑Up.

## 6) CE and STAT/INT GPIOs
If you route `CE` and `STAT/INT` to MCU pins:
```c
// CE as push-pull output
typedef struct { GPIO_TypeDef *port; uint16_t pin; } bq_gpio_t;
static bq_gpio_t ce = { .port = GPIOA, .pin = GPIO_PIN_8 };

static inline void BQ_CE_Set(int enable) {
	HAL_GPIO_WritePin(ce.port, ce.pin, enable ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

// STAT/INT as input; use EXTI if needed
```

Bring‑up sequence suggestion:
1) Power rails stable; `IN` present per board design
2) Initialize I2C
3) Optionally assert `CE` high via MCU (or strap high)
4) Read `MASK_ID` to confirm I2C comms
5) Apply configuration (VREG/ICHG/etc.)

## 7) Verifying Communication
```c
uint8_t id;
if (bq25180_read_id(&bq, &id) == BQ25180_OK) {
	// ok
} else {
	// check pull-ups, address, wiring, power
}
```

## 8) Configure the charger (feature‑based)
Below are common operations with the provided helpers. Confirm limits/steps in the TI datasheet.

- Set battery regulation voltage (VREG):
```c
bq25180_set_vreg_mv(&bq, 4200); // 4.200 V target
```

- Set fast charge current (ICHG):
```c
bq25180_set_ichg_ma(&bq, 500); // segmented steps
```

- Apply a configuration in one call:
```c
bq25180_config_t cfg = { .vreg_mv = 4200, .ichg_ma = 500, .chg_enable = 1 };
bq25180_apply_config(&bq, &cfg);
```

- Raw register read/write:
```c
uint8_t v;
bq25180_read_register(&bq, BQ25180_REG_VBAT_CTRL, &v);
bq25180_write_register(&bq, BQ25180_REG_ICHG_CTRL, 0x12);
```

- Update masked bits in a register:
```c
// Example: set bit(s) in CHG_CTRL0 using mask
bq25180_update_register_bits(&bq, BQ25180_REG_CHG_CTRL0, 0x20, 0x20);
```

- Software reset / Ship config:
```c
bq25180_software_reset(&bq);
// Configure ship behavior via EN_RST_SHIP
bq25180_configure_ship_reset(&bq, BQ25180_SHIP_CFG_SHIP_EN);
// Optionally set MR long-press code and request MR via board callback
// bq25180_set_mr_long_press_code(&bq, /*code 0..3*/ 1);
// bq25180_enter_ship_via_mr(&bq, board_mr_press_callback, /*hold_ms*/ 2000);
```

- Read device ID (MASK_ID):
```c
uint8_t id2 = 0;
if (bq25180_read_id(&bq, &id2) == BQ25180_OK) {
	// id2 upper bits include mask settings; lower Device_ID identifies BQ25180
}
```

## 9) Reading Status and Interpreting Flags
```c
bq25180_status_t raw;
if (bq25180_read_status(&bq, &raw) == BQ25180_OK) {
	// raw.stat0, raw.stat1, raw.flag0 are the status registers
	bq25180_status_decoded_t dec;
	bq25180_decode_status_bits(&raw, &dec);
	// dec.stat0_bits[i] is bit i of STAT0, etc.
}
```
Typical usage:
- Map `STAT0/STAT1/FLAG0` bits to the datasheet status/fault tables and branch in your application.
- Optionally use `MASK_ID` via `bq25180_set_interrupt_masks(...)` to mask TS/TREG/BAT/PG interrupts as needed.

## 10) Behavior and safety guidance (configure with intent)
- Battery voltage target (`VREG`):
  - Li‑ion typical: 4.20 V; LiFePO4: ~3.60 V. Higher VREG increases capacity but stresses cycle life; pick chemistry‑appropriate values.
  - Host can reduce VREG in warm conditions (TS) to manage thermals.
- Charge current (`ICHG`):
  - Set per cell/thermal budget. The IC uses segmented steps (≤35 mA in 5 mA steps, ≥40 mA in 10 mA steps). When raising `ICHG`, also ensure `IINLIM` and adapter capability are sufficient.
- Termination (`ITERM`) and precharge ratio:
  - ITERM is a fraction of `ICHG`. Lower percent yields longer taper but fuller charge.
  - Precharge can be 2× or 1× ITERM. 1× is gentler; 2× can speed recovery from deep discharge.
- Input current limit (`IINLIM`):
  - Protects the source, reduces brownouts, and constrains thermal load. Choose the nearest step at or below your adapter budget.
- Safety timer:
  - Limits worst‑case charge duration. If your application can sometimes charge slowly (e.g., low `IINLIM`, cold), consider extending to 12h or disabling with care and adding host supervision.
- Watchdog:
  - Enable only if your design can service it reliably; otherwise keep it off to avoid unintended resets.
- TS (thermistor):
  - Use TS to disable outside HOT/COLD limits and derate during WARM/COOL. If TS not used, either disable TS or strap per datasheet so charging is not falsely inhibited.

Safety checklist:
- Confirm chemistry and `VREG`.
- Confirm `ICHG` within cell and thermal limits.
- Set `IINLIM` not to exceed adapter/current path.
- Choose `ITERM` and precharge policy.
- Configure safety timer and watchdog to match system behavior.
- Configure TS thresholds and derates if NTC is present.

## 11) Configuration recipes
- Quiet thermal profile (lower heat):
  - Lower `ICHG`, keep `IINLIM` below adapter budget, use WARM derates (0.2× ICHG and −200 mV VREG).
- Fast recovery from deep discharge:
  - Use precharge 2× ITERM, set `IINLIM` to a safe higher step, ensure safety timer accommodates longer CV taper.
- Adapter‑limited scenario:
  - Pin `IINLIM` to the adapter’s rating; tune `ICHG` accordingly so input + system load ≤ `IINLIM`.

## 12) Quick Start (Firmware)
- Initialize I2C (100–400 kHz typical) and GPIOs (CE as output if routed; STAT/INT as input if used)
- Bind driver to your platform I2C (HAL example provided)
- Read `MASK_ID` to confirm link
- Configure VREG/ICHG and other limits
- Enable charging

Minimal sequence:
```c
bq25180_ctx_t bq; bq25180_stm32_bus_t bus = { .i2c = &hi2c1 };
bq25180_stm32_bind(&bq, &bus, 0);
uint8_t id = 0; bq25180_read_id(&bq, &id);
bq25180_set_vreg_mv(&bq, 4200);
bq25180_set_ichg_ma(&bq, 500);
bq25180_enable_charging(&bq, 1);
```

## 13) End‑to‑End Examples
### A) Typical Li‑ion 4.2 V, 500 mA, with safety timer and watchdog
```c
bq25180_set_vreg_mv(&bq, 4200);
bq25180_set_ichg_ma(&bq, 500);
// Choose termination as a percentage of ICHG and precharge ratio
bq25180_set_termination_pct(&bq, BQ25180_ITERM_10PCT);
bq25180_set_precharge_matches_term(&bq, 1); // 1x ITERM (else 2x)
bq25180_set_input_current_limit_ma(&bq, 500);
bq25180_set_safety_timer(&bq, BQ25180_SFTMR_6H);
bq25180_set_watchdog(&bq, BQ25180_WD_40S);
bq25180_enable_charging(&bq, 1);
```

### B) LiFePO4 profile example (lower VREG)
```c
bq25180_set_vreg_mv(&bq, 3600);
bq25180_set_ichg_ma(&bq, 1000); // if thermals/adapter allow
bq25180_set_input_current_limit_ma(&bq, 1000);
bq25180_enable_charging(&bq, 1);
```

### C) Disable TS (if NTC not used), no watchdog
```c
bq25180_ts_enable(&bq, 0);
bq25180_set_watchdog(&bq, BQ25180_WD_OFF);
```

### D) Raw register tweak (advanced)
```c
// Set a specific bit field in CHG_CTRL0 (verify mask/position in your datasheet)
bq25180_update_register_bits(&bq, BQ25180_REG_CHG_CTRL0, 0x20, 0x20);
```

## 16) Profiles and Watchdog

### Profiles
These presets configure typical combinations quickly. Tune inputs per your hardware.

```c
// Li-ion standard profile
bq25180_apply_profile_liion_standard(&bq,
    4200,   // VREG mV
    500,    // ICHG mA
    500,    // IINLIM mA
    BQ25180_SFTMR_6H);

// Thermal-quiet (gentler under warm conditions)
bq25180_apply_profile_thermal_quiet(&bq,
    4200,   // VREG mV
    300,    // ICHG mA
    300);   // IINLIM mA

// Adapter-limited
bq25180_apply_profile_adapter_limited(&bq,
    500,    // IINLIM mA per adapter
    500);   // cap ICHG to 500 mA
```

### Watchdog service
If you enabled the watchdog, call this periodically within the configured interval.
```c
bq25180_kick_watchdog(&bq);
```

## 17) Default status mapping (example – verify against your datasheet)
The driver provides `bq25180_status_map_default(&map)` to seed a reasonable mapping of bits → meanings. Verify these bit indices on your board/datasheet and adjust if needed.

- STAT0:
  - bit0: charging
  - bit1: charge_done
  - bit2: precharge
- STAT1:
  - bit0: power_good
  - bit1: vindpm_active
  - bit2: ilim_throttled
- FLAG0:
  - bit0: thermal_regulating
  - bit1: battery_uvlo

Usage:
```c
bq25180_status_t raw; bq25180_status_named_t named; bq25180_status_map_t map;
bq25180_read_status(&bq, &raw);
bq25180_status_map_default(&map); // then verify/adjust indices per datasheet
bq25180_decode_status_named(&raw, &map, &named);

if (named.power_good && named.charging) {
    // normal charging
}
if (named.vindpm_active || named.ilim_throttled || named.thermal_regulating) {
    // charger is regulating; consider reducing ICHG or IINLIM
}
```

## 14) Integration Notes
- I2C address depends on ADR strap; default macro is `0x6A` (`BQ25180_I2C_ADDR`)
- Use external pull‑ups sized to bus speed/capacitance
- CE pin can be strapped or MCU‑controlled; STAT/INT optional with EXTI
- Ensure charger thermal/adapter limits align with `ICHG`/`IINLIM`

## 15) Troubleshooting
- NACK on I2C: check address strap, pull‑ups, power rails, and level shifting
- Charge not starting: ensure CE is asserted, `ICHG`/`IINLIM` adequate, and TS mode matches hardware
- Premature termination: verify `ITERM` vs actual current and VREG accuracy

## 18) Building and running unit tests (host‑only)
The tests use a fake I2C device (an in‑memory register array). No hardware is required.

Commands:
- Cross‑platform / single‑config generators (e.g., Ninja, Unix Makefiles):
```bash
cmake -S . -B build
cmake --build build
ctest --test-dir build --output-on-failure
```
- Windows MSBuild (multi‑config): add the configuration (Debug/Release):
```powershell
cmake -S . -B build
cmake --build build
ctest --test-dir build -C Debug --output-on-failure
```

The test prints what it checks. Typical output lines:
- [init] context initialized
- [VREG] set=4200mV read=4200mV
- [ICHG] set=35mA read=35mA; set=500mA read=500mA (segmented mapping)
- [ILIM] set~=700mA code->read=700mA
- [TIMER] safety timer=6h; [WDOG] watchdog=40s
- [TERM] iterm=10% precharge=1x
- [TS] warm derates: ICHG=0.2x, VREG=-200mV
- [TS] hot_code=1 cold_code=2 (reg=0x..)
- [CTRL1] BUVLO_code=5 IBAT_OCP_code=2; INT masks set
- [STATUS] charging=1 done=1 PG=1 thermal=1
- [RESULT] All checks passed.

What’s covered:
- Encoders/decoders: VREG (10 mV), ICHG segmented, ILIM discrete table
- Bitfields: safety timer, watchdog, CHG_CTRL0 (ITERM/IPRECHG), CHG_CTRL1 (BUVLO/OCP, INT masks), TS (HOT/COLD codes, derates)
- Status: raw→bits→named decode

