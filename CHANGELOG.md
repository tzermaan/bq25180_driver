# Changelog

## v1.0.0 (2025-01-XX)

Initial public release of the BQ25180 driver

- Portable C99 driver (no dynamic allocation), context-injected I2C transport
- Complete register coverage for VBAT_CTRL, ICHG_CTRL, CHG_CTRL0/1, IC_CTRL, TMR_ILIM, SHIP_RST, TS_CONTROL, SYS_REG, MASK_ID
- Encoders/decoders
  - VREG (10 mV steps, 3.5–4.65 V)
  - ICHG segmented (≤35 mA: 5 mA; ≥40 mA: 10 mA)
  - ILIM discrete table {50, 100, 200, 300, 400, 500, 700, 1100} mA
  - Safety timer and watchdog codes
- Configuration helpers
  - Termination percent (5/10/20%), precharge ratio (1×/2×)
  - TS HOT/COLD codes, WARM/COOL disables, ICHG and VREG derates
  - CHG_CTRL0: VINDPM, THERM_REG; CHG_CTRL1: BUVLO/OCP, INT masks
  - SHIP_RST configuration (REG_RST, EN_RST_SHIP); MR long-press helper
  - Profiles: Li-ion standard, thermal-quiet, adapter-limited
- Observability & status
  - STAT0/STAT1/FLAG0 raw read, bit-decoder, named decoder, state classifier
- STM32 HAL adapter and wiring examples
- Unit tests (host-only, fake I2C) with printed expectations/results
- GitHub Actions CI (Windows/Linux/macOS)
- MIT License (c) 2025 Tzer Maan

Known notes
- Ship entry requires MR/power-path trigger; no I2C “ship now” command
- Verify status bit indices against your datasheet revision and board
