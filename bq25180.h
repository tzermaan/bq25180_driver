// BQ25180YBGR I2C Charger Driver - Public API
// Portable, minimal dependency header

#ifndef BQ25180_H
#define BQ25180_H

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

// Notes:
// - This driver uses a user-supplied I2C transport via function pointers
// - No dynamic memory is used; the caller owns the context lifetime

// Default 7-bit I2C address when ADR pin strapped to GND. Adjust if different.
#ifndef BQ25180_I2C_ADDR
#define BQ25180_I2C_ADDR 0x6A
#endif

// Register addresses (from TI datasheet, Table 8-7)
typedef enum {
	BQ25180_REG_STAT0       = 0x00,
	BQ25180_REG_STAT1       = 0x01,
	BQ25180_REG_FLAG0       = 0x02,
	BQ25180_REG_VBAT_CTRL   = 0x03,
	BQ25180_REG_ICHG_CTRL   = 0x04,
	BQ25180_REG_CHG_CTRL0   = 0x05,
	BQ25180_REG_CHG_CTRL1   = 0x06,
	BQ25180_REG_IC_CTRL     = 0x07,
	BQ25180_REG_TMR_ILIM    = 0x08,
	BQ25180_REG_SHIP_RST    = 0x09,
	BQ25180_REG_SYS_REG     = 0x0A,
	BQ25180_REG_TS_CONTROL  = 0x0B,
	BQ25180_REG_MASK_ID     = 0x0C
} bq25180_register_t;

// Field masks (align with datasheet; verify per chosen revision)
#define BQ25180_VBAT_CTRL_VREG_MASK			(0x7Fu)        // VBATREG[6:0]

// ICHG_CTRL (0x04)
#define BQ25180_ICHG_CTRL_CHG_DIS_MASK			(0x80u)        // bit7
#define BQ25180_ICHG_CTRL_ICHG_MASK			(0x7Fu)        // bits[6:0]

// CHG_CTRL0 (0x05)
#define BQ25180_CHG_CTRL0_IPRECHG_MASK			(0x40u)        // bit6
#define BQ25180_CHG_CTRL0_ITERM_MASK			(0x30u)        // bits[5:4]
#define BQ25180_CHG_CTRL0_VINDPM_MASK			(0x0Cu)        // bits[3:2]
#define BQ25180_CHG_CTRL0_THERM_REG_MASK		(0x03u)        // bits[1:0]

// CHG_CTRL1 (0x06)
#define BQ25180_CHG_CTRL1_IBAT_OCP_MASK		(0xC0u)        // bits[7:6]
#define BQ25180_CHG_CTRL1_BUVLO_MASK			(0x38u)        // bits[5:3]
#define BQ25180_CHG_CTRL1_CHG_STATUS_INT_MASK	(0x04u)        // bit2
#define BQ25180_CHG_CTRL1_ILIM_INT_MASK		(0x02u)        // bit1
#define BQ25180_CHG_CTRL1_VDPM_INT_MASK		(0x01u)        // bit0

// IC_CTRL (0x07)
#define BQ25180_IC_CTRL_TS_EN_MASK			(0x80u)        // bit7
#define BQ25180_IC_CTRL_VLOWV_SEL_MASK		(0x40u)        // bit6
#define BQ25180_IC_CTRL_VRCH_MASK			(0x20u)        // bit5
#define BQ25180_IC_CTRL_2XTMR_EN_MASK			(0x10u)        // bit4
#define BQ25180_IC_CTRL_SFTMR_MASK			(0x0Cu)        // bits[3:2]
#define BQ25180_IC_CTRL_WATCHDOG_MASK			(0x03u)        // bits[1:0]

// TMR_ILIM (0x08)
#define BQ25180_TMR_ILIM_MR_LPRESS_MASK		(0xC0u)        // bits[7:6]
#define BQ25180_TMR_ILIM_MR_RESET_VIN_MASK		(0x20u)        // bit5
#define BQ25180_TMR_ILIM_AUTOWAKE_MASK		(0x18u)        // bits[4:3]
#define BQ25180_TMR_ILIM_ILIM_MASK			(0x07u)        // bits[2:0]

// SHIP_RST (0x09)
#define BQ25180_SHIP_RST_REG_RST_MASK			(0x80u)        // bit7
#define BQ25180_SHIP_RST_EN_RST_SHIP_MASK		(0x60u)        // bits[6:5]

// TS_CONTROL (0x0B)
#define BQ25180_TS_CTRL_TS_HOT_MASK			(0xC0u)        // bits[7:6]
#define BQ25180_TS_CTRL_TS_COLD_MASK			(0x30u)        // bits[5:4]
#define BQ25180_TS_CTRL_TS_WARM_DISABLE_MASK		(0x08u)        // bit3
#define BQ25180_TS_CTRL_TS_COOL_DISABLE_MASK		(0x04u)        // bit2
#define BQ25180_TS_CTRL_TS_ICHG_MASK			(0x02u)        // bit1
#define BQ25180_TS_CTRL_TS_VRCG_MASK			(0x01u)        // bit0

// Encoding parameters (set per datasheet revision)
// Battery regulation voltage: 3.5V..4.65V, 10mV/LSB
#ifndef BQ25180_VREG_MIN_MV
#define BQ25180_VREG_MIN_MV 3500
#endif
#ifndef BQ25180_VREG_MAX_MV
#define BQ25180_VREG_MAX_MV 4650
#endif
#ifndef BQ25180_VREG_STEP_MV
#define BQ25180_VREG_STEP_MV 10
#endif

// Fast charge current segmented steps
#ifndef BQ25180_ICHG_MIN_MA
#define BQ25180_ICHG_MIN_MA 0
#endif
#ifndef BQ25180_ICHG_MAX_MA
#define BQ25180_ICHG_MAX_MA 1000
#endif
#ifndef BQ25180_ICHG_STEP_MA
#define BQ25180_ICHG_STEP_MA 10
#endif

// Precharge and termination (nibbles in CHARGECTRL0)
#ifndef BQ25180_IPRE_STEP_MA
#define BQ25180_IPRE_STEP_MA 5
#endif
#ifndef BQ25180_IPRE_MAX_MA
#define BQ25180_IPRE_MAX_MA (BQ25180_IPRE_STEP_MA * 15)
#endif
#ifndef BQ25180_ITERM_STEP_MA
#define BQ25180_ITERM_STEP_MA 5
#endif
#ifndef BQ25180_ITERM_MAX_MA
#define BQ25180_ITERM_MAX_MA (BQ25180_ITERM_STEP_MA * 15)
#endif

// Input current limit table (encoded elsewhere)
#ifndef BQ25180_IINLIM_STEP_MA
#define BQ25180_IINLIM_STEP_MA 100
#endif
#ifndef BQ25180_IINLIM_MAX_MA
#define BQ25180_IINLIM_MAX_MA (BQ25180_IINLIM_STEP_MA * 15)
#endif

// Forward-declare opaque context
typedef struct bq25180_ctx bq25180_ctx_t;

// Transport: Write and Read a single 8-bit register
typedef int (*bq25180_i2c_write_fn)(void *user, uint8_t i2c_addr, uint8_t reg, uint8_t value);
typedef int (*bq25180_i2c_read_fn)(void *user, uint8_t i2c_addr, uint8_t reg, uint8_t *value);

// Monotonic sleep/delay in milliseconds (optional, may be NULL)
typedef void (*bq25180_delay_ms_fn)(void *user, uint32_t ms);

// Driver context
struct bq25180_ctx {
	uint8_t i2c_addr;                    // 7-bit address
	void *user;                          // User pointer handed back to callbacks
	bq25180_i2c_write_fn write_reg;      // Required
	bq25180_i2c_read_fn  read_reg;       // Required
	bq25180_delay_ms_fn  delay_ms;       // Optional
};

// Result codes (0 == OK). Negative values are errors.
enum {
	BQ25180_OK = 0,
	BQ25180_ERR_PARAM = -1,
	BQ25180_ERR_I2C   = -2
};

// Minimal configuration knobs (extend as needed)
typedef struct {
	// Battery regulation voltage in millivolts. Device supports step sizing per datasheet.
	uint16_t vreg_mv;
	// Fast charge current in milliamps.
	uint16_t ichg_ma;
	// Precharge current in milliamps (optional; 0 to keep default)
	uint16_t ipre_ma;
	// Termination current in milliamps (optional; 0 to keep default)
	uint16_t iterm_ma;
	// Enable charging when set true
	uint8_t chg_enable;
} bq25180_config_t;

// Public API
int bq25180_init(bq25180_ctx_t *ctx,
		uint8_t i2c_addr,
		void *user,
		bq25180_i2c_write_fn write_fn,
		bq25180_i2c_read_fn read_fn,
		bq25180_delay_ms_fn delay_fn);

int bq25180_read_id(bq25180_ctx_t *ctx, uint8_t *mask_id);
int bq25180_software_reset(bq25180_ctx_t *ctx);

int bq25180_apply_config(bq25180_ctx_t *ctx, const bq25180_config_t *cfg);

// Raw register access helpers
int bq25180_write_register(bq25180_ctx_t *ctx, bq25180_register_t reg, uint8_t value);
int bq25180_read_register(bq25180_ctx_t *ctx, bq25180_register_t reg, uint8_t *value);

// Read-modify-write helper: updates masked bits in a register
int bq25180_update_register_bits(bq25180_ctx_t *ctx, bq25180_register_t reg, uint8_t mask, uint8_t value);

// Simple convenience setters (encode values per datasheet ranges)
int bq25180_set_vreg_mv(bq25180_ctx_t *ctx, uint16_t vreg_mv);
int bq25180_set_ichg_ma(bq25180_ctx_t *ctx, uint16_t ichg_ma);

// Additional setters
int bq25180_enable_charging(bq25180_ctx_t *ctx, int enable);

// Termination and precharge (CHARGECTRL0 @ 0x05)
typedef enum {
	BQ25180_ITERM_DISABLE = 0,
	BQ25180_ITERM_5PCT,
	BQ25180_ITERM_10PCT,
	BQ25180_ITERM_20PCT
} bq25180_iterm_pct_t;
int bq25180_set_termination_pct(bq25180_ctx_t *ctx, bq25180_iterm_pct_t pct);
// match_term = 0 => precharge = 2x termination; match_term = 1 => precharge = 1x termination
int bq25180_set_precharge_matches_term(bq25180_ctx_t *ctx, int match_term);

// Convenience: approximate ITERM percent from desired mA vs current ICHG
int bq25180_set_termination_current_ma(bq25180_ctx_t *ctx, uint16_t iterm_ma);

// Input current limit (TMR_ILIM @ 0x08, ILIM_2:0)
int bq25180_set_input_current_limit_ma(bq25180_ctx_t *ctx, uint16_t iin_ma);

typedef enum {
	BQ25180_SFTMR_OFF = 0,
	BQ25180_SFTMR_3H,
	BQ25180_SFTMR_6H,
	BQ25180_SFTMR_12H
} bq25180_safety_timer_t;
int bq25180_set_safety_timer(bq25180_ctx_t *ctx, bq25180_safety_timer_t tmr);

int bq25180_ts_enable(bq25180_ctx_t *ctx, int enable);

typedef enum {
	BQ25180_WD_OFF = 0,
	BQ25180_WD_40S,
	BQ25180_WD_160S
} bq25180_watchdog_t;
int bq25180_set_watchdog(bq25180_ctx_t *ctx, bq25180_watchdog_t wd);

// Diagnostics: read a contiguous block of registers starting at 0
// out_len should be <= 10 to cover 0x00..0x09
int bq25180_dump_registers(bq25180_ctx_t *ctx, uint8_t *out, size_t out_len);

// SHIP_RST helpers
int bq25180_software_reset(bq25180_ctx_t *ctx); // sets REG_RST
typedef enum {
	BQ25180_SHIP_CFG_NONE = 0,     // no action
	BQ25180_SHIP_CFG_SHIP_EN = 1,  // enable ship on MR action (datasheet-defined)
	BQ25180_SHIP_CFG_SHUTDOWN = 2  // enable shutdown option
} bq25180_ship_cfg_t;
int bq25180_configure_ship_reset(bq25180_ctx_t *ctx, bq25180_ship_cfg_t cfg);

// Optional: MCU-driven MR long-press to enter ship
// Configure MR long-press duration code (TMR_ILIM.MR_LPRESS[1:0])
int bq25180_set_mr_long_press_code(bq25180_ctx_t *ctx, uint8_t code_0_to_3);

// User callback to perform an MR long-press for 'hold_ms' (active-low or high per board).
// Should return 0 on success.
typedef int (*bq25180_mr_long_press_fn)(void *user, uint32_t hold_ms);

// Convenience: prepare EN_RST_SHIP=SHIP_EN then request MR long-press via callback.
// hold_ms should match your MR_LPRESS setting (datasheet table).
int bq25180_enter_ship_via_mr(bq25180_ctx_t *ctx,
		bq25180_mr_long_press_fn mr_press,
		uint32_t hold_ms);

// ---------- Getters ----------
int bq25180_get_vreg_mv(bq25180_ctx_t *ctx, uint16_t *out_mv);
int bq25180_get_ichg_ma(bq25180_ctx_t *ctx, uint16_t *out_ma);
int bq25180_get_input_current_limit_ma(bq25180_ctx_t *ctx, uint16_t *out_ma);
int bq25180_get_termination_pct(bq25180_ctx_t *ctx, bq25180_iterm_pct_t *out_pct);
int bq25180_get_precharge_matches_term(bq25180_ctx_t *ctx, int *out_match);
int bq25180_get_safety_timer(bq25180_ctx_t *ctx, bq25180_safety_timer_t *out_tmr);
int bq25180_get_watchdog(bq25180_ctx_t *ctx, bq25180_watchdog_t *out_wd);

// ---------- CHARGECTRL0 extra setters ----------
// Raw code setters to avoid guessing absolute units; see datasheet tables
int bq25180_set_vindpm_code(bq25180_ctx_t *ctx, uint8_t code_0_to_3);     // writes bits[3:2]
int bq25180_set_therm_reg_code(bq25180_ctx_t *ctx, uint8_t code_0_to_3);  // writes bits[1:0]

// ---------- CHARGECTRL1 extras ----------
int bq25180_set_buvlo_code(bq25180_ctx_t *ctx, uint8_t code_0_to_7);      // bits[5:3]
int bq25180_get_buvlo_code(bq25180_ctx_t *ctx, uint8_t *out_code);
int bq25180_set_ibat_ocp_code(bq25180_ctx_t *ctx, uint8_t code_0_to_3);    // bits[7:6]
int bq25180_get_ibat_ocp_code(bq25180_ctx_t *ctx, uint8_t *out_code);
int bq25180_set_status_int_mask(bq25180_ctx_t *ctx, int mask_on);
int bq25180_set_ilim_int_mask(bq25180_ctx_t *ctx, int mask_on);
int bq25180_set_vdpm_int_mask(bq25180_ctx_t *ctx, int mask_on);

// ---------- TS_CONTROL setters ----------
int bq25180_set_ts_hot_code(bq25180_ctx_t *ctx, uint8_t code_0_to_3);   // bits[7:6]
int bq25180_set_ts_cold_code(bq25180_ctx_t *ctx, uint8_t code_0_to_3);  // bits[5:4]
int bq25180_set_ts_warm_disable(bq25180_ctx_t *ctx, int disable);
int bq25180_set_ts_cool_disable(bq25180_ctx_t *ctx, int disable);
int bq25180_set_ts_derate_ichg_20pct(bq25180_ctx_t *ctx, int enable_20pct); // 1 => 0.2*ICHG, 0 => 0.5*ICHG
int bq25180_set_ts_vrcg_200mv(bq25180_ctx_t *ctx, int enable_200mv);       // 1 => -200mV, 0 => -100mV

// ---------- MASK_ID mask setters ----------
int bq25180_set_interrupt_masks(bq25180_ctx_t *ctx,
		int mask_ts,
		int mask_treg,
		int mask_bat,
		int mask_pg);

// ---------- Status ----------
typedef struct {
	uint8_t stat0;
	uint8_t stat1;
	uint8_t flag0;
} bq25180_status_t;
int bq25180_read_status(bq25180_ctx_t *ctx, bq25180_status_t *out);

typedef struct {
	uint8_t stat0;
	uint8_t stat1;
	uint8_t flag0;
	uint8_t stat0_bits[8];
	uint8_t stat1_bits[8];
	uint8_t flag0_bits[8];
} bq25180_status_decoded_t;
int bq25180_decode_status_bits(const bq25180_status_t *in, bq25180_status_decoded_t *out);
int bq25180_poll_status(bq25180_ctx_t *ctx, bq25180_status_decoded_t *out);

// Named status view using a caller-provided bit mapping. For each field, set
// the corresponding bit index (0..7) in the map; set to 0xFF if not applicable.
typedef struct {
	uint8_t charging_bit_idx_stat0;
	uint8_t charge_done_bit_idx_stat0;
	uint8_t precharge_bit_idx_stat0;
	uint8_t power_good_bit_idx_stat1;
	uint8_t vindpm_active_bit_idx_stat1;
	uint8_t ilim_throttled_bit_idx_stat1;
	uint8_t thermal_regulating_bit_idx_flag0;
	uint8_t battery_uvlo_bit_idx_flag0;
} bq25180_status_map_t;

typedef struct {
	uint8_t charging;
	uint8_t charge_done;
	uint8_t precharge;
	uint8_t power_good;
	uint8_t vindpm_active;
	uint8_t ilim_throttled;
	uint8_t thermal_regulating;
	uint8_t battery_uvlo;
} bq25180_status_named_t;

// Provide a zeroed map (all indices set to 0xFF) for the caller to fill.
static inline void bq25180_status_map_init(bq25180_status_map_t *map) {
	if (!map) return;
	map->charging_bit_idx_stat0 = 0xFF;
	map->charge_done_bit_idx_stat0 = 0xFF;
	map->precharge_bit_idx_stat0 = 0xFF;
	map->power_good_bit_idx_stat1 = 0xFF;
	map->vindpm_active_bit_idx_stat1 = 0xFF;
	map->ilim_throttled_bit_idx_stat1 = 0xFF;
	map->thermal_regulating_bit_idx_flag0 = 0xFF;
	map->battery_uvlo_bit_idx_flag0 = 0xFF;
}

int bq25180_decode_status_named(const bq25180_status_t *in,
		const bq25180_status_map_t *map,
		bq25180_status_named_t *out);

// Provide a reasonable default mapping for common interpretations.
// NOTE: Verify against your datasheet tables and adjust if needed.
void bq25180_status_map_default(bq25180_status_map_t *map);

// ---------- Watchdog service ----------
// Services the watchdog by performing a safe, no-op write to IC_CTRL.
int bq25180_kick_watchdog(bq25180_ctx_t *ctx);

// ---------- High-level state classification ----------
typedef enum {
	BQ25180_STATE_IDLE = 0,
	BQ25180_STATE_PRECHARGE,
	BQ25180_STATE_CHARGING,
	BQ25180_STATE_DONE
} bq25180_charge_state_t;

int bq25180_classify_state(const bq25180_status_named_t *named, bq25180_charge_state_t *out_state);

// ---------- Profile helpers (presets) ----------
int bq25180_apply_profile_liion_standard(bq25180_ctx_t *ctx,
		uint16_t vreg_mv,
		uint16_t ichg_ma,
		uint16_t iinlim_ma,
		bq25180_safety_timer_t timer);

int bq25180_apply_profile_thermal_quiet(bq25180_ctx_t *ctx,
		uint16_t vreg_mv,
		uint16_t ichg_ma,
		uint16_t iinlim_ma);

int bq25180_apply_profile_adapter_limited(bq25180_ctx_t *ctx,
		uint16_t iinlim_ma,
		uint16_t ichg_ma_cap);

#ifdef __cplusplus
}
#endif

#endif // BQ25180_H


