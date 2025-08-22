#include "bq25180.h"

// Internal utilities
static int bq25180_validate_ctx(const bq25180_ctx_t *ctx) {
	if (ctx == NULL) return BQ25180_ERR_PARAM;
	if (ctx->write_reg == NULL || ctx->read_reg == NULL) return BQ25180_ERR_PARAM;
	return BQ25180_OK;
}

int bq25180_init(bq25180_ctx_t *ctx,
		uint8_t i2c_addr,
		void *user,
		bq25180_i2c_write_fn write_fn,
		bq25180_i2c_read_fn read_fn,
		bq25180_delay_ms_fn delay_fn) {
	if (ctx == NULL || write_fn == NULL || read_fn == NULL) return BQ25180_ERR_PARAM;
	ctx->i2c_addr = i2c_addr ? i2c_addr : BQ25180_I2C_ADDR;
	ctx->user = user;
	ctx->write_reg = write_fn;
	ctx->read_reg = read_fn;
	ctx->delay_ms = delay_fn;
	return BQ25180_OK;
}

int bq25180_write_register(bq25180_ctx_t *ctx, bq25180_register_t reg, uint8_t value) {
	if (bq25180_validate_ctx(ctx) != BQ25180_OK) return BQ25180_ERR_PARAM;
	int rc = ctx->write_reg(ctx->user, ctx->i2c_addr, (uint8_t)reg, value);
	return (rc == 0) ? BQ25180_OK : BQ25180_ERR_I2C;
}

int bq25180_read_register(bq25180_ctx_t *ctx, bq25180_register_t reg, uint8_t *value) {
	if (bq25180_validate_ctx(ctx) != BQ25180_OK || value == NULL) return BQ25180_ERR_PARAM;
	int rc = ctx->read_reg(ctx->user, ctx->i2c_addr, (uint8_t)reg, value);
	return (rc == 0) ? BQ25180_OK : BQ25180_ERR_I2C;
}

int bq25180_update_register_bits(bq25180_ctx_t *ctx, bq25180_register_t reg, uint8_t mask, uint8_t value) {
	uint8_t regval;
	int rc = bq25180_read_register(ctx, reg, &regval);
	if (rc != BQ25180_OK) return rc;
	regval = (uint8_t)((regval & (uint8_t)~mask) | (value & mask));
	return bq25180_write_register(ctx, reg, regval);
}

int bq25180_read_id(bq25180_ctx_t *ctx, uint8_t *mask_id) {
	if (mask_id == NULL) return BQ25180_ERR_PARAM;
	return bq25180_read_register(ctx, BQ25180_REG_MASK_ID, mask_id);
}

int bq25180_software_reset(bq25180_ctx_t *ctx) {
	// Set REG_RST (SHIP_RST bit7)
	return bq25180_update_register_bits(ctx, BQ25180_REG_SHIP_RST, BQ25180_SHIP_RST_REG_RST_MASK, BQ25180_SHIP_RST_REG_RST_MASK);
}

int bq25180_configure_ship_reset(bq25180_ctx_t *ctx, bq25180_ship_cfg_t cfg) {
	uint8_t field = 0u;
	switch (cfg) {
		case BQ25180_SHIP_CFG_NONE: field = 0x00; break;
		case BQ25180_SHIP_CFG_SHIP_EN: field = 0x20; break;   // EN_RST_SHIP[1:0] = 01
		case BQ25180_SHIP_CFG_SHUTDOWN: field = 0x40; break;  // EN_RST_SHIP[1:0] = 10
		default: return BQ25180_ERR_PARAM;
	}
	return bq25180_update_register_bits(ctx, BQ25180_REG_SHIP_RST, BQ25180_SHIP_RST_EN_RST_SHIP_MASK, field);
}

int bq25180_set_mr_long_press_code(bq25180_ctx_t *ctx, uint8_t code_0_to_3) {
    // TMR_ILIM.MR_LPRESS[1:0] at bits[7:6]
    uint8_t field = (uint8_t)((code_0_to_3 & 0x03) << 6);
    return bq25180_update_register_bits(ctx, BQ25180_REG_TMR_ILIM, BQ25180_TMR_ILIM_MR_LPRESS_MASK, field);
}

int bq25180_enter_ship_via_mr(bq25180_ctx_t *ctx,
        bq25180_mr_long_press_fn mr_press,
        uint32_t hold_ms) {
    if (mr_press == NULL) return BQ25180_ERR_PARAM;
    // Configure ship behavior
    int rc = bq25180_configure_ship_reset(ctx, BQ25180_SHIP_CFG_SHIP_EN);
    if (rc != BQ25180_OK) return rc;
    // Request the board to simulate MR long-press
    return mr_press(ctx ? ctx->user : NULL, hold_ms);
}

// ---------- Getters ----------
int bq25180_get_vreg_mv(bq25180_ctx_t *ctx, uint16_t *out_mv) {
	if (out_mv == NULL) return BQ25180_ERR_PARAM;
	uint8_t v = 0; int rc = bq25180_read_register(ctx, BQ25180_REG_VBAT_CTRL, &v);
	if (rc != BQ25180_OK) return rc;
	uint16_t code = (uint16_t)(v & BQ25180_VBAT_CTRL_VREG_MASK);
	*out_mv = (uint16_t)(BQ25180_VREG_MIN_MV + code * BQ25180_VREG_STEP_MV);
	return BQ25180_OK;
}

int bq25180_get_ichg_ma(bq25180_ctx_t *ctx, uint16_t *out_ma) {
	if (out_ma == NULL) return BQ25180_ERR_PARAM;
	uint8_t v = 0; int rc = bq25180_read_register(ctx, BQ25180_REG_ICHG_CTRL, &v);
	if (rc != BQ25180_OK) return rc;
	uint8_t code = (uint8_t)(v & 0x7F);
	uint16_t ma;
	if (code <= 6) ma = (uint16_t)((code + 1) * 5);
	else ma = (uint16_t)(40 + (code - 7) * 10);
	*out_ma = ma;
	return BQ25180_OK;
}

int bq25180_get_input_current_limit_ma(bq25180_ctx_t *ctx, uint16_t *out_ma) {
	if (out_ma == NULL) return BQ25180_ERR_PARAM;
	uint8_t v = 0; int rc = bq25180_read_register(ctx, BQ25180_REG_TMR_ILIM, &v);
	if (rc != BQ25180_OK) return rc;
	uint8_t code = (uint8_t)(v & BQ25180_TMR_ILIM_ILIM_MASK);
	const uint16_t steps_ma[8] = {50, 100, 200, 300, 400, 500, 700, 1100};
	*out_ma = steps_ma[code & 0x07];
	return BQ25180_OK;
}

int bq25180_get_termination_pct(bq25180_ctx_t *ctx, bq25180_iterm_pct_t *out_pct) {
	if (out_pct == NULL) return BQ25180_ERR_PARAM;
	uint8_t v = 0; int rc = bq25180_read_register(ctx, BQ25180_REG_CHG_CTRL0, &v);
	if (rc != BQ25180_OK) return rc;
	uint8_t code = (uint8_t)((v & BQ25180_CHG_CTRL0_ITERM_MASK) >> 4);
	*out_pct = (bq25180_iterm_pct_t)code;
	return BQ25180_OK;
}

int bq25180_get_precharge_matches_term(bq25180_ctx_t *ctx, int *out_match) {
	if (out_match == NULL) return BQ25180_ERR_PARAM;
	uint8_t v = 0; int rc = bq25180_read_register(ctx, BQ25180_REG_CHG_CTRL0, &v);
	if (rc != BQ25180_OK) return rc;
	*out_match = ((v & BQ25180_CHG_CTRL0_IPRECHG_MASK) ? 1 : 0);
	return BQ25180_OK;
}

int bq25180_get_safety_timer(bq25180_ctx_t *ctx, bq25180_safety_timer_t *out_tmr) {
	if (out_tmr == NULL) return BQ25180_ERR_PARAM;
	uint8_t v = 0; int rc = bq25180_read_register(ctx, BQ25180_REG_IC_CTRL, &v);
	if (rc != BQ25180_OK) return rc;
	uint8_t code = (uint8_t)((v & BQ25180_IC_CTRL_SFTMR_MASK) >> 2);
	switch (code) {
		case 0x00: *out_tmr = BQ25180_SFTMR_3H; break;
		case 0x01: *out_tmr = BQ25180_SFTMR_6H; break;
		case 0x02: *out_tmr = BQ25180_SFTMR_12H; break;
		default:   *out_tmr = BQ25180_SFTMR_OFF; break;
	}
	return BQ25180_OK;
}

int bq25180_get_watchdog(bq25180_ctx_t *ctx, bq25180_watchdog_t *out_wd) {
	if (out_wd == NULL) return BQ25180_ERR_PARAM;
	uint8_t v = 0; int rc = bq25180_read_register(ctx, BQ25180_REG_IC_CTRL, &v);
	if (rc != BQ25180_OK) return rc;
	uint8_t code = (uint8_t)(v & BQ25180_IC_CTRL_WATCHDOG_MASK);
	switch (code) {
		case 0x00: *out_wd = BQ25180_WD_160S; break;
		case 0x01: *out_wd = BQ25180_WD_40S; break;
		default:   *out_wd = BQ25180_WD_OFF; break;
	}
	return BQ25180_OK;
}

// ---------- CHARGECTRL0 extra setters ----------
int bq25180_set_vindpm_code(bq25180_ctx_t *ctx, uint8_t code_0_to_3) {
	uint8_t field = (uint8_t)((code_0_to_3 & 0x03) << 2);
	return bq25180_update_register_bits(ctx, BQ25180_REG_CHG_CTRL0, BQ25180_CHG_CTRL0_VINDPM_MASK, field);
}

int bq25180_set_therm_reg_code(bq25180_ctx_t *ctx, uint8_t code_0_to_3) {
	uint8_t field = (uint8_t)(code_0_to_3 & 0x03);
	return bq25180_update_register_bits(ctx, BQ25180_REG_CHG_CTRL0, BQ25180_CHG_CTRL0_THERM_REG_MASK, field);
}

// ---------- TS_CONTROL setters ----------
int bq25180_set_ts_hot_code(bq25180_ctx_t *ctx, uint8_t code_0_to_3) {
	uint8_t field = (uint8_t)((code_0_to_3 & 0x03) << 6);
	return bq25180_update_register_bits(ctx, BQ25180_REG_TS_CONTROL, BQ25180_TS_CTRL_TS_HOT_MASK, field);
}
int bq25180_set_ts_cold_code(bq25180_ctx_t *ctx, uint8_t code_0_to_3) {
	uint8_t field = (uint8_t)((code_0_to_3 & 0x03) << 4);
	return bq25180_update_register_bits(ctx, BQ25180_REG_TS_CONTROL, BQ25180_TS_CTRL_TS_COLD_MASK, field);
}
int bq25180_set_ts_warm_disable(bq25180_ctx_t *ctx, int disable) {
	uint8_t field = disable ? BQ25180_TS_CTRL_TS_WARM_DISABLE_MASK : 0u;
	return bq25180_update_register_bits(ctx, BQ25180_REG_TS_CONTROL, BQ25180_TS_CTRL_TS_WARM_DISABLE_MASK, field);
}
int bq25180_set_ts_cool_disable(bq25180_ctx_t *ctx, int disable) {
	uint8_t field = disable ? BQ25180_TS_CTRL_TS_COOL_DISABLE_MASK : 0u;
	return bq25180_update_register_bits(ctx, BQ25180_REG_TS_CONTROL, BQ25180_TS_CTRL_TS_COOL_DISABLE_MASK, field);
}
int bq25180_set_ts_derate_ichg_20pct(bq25180_ctx_t *ctx, int enable_20pct) {
	uint8_t field = enable_20pct ? BQ25180_TS_CTRL_TS_ICHG_MASK : 0u;
	return bq25180_update_register_bits(ctx, BQ25180_REG_TS_CONTROL, BQ25180_TS_CTRL_TS_ICHG_MASK, field);
}
int bq25180_set_ts_vrcg_200mv(bq25180_ctx_t *ctx, int enable_200mv) {
	uint8_t field = enable_200mv ? BQ25180_TS_CTRL_TS_VRCG_MASK : 0u;
	return bq25180_update_register_bits(ctx, BQ25180_REG_TS_CONTROL, BQ25180_TS_CTRL_TS_VRCG_MASK, field);
}

// ---------- MASK_ID masks ----------
int bq25180_set_interrupt_masks(bq25180_ctx_t *ctx,
		int mask_ts,
		int mask_treg,
		int mask_bat,
		int mask_pg) {
	uint8_t v = 0; int rc = bq25180_read_register(ctx, BQ25180_REG_MASK_ID, &v);
	if (rc != BQ25180_OK) return rc;
	if (mask_ts  >= 0) v = (uint8_t)((v & ~0x80u) | (mask_ts  ? 0x80u : 0u));
	if (mask_treg>= 0) v = (uint8_t)((v & ~0x40u) | (mask_treg? 0x40u : 0u));
	if (mask_bat >= 0) v = (uint8_t)((v & ~0x20u) | (mask_bat ? 0x20u : 0u));
	if (mask_pg  >= 0) v = (uint8_t)((v & ~0x10u) | (mask_pg  ? 0x10u : 0u));
	return bq25180_write_register(ctx, BQ25180_REG_MASK_ID, v);
}

// ---------- Status ----------
int bq25180_read_status(bq25180_ctx_t *ctx, bq25180_status_t *out) {
	if (out == NULL) return BQ25180_ERR_PARAM;
	int rc = bq25180_read_register(ctx, BQ25180_REG_STAT0, &out->stat0); if (rc != BQ25180_OK) return rc;
	rc = bq25180_read_register(ctx, BQ25180_REG_STAT1, &out->stat1); if (rc != BQ25180_OK) return rc;
	rc = bq25180_read_register(ctx, BQ25180_REG_FLAG0, &out->flag0); if (rc != BQ25180_OK) return rc;
	return BQ25180_OK;
}

int bq25180_decode_status_bits(const bq25180_status_t *in, bq25180_status_decoded_t *out) {
	if (in == NULL || out == NULL) return BQ25180_ERR_PARAM;
	out->stat0 = in->stat0;
	out->stat1 = in->stat1;
	out->flag0 = in->flag0;
	for (int i = 0; i < 8; ++i) {
		out->stat0_bits[i] = (uint8_t)((in->stat0 >> i) & 0x01u);
		out->stat1_bits[i] = (uint8_t)((in->stat1 >> i) & 0x01u);
		out->flag0_bits[i] = (uint8_t)((in->flag0 >> i) & 0x01u);
	}
	return BQ25180_OK;
}

// Convenience: read and decode in one call
int bq25180_poll_status(bq25180_ctx_t *ctx, bq25180_status_decoded_t *out) {
    if (out == NULL) return BQ25180_ERR_PARAM;
    bq25180_status_t raw;
    int rc = bq25180_read_status(ctx, &raw);
    if (rc != BQ25180_OK) return rc;
    return bq25180_decode_status_bits(&raw, out);
}

int bq25180_decode_status_named(const bq25180_status_t *in,
        const bq25180_status_map_t *map,
        bq25180_status_named_t *out) {
    if (in == NULL || map == NULL || out == NULL) return BQ25180_ERR_PARAM;
    // READ_IDX: returns bit value from byte if index 0..7, else 0
    #define READ_IDX(byte, idx) ((uint8_t)(((idx) <= 7) ? (((byte) >> (idx)) & 0x01u) : 0u))
    out->charging           = READ_IDX(in->stat0, map->charging_bit_idx_stat0);
    out->charge_done        = READ_IDX(in->stat0, map->charge_done_bit_idx_stat0);
    out->precharge          = READ_IDX(in->stat0, map->precharge_bit_idx_stat0);
    out->power_good         = READ_IDX(in->stat1, map->power_good_bit_idx_stat1);
    out->vindpm_active      = READ_IDX(in->stat1, map->vindpm_active_bit_idx_stat1);
    out->ilim_throttled     = READ_IDX(in->stat1, map->ilim_throttled_bit_idx_stat1);
    out->thermal_regulating = READ_IDX(in->flag0, map->thermal_regulating_bit_idx_flag0);
    out->battery_uvlo       = READ_IDX(in->flag0, map->battery_uvlo_bit_idx_flag0);
    #undef READ_IDX
    return BQ25180_OK;
}

void bq25180_status_map_default(bq25180_status_map_t *map) {
    if (!map) return;
    bq25180_status_map_init(map);
    // Defaults below are illustrative; verify bit meanings for your revision.
    // STAT0 interpretations
    map->charging_bit_idx_stat0 = 0;        // e.g., charging active
    map->charge_done_bit_idx_stat0 = 1;     // e.g., charge done
    map->precharge_bit_idx_stat0 = 2;       // e.g., precharge region
    // STAT1 interpretations
    map->power_good_bit_idx_stat1 = 0;      // e.g., PG asserted
    map->vindpm_active_bit_idx_stat1 = 1;   // e.g., VINDPM regulation active
    map->ilim_throttled_bit_idx_stat1 = 2;  // e.g., ILIM regulation active
    // FLAG0 interpretations
    map->thermal_regulating_bit_idx_flag0 = 0; // e.g., thermal regulation
    map->battery_uvlo_bit_idx_flag0 = 1;       // e.g., battery UVLO
}

// ---------- Watchdog service ----------
int bq25180_kick_watchdog(bq25180_ctx_t *ctx) {
    // Perform a read-modify-write on IC_CTRL that preserves the current value.
    // This touches the register and can be used as a periodic host activity.
    uint8_t v = 0; int rc = bq25180_read_register(ctx, BQ25180_REG_IC_CTRL, &v);
    if (rc != BQ25180_OK) return rc;
    return bq25180_write_register(ctx, BQ25180_REG_IC_CTRL, v);
}

// ---------- Profile helpers ----------
int bq25180_apply_profile_liion_standard(bq25180_ctx_t *ctx,
        uint16_t vreg_mv,
        uint16_t ichg_ma,
        uint16_t iinlim_ma,
        bq25180_safety_timer_t timer) {
    int rc;
    rc = bq25180_set_vreg_mv(ctx, vreg_mv); if (rc) return rc;
    rc = bq25180_set_ichg_ma(ctx, ichg_ma); if (rc) return rc;
    rc = bq25180_set_input_current_limit_ma(ctx, iinlim_ma); if (rc) return rc;
    rc = bq25180_set_termination_pct(ctx, BQ25180_ITERM_10PCT); if (rc) return rc;
    rc = bq25180_set_precharge_matches_term(ctx, 1); if (rc) return rc; // 1x ITERM
    rc = bq25180_set_safety_timer(ctx, timer); if (rc) return rc;
    rc = bq25180_ts_enable(ctx, 1); if (rc) return rc;
    return bq25180_enable_charging(ctx, 1);
}

int bq25180_apply_profile_thermal_quiet(bq25180_ctx_t *ctx,
        uint16_t vreg_mv,
        uint16_t ichg_ma,
        uint16_t iinlim_ma) {
    int rc;
    // Lower current; keep IINLIM modest; derate aggressively in warm
    rc = bq25180_set_vreg_mv(ctx, vreg_mv); if (rc) return rc;
    rc = bq25180_set_ichg_ma(ctx, ichg_ma); if (rc) return rc;
    rc = bq25180_set_input_current_limit_ma(ctx, iinlim_ma); if (rc) return rc;
    rc = bq25180_set_termination_pct(ctx, BQ25180_ITERM_10PCT); if (rc) return rc;
    rc = bq25180_set_precharge_matches_term(ctx, 1); if (rc) return rc;
    rc = bq25180_ts_enable(ctx, 1); if (rc) return rc;
    rc = bq25180_set_ts_derate_ichg_20pct(ctx, 1); if (rc) return rc;
    rc = bq25180_set_ts_vrcg_200mv(ctx, 1); if (rc) return rc;
    return bq25180_enable_charging(ctx, 1);
}

int bq25180_apply_profile_adapter_limited(bq25180_ctx_t *ctx,
        uint16_t iinlim_ma,
        uint16_t ichg_ma_cap) {
    int rc;
    rc = bq25180_set_input_current_limit_ma(ctx, iinlim_ma); if (rc) return rc;
    rc = bq25180_set_ichg_ma(ctx, ichg_ma_cap); if (rc) return rc;
    rc = bq25180_set_termination_pct(ctx, BQ25180_ITERM_10PCT); if (rc) return rc;
    rc = bq25180_set_precharge_matches_term(ctx, 1); if (rc) return rc;
    return bq25180_enable_charging(ctx, 1);
}

// ---------- CHARGECTRL1 helpers ----------
int bq25180_set_buvlo_code(bq25180_ctx_t *ctx, uint8_t code_0_to_7) {
    uint8_t field = (uint8_t)((code_0_to_7 & 0x07) << 3);
    return bq25180_update_register_bits(ctx, BQ25180_REG_CHG_CTRL1, BQ25180_CHG_CTRL1_BUVLO_MASK, field);
}

int bq25180_get_buvlo_code(bq25180_ctx_t *ctx, uint8_t *out_code) {
    if (out_code == NULL) return BQ25180_ERR_PARAM;
    uint8_t v = 0; int rc = bq25180_read_register(ctx, BQ25180_REG_CHG_CTRL1, &v);
    if (rc != BQ25180_OK) return rc;
    *out_code = (uint8_t)((v & BQ25180_CHG_CTRL1_BUVLO_MASK) >> 3);
    return BQ25180_OK;
}

int bq25180_set_ibat_ocp_code(bq25180_ctx_t *ctx, uint8_t code_0_to_3) {
    uint8_t field = (uint8_t)((code_0_to_3 & 0x03) << 6);
    return bq25180_update_register_bits(ctx, BQ25180_REG_CHG_CTRL1, BQ25180_CHG_CTRL1_IBAT_OCP_MASK, field);
}

int bq25180_get_ibat_ocp_code(bq25180_ctx_t *ctx, uint8_t *out_code) {
    if (out_code == NULL) return BQ25180_ERR_PARAM;
    uint8_t v = 0; int rc = bq25180_read_register(ctx, BQ25180_REG_CHG_CTRL1, &v);
    if (rc != BQ25180_OK) return rc;
    *out_code = (uint8_t)((v & BQ25180_CHG_CTRL1_IBAT_OCP_MASK) >> 6);
    return BQ25180_OK;
}

int bq25180_set_status_int_mask(bq25180_ctx_t *ctx, int mask_on) {
    uint8_t field = mask_on ? BQ25180_CHG_CTRL1_CHG_STATUS_INT_MASK : 0u;
    return bq25180_update_register_bits(ctx, BQ25180_REG_CHG_CTRL1, BQ25180_CHG_CTRL1_CHG_STATUS_INT_MASK, field);
}

int bq25180_set_ilim_int_mask(bq25180_ctx_t *ctx, int mask_on) {
    uint8_t field = mask_on ? BQ25180_CHG_CTRL1_ILIM_INT_MASK : 0u;
    return bq25180_update_register_bits(ctx, BQ25180_REG_CHG_CTRL1, BQ25180_CHG_CTRL1_ILIM_INT_MASK, field);
}

int bq25180_set_vdpm_int_mask(bq25180_ctx_t *ctx, int mask_on) {
    uint8_t field = mask_on ? BQ25180_CHG_CTRL1_VDPM_INT_MASK : 0u;
    return bq25180_update_register_bits(ctx, BQ25180_REG_CHG_CTRL1, BQ25180_CHG_CTRL1_VDPM_INT_MASK, field);
}

int bq25180_classify_state(const bq25180_status_named_t *named, bq25180_charge_state_t *out_state) {
    if (named == NULL || out_state == NULL) return BQ25180_ERR_PARAM;
    if (named->charge_done) { *out_state = BQ25180_STATE_DONE; return BQ25180_OK; }
    if (named->precharge)   { *out_state = BQ25180_STATE_PRECHARGE; return BQ25180_OK; }
    if (named->charging)    { *out_state = BQ25180_STATE_CHARGING; return BQ25180_OK; }
    *out_state = BQ25180_STATE_IDLE;
    return BQ25180_OK;
}

// ---------- SYS_REG raw access ----------
int bq25180_read_sys_reg(bq25180_ctx_t *ctx, uint8_t *out_value) {
    if (out_value == NULL) return BQ25180_ERR_PARAM;
    return bq25180_read_register(ctx, BQ25180_REG_SYS_REG, out_value);
}

int bq25180_write_sys_reg(bq25180_ctx_t *ctx, uint8_t value) {
    return bq25180_write_register(ctx, BQ25180_REG_SYS_REG, value);
}

// Helpers to encode common fields according to datasheet tables
static uint8_t bq25180_encode_vreg_mv(uint16_t vreg_mv) {
	if (vreg_mv < BQ25180_VREG_MIN_MV) vreg_mv = BQ25180_VREG_MIN_MV;
	if (vreg_mv > BQ25180_VREG_MAX_MV) vreg_mv = BQ25180_VREG_MAX_MV;
	uint16_t delta = (uint16_t)(vreg_mv - BQ25180_VREG_MIN_MV);
	uint8_t code = (uint8_t)(delta / BQ25180_VREG_STEP_MV);
	return (uint8_t)(code & 0x7F); // 7-bit field example
}

static uint8_t bq25180_encode_ichg_ma(uint16_t ichg_ma) {
	if (ichg_ma < 0) ichg_ma = 0;
	if (ichg_ma > 1000) ichg_ma = 1000;
	// Per datasheet:
	// codes 0..6 => 5,10,15,20,25,30,35mA (5mA * (code+1))
	// code 7 => 40mA; 8..? => +10mA steps up to 1000mA
	if (ichg_ma <= 35) {
		uint8_t code = (uint8_t)((ichg_ma + 4) / 5); // round to nearest 5mA step
		if (code == 0) code = 1;
		if (code > 7) code = 7; // 35mA corresponds to code 7? (see mapping below)
		return (uint8_t)(code - 1);
	} else {
		// For >=40mA, code 7 => 40mA; subsequent codes add 10mA
		uint8_t code = 7 + (uint8_t)((ichg_ma - 40 + 5) / 10); // round to nearest 10mA
		if (code > 0x7F) code = 0x7F;
		return (uint8_t)code;
	}
}

int bq25180_set_vreg_mv(bq25180_ctx_t *ctx, uint16_t vreg_mv) {
	uint8_t code = bq25180_encode_vreg_mv(vreg_mv);
	return bq25180_write_register(ctx, BQ25180_REG_VBAT_CTRL, code);
}

int bq25180_set_ichg_ma(bq25180_ctx_t *ctx, uint16_t ichg_ma) {
	uint8_t code = bq25180_encode_ichg_ma(ichg_ma);
	return bq25180_write_register(ctx, BQ25180_REG_ICHG_CTRL, code);
}

int bq25180_enable_charging(bq25180_ctx_t *ctx, int enable) {
	// ICHG_CTRL bit7 CHG_DIS: 0=enabled, 1=disabled
	uint8_t mask = BQ25180_ICHG_CTRL_CHG_DIS_MASK;
	uint8_t value = enable ? 0u : BQ25180_ICHG_CTRL_CHG_DIS_MASK;
	return bq25180_update_register_bits(ctx, BQ25180_REG_ICHG_CTRL, mask, value);
}

// Encoding helpers for ipre/iterm/iinlim are placeholder clamps; update with datasheet-accurate tables
static uint8_t bq25180_encode_ipre_ma(uint16_t ipre_ma) {
	if (ipre_ma > BQ25180_IPRE_MAX_MA) ipre_ma = BQ25180_IPRE_MAX_MA;
	uint8_t code = (uint8_t)(ipre_ma / BQ25180_IPRE_STEP_MA);
	if (code > 0x0F) code = 0x0F;
	return (uint8_t)(code << 4); // upper nibble
}

static uint8_t bq25180_encode_iterm_ma(uint16_t iterm_ma) {
	if (iterm_ma > BQ25180_ITERM_MAX_MA) iterm_ma = BQ25180_ITERM_MAX_MA;
	uint8_t code = (uint8_t)(iterm_ma / BQ25180_ITERM_STEP_MA);
	if (code > 0x0F) code = 0x0F;
	return (uint8_t)(code & 0x0F); // lower nibble
}

static uint8_t bq25180_encode_iin_ma(uint16_t iin_ma) {
	// Map to ILIM_2:0 codes per datasheet Table 8-17
	// 000=50, 001=100, 010=200, 011=300, 100=400, 101=500, 110=700, 111=1100
	const uint16_t steps_ma[8] = {50, 100, 200, 300, 400, 500, 700, 1100};
	uint8_t best = 0;
	uint16_t best_diff = 0xFFFF;
	for (uint8_t code = 0; code < 8; ++code) {
		uint16_t val = steps_ma[code];
		uint16_t diff = (val > iin_ma) ? (val - iin_ma) : (iin_ma - val);
		if (diff < best_diff) { best_diff = diff; best = code; }
	}
	return (uint8_t)(best & 0x07);
}

int bq25180_set_precharge_matches_term(bq25180_ctx_t *ctx, int match_term) {
	uint8_t value = match_term ? BQ25180_CHG_CTRL0_IPRECHG_MASK : 0u;
	return bq25180_update_register_bits(ctx, BQ25180_REG_CHG_CTRL0, BQ25180_CHG_CTRL0_IPRECHG_MASK, value);
}

int bq25180_set_termination_pct(bq25180_ctx_t *ctx, bq25180_iterm_pct_t pct) {
	uint8_t field;
	switch (pct) {
		case BQ25180_ITERM_DISABLE: field = 0x00; break; // 00
		case BQ25180_ITERM_5PCT:    field = 0x10; break; // 01 in bits [5:4]
		case BQ25180_ITERM_10PCT:   field = 0x20; break; // 10
		case BQ25180_ITERM_20PCT:   field = 0x30; break; // 11
		default: return BQ25180_ERR_PARAM;
	}
	return bq25180_update_register_bits(ctx, BQ25180_REG_CHG_CTRL0, BQ25180_CHG_CTRL0_ITERM_MASK, field);
}

int bq25180_set_input_current_limit_ma(bq25180_ctx_t *ctx, uint16_t iin_ma) {
	uint8_t code = bq25180_encode_iin_ma(iin_ma);
	return bq25180_update_register_bits(ctx, BQ25180_REG_TMR_ILIM, BQ25180_TMR_ILIM_ILIM_MASK, code);
}

int bq25180_set_safety_timer(bq25180_ctx_t *ctx, bq25180_safety_timer_t tmr) {
	uint8_t field = 0u;
	switch (tmr) {
		case BQ25180_SFTMR_OFF: field = 0x0Cu; break; // 11 per datasheet: disable
		case BQ25180_SFTMR_3H:  field = 0x00u; break; // 00
		case BQ25180_SFTMR_6H:  field = 0x04u; break; // 01
		case BQ25180_SFTMR_12H: field = 0x08u; break; // 10
		default: return BQ25180_ERR_PARAM;
	}
	return bq25180_update_register_bits(ctx, BQ25180_REG_IC_CTRL, BQ25180_IC_CTRL_SFTMR_MASK, field);
}

int bq25180_ts_enable(bq25180_ctx_t *ctx, int enable) {
	uint8_t value = enable ? BQ25180_IC_CTRL_TS_EN_MASK : 0u;
	return bq25180_update_register_bits(ctx, BQ25180_REG_IC_CTRL, BQ25180_IC_CTRL_TS_EN_MASK, value);
}

int bq25180_set_watchdog(bq25180_ctx_t *ctx, bq25180_watchdog_t wd) {
	uint8_t field;
	switch (wd) {
		case BQ25180_WD_OFF:  field = 0x03u; break;
		case BQ25180_WD_40S:  field = 0x01u; break;
		case BQ25180_WD_160S: field = 0x00u; break;
		default: return BQ25180_ERR_PARAM;
	}
	return bq25180_update_register_bits(ctx, BQ25180_REG_IC_CTRL, BQ25180_IC_CTRL_WATCHDOG_MASK, field);
}

int bq25180_dump_registers(bq25180_ctx_t *ctx, uint8_t *out, size_t out_len) {
	if (bq25180_validate_ctx(ctx) != BQ25180_OK || out == NULL || out_len == 0) return BQ25180_ERR_PARAM;
	if (out_len > 13) out_len = 13; // 0x00..0x0C
	for (size_t i = 0; i < out_len; ++i) {
		int rc = bq25180_read_register(ctx, (bq25180_register_t)i, &out[i]);
		if (rc != BQ25180_OK) return rc;
	}
	return BQ25180_OK;
}

int bq25180_apply_config(bq25180_ctx_t *ctx, const bq25180_config_t *cfg) {
	if (bq25180_validate_ctx(ctx) != BQ25180_OK || cfg == NULL) return BQ25180_ERR_PARAM;

	int rc;
	uint8_t reg;

	// VBAT_CTRL: set regulation voltage
	reg = bq25180_encode_vreg_mv(cfg->vreg_mv);
	rc = bq25180_write_register(ctx, BQ25180_REG_VBAT_CTRL, reg);
	if (rc != BQ25180_OK) return rc;

	// ICHG_CTRL: set fast charge current
	reg = bq25180_encode_ichg_ma(cfg->ichg_ma);
	rc = bq25180_write_register(ctx, BQ25180_REG_ICHG_CTRL, reg);
	if (rc != BQ25180_OK) return rc;

	// CHG_CTRL0/1 and others: apply termination percentage and precharge ratio
	if (cfg->ichg_ma && cfg->iterm_ma) {
		uint32_t pct = (uint32_t)cfg->iterm_ma * 100u / (uint32_t)cfg->ichg_ma;
		bq25180_iterm_pct_t sel = BQ25180_ITERM_10PCT;
		uint32_t diff5 = (pct > 5) ? (pct - 5) : (5 - pct);
		uint32_t diff10 = (pct > 10) ? (pct - 10) : (10 - pct);
		uint32_t diff20 = (pct > 20) ? (pct - 20) : (20 - pct);
		if (diff5 <= diff10 && diff5 <= diff20) sel = BQ25180_ITERM_5PCT;
		else if (diff10 <= diff20) sel = BQ25180_ITERM_10PCT;
		else sel = BQ25180_ITERM_20PCT;
		rc = bq25180_set_termination_pct(ctx, sel);
		if (rc != BQ25180_OK) return rc;
		// Precharge ratio: choose 1x if ipre ~= iterm; else 2x
		int match_term = (cfg->ipre_ma && (cfg->ipre_ma <= cfg->iterm_ma + (cfg->iterm_ma/5)));
		rc = bq25180_set_precharge_matches_term(ctx, match_term);
		if (rc != BQ25180_OK) return rc;
	}
	// Enable/disable charging
	rc = bq25180_enable_charging(ctx, cfg->chg_enable ? 1 : 0);
	if (rc != BQ25180_OK) return rc;

	return BQ25180_OK;
}


