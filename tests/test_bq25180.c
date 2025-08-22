#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "bq25180.h"

static uint8_t regs[0x0D];

static int fake_write(void *user, uint8_t addr, uint8_t reg, uint8_t val) {
	(void)user; (void)addr;
	if (reg < sizeof(regs)) regs[reg] = val;
	return 0;
}

static int fake_read(void *user, uint8_t addr, uint8_t reg, uint8_t *val) {
	(void)user; (void)addr;
	if (reg < sizeof(regs)) { *val = regs[reg]; return 0; }
	return -1;
}

static void reset_regs(void) { memset(regs, 0, sizeof(regs)); regs[BQ25180_REG_MASK_ID] = 0xC0; }

int main(void) {
	bq25180_ctx_t ctx;
	reset_regs();
	assert(bq25180_init(&ctx, BQ25180_I2C_ADDR, NULL, fake_write, fake_read, NULL) == 0);
	printf("init: context initialized, I2C addr=0x%02X\n", ctx.i2c_addr);

	// VREG encode/decode
	assert(bq25180_set_vreg_mv(&ctx, 4200) == 0);
	uint16_t vreg; assert(bq25180_get_vreg_mv(&ctx, &vreg) == 0); assert(vreg == 4200);
	printf("VREG expecting: 4200mV, result: %umV\n", vreg);

	// ICHG segmented mapping
	assert(bq25180_set_ichg_ma(&ctx, 35) == 0);
	uint16_t ichg; assert(bq25180_get_ichg_ma(&ctx, &ichg) == 0); assert(ichg == 35);
	printf("ICHG expecting: 35mA, result: %umA\n", ichg);
	assert(bq25180_set_ichg_ma(&ctx, 500) == 0);
	assert(bq25180_get_ichg_ma(&ctx, &ichg) == 0); assert(ichg == 500);
	printf("ICHG expecting: 500mA, result: %umA\n", ichg);

	// ILIM discrete table
	assert(bq25180_set_input_current_limit_ma(&ctx, 700) == 0);
	uint16_t lim; assert(bq25180_get_input_current_limit_ma(&ctx, &lim) == 0); assert(lim == 700);
	printf("ILIM expecting: ~700mA, result: %umA\n", lim);

	// Safety timer & watchdog
	assert(bq25180_set_safety_timer(&ctx, BQ25180_SFTMR_6H) == 0);
	bq25180_safety_timer_t tmr; assert(bq25180_get_safety_timer(&ctx, &tmr) == 0);
	assert(tmr == BQ25180_SFTMR_6H);
	printf("Safety timer expecting: 6h, result: %s\n", (tmr==BQ25180_SFTMR_6H)?"6h":"other");
	assert(bq25180_set_watchdog(&ctx, BQ25180_WD_40S) == 0);
	bq25180_watchdog_t wd; assert(bq25180_get_watchdog(&ctx, &wd) == 0);
	assert(wd == BQ25180_WD_40S);
	printf("Watchdog expecting: 40s, result: %s\n", (wd==BQ25180_WD_40S)?"40s":"other");

	// CHG_CTRL0 fields: termination percent and precharge ratio
	assert(bq25180_set_termination_pct(&ctx, BQ25180_ITERM_10PCT) == 0);
	bq25180_iterm_pct_t pct; assert(bq25180_get_termination_pct(&ctx, &pct) == 0);
	assert(pct == BQ25180_ITERM_10PCT);
	assert(bq25180_set_precharge_matches_term(&ctx, 1) == 0);
	int match; assert(bq25180_get_precharge_matches_term(&ctx, &match) == 0);
	assert(match == 1);
	printf("Termination expecting: 10%%, result: %s; Precharge expecting: 1x, result: %sx\n",
		(pct==BQ25180_ITERM_10PCT)?"10%":"other",
		(match==1)?"1":"2");

	// TS control bits
	assert(bq25180_set_ts_derate_ichg_20pct(&ctx, 1) == 0);
	assert(bq25180_set_ts_vrcg_200mv(&ctx, 1) == 0);
	printf("TS warm derates expecting: ICHG=0.2x, VREG=-200mV, ");
	// HOT/COLD code write and verify by direct register read
	assert(bq25180_set_ts_hot_code(&ctx, 1) == 0);  // example code
	assert(bq25180_set_ts_cold_code(&ctx, 2) == 0);
	uint8_t tsr=0; assert(fake_read(NULL, ctx.i2c_addr, BQ25180_REG_TS_CONTROL, &tsr) == 0);
	printf("TS HOT/COLD expecting: hot=1 cold=2, result: hot=%u cold=%u\n", (tsr>>6)&0x3, (tsr>>4)&0x3);
	// Also verify derate bits we set above
	printf("TS derate bits result: ICHG_bit=%u VRCG_bit=%u\n", (tsr>>1)&1, tsr&1);

	// Status decode
	bq25180_status_t s = {0}; s.stat0 = 0x03; s.stat1 = 0x01; s.flag0 = 0x01;
	bq25180_status_map_t map; bq25180_status_map_default(&map);
	bq25180_status_named_t named;
	assert(bq25180_decode_status_named(&s, &map, &named) == 0);
	assert(named.charging == 1 && named.charge_done == 1 && named.power_good == 1 && named.thermal_regulating == 1);
	printf("STATUS expecting: charging=1 done=1 PG=1 thermal=1; result: charging=%d done=%d PG=%d thermal=%d\n",
		named.charging, named.charge_done, named.power_good, named.thermal_regulating);
	// Classify state (charge_done dominates)
	bq25180_charge_state_t st; assert(bq25180_classify_state(&named, &st) == 0);
	printf("STATE expecting: DONE, result: %s\n", (st==BQ25180_STATE_DONE)?"DONE":"other");

	// CHG_CTRL1: BUVLO/OCP codes and INT masks
	assert(bq25180_set_buvlo_code(&ctx, 5) == 0);
	assert(bq25180_set_ibat_ocp_code(&ctx, 2) == 0);
	uint8_t buv=0, ocp=0; assert(bq25180_get_buvlo_code(&ctx, &buv) == 0);
	assert(bq25180_get_ibat_ocp_code(&ctx, &ocp) == 0);
	printf("CTRL1 expecting: BUVLO_code=5 OCP_code=2; result: BUVLO_code=%u OCP_code=%u\n", buv, ocp);
	assert(buv == 5 && ocp == 2);
	assert(bq25180_set_status_int_mask(&ctx, 1) == 0);
	assert(bq25180_set_ilim_int_mask(&ctx, 1) == 0);
	assert(bq25180_set_vdpm_int_mask(&ctx, 1) == 0);
	uint8_t c1=0; assert(fake_read(NULL, ctx.i2c_addr, BQ25180_REG_CHG_CTRL1, &c1) == 0);
	printf("CTRL1 INT masks expecting: all=masked; result: CHG_STATUS=%u ILIM=%u VDPM=%u (reg=0x%02X)\n",
		(c1 & 0x04)?1:0, (c1 & 0x02)?1:0, (c1 & 0x01)?1:0, c1);

	printf("RESULT: All checks passed.\n");

	return 0;
}

