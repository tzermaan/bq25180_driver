#include "bq25180_stm32_adapter.h"

// Include the correct HAL header for your MCU family in your project build
#include "stm32xx_hal.h" // Replace with e.g. "stm32f4xx_hal.h" in your project

static int stm32_i2c_write(void *user, uint8_t addr7, uint8_t reg, uint8_t value) {
	bq25180_stm32_bus_t *bus = (bq25180_stm32_bus_t *)user;
	uint8_t buf[2] = { reg, value };
	return (HAL_I2C_Master_Transmit(bus->i2c, (uint16_t)(addr7 << 1), buf, 2, 100) == HAL_OK) ? 0 : -1;
}

static int stm32_i2c_read(void *user, uint8_t addr7, uint8_t reg, uint8_t *value) {
	bq25180_stm32_bus_t *bus = (bq25180_stm32_bus_t *)user;
	if (HAL_I2C_Master_Transmit(bus->i2c, (uint16_t)(addr7 << 1), &reg, 1, 100) != HAL_OK) return -1;
	return (HAL_I2C_Master_Receive(bus->i2c, (uint16_t)(addr7 << 1), value, 1, 100) == HAL_OK) ? 0 : -1;
}

static void stm32_delay(void *user, uint32_t ms) {
	(void)user;
	HAL_Delay(ms);
}

int bq25180_stm32_bind(bq25180_ctx_t *ctx,
		bq25180_stm32_bus_t *bus,
		uint8_t i2c_addr) {
	if (ctx == NULL || bus == NULL || bus->i2c == NULL) return BQ25180_ERR_PARAM;
	return bq25180_init(ctx,
			(i2c_addr ? i2c_addr : BQ25180_I2C_ADDR),
			(void *)bus,
			stm32_i2c_write,
			stm32_i2c_read,
			stm32_delay);
}

void bq25180_stm32_ce_write(const bq25180_stm32_gpio_t *ce, int enable) {
	if (!ce || !ce->port) return;
	HAL_GPIO_WritePin((GPIO_TypeDef *)ce->port, ce->pin, enable ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

int bq25180_stm32_read_id(bq25180_ctx_t *ctx, uint8_t *mask_id) {
	return bq25180_read_id(ctx, mask_id);
}


