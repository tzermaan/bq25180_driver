#ifndef BQ25180_STM32_ADAPTER_H
#define BQ25180_STM32_ADAPTER_H

#include <stdint.h>
#include "bq25180.h"

// Forward-declare HAL I2C handle from your project
typedef struct __I2C_HandleTypeDef I2C_HandleTypeDef;

typedef struct {
	I2C_HandleTypeDef *i2c;  // e.g., &hi2c1
	void *unused;            // reserved
} bq25180_stm32_bus_t;

// Optional GPIO wrapper for CE (charge enable)
typedef struct {
	void *port;    // GPIO_TypeDef*
	uint16_t pin;  // GPIO_PIN_X
} bq25180_stm32_gpio_t;

// Initialize context using STM32 HAL I2C
int bq25180_stm32_bind(bq25180_ctx_t *ctx,
		bq25180_stm32_bus_t *bus,
		uint8_t i2c_addr);

// Control CE line via HAL GPIO (set enable=1 to enable charge)
void bq25180_stm32_ce_write(const bq25180_stm32_gpio_t *ce, int enable);

// Read device ID (MASK_ID register). Returns 0 on success.
int bq25180_stm32_read_id(bq25180_ctx_t *ctx, uint8_t *mask_id);

#endif // BQ25180_STM32_ADAPTER_H


