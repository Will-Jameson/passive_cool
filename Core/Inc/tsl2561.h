#ifndef TSL2561_H
#define TSL2561_H

#include "stm32f3xx_hal.h"
#include <stdint.h>

/*
 * I2C address is set by the ADDR pin:
 *   ADDR → GND : 0x29
 *   ADDR → float: 0x39  (avoid — unreliable)
 *   ADDR → VCC : 0x49
 *
 * Wire your two sensors to GND and VCC respectively.
 */
#define TSL2561_ADDR_LOW    0x29
#define TSL2561_ADDR_FLOAT  0x39
#define TSL2561_ADDR_HIGH   0x49

HAL_StatusTypeDef tsl2561_init(I2C_HandleTypeDef *hi2c, uint8_t addr);
HAL_StatusTypeDef tsl2561_read_raw(I2C_HandleTypeDef *hi2c, uint8_t addr,
                                   uint16_t *ch0, uint16_t *ch1);
float tsl2561_calculate_lux(uint16_t ch0, uint16_t ch1);

#endif /* TSL2561_H */
