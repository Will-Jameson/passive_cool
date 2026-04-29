#include "tsl2561.h"

/*
 * Command byte format: CMD(1) | CLEAR(0) | WORD(0/1) | BLOCK(0) | REG[3:0]
 */
#define TSL2561_CMD         0x80
#define TSL2561_CMD_WORD    0xA0  /* CMD | WORD — reads/writes 2 bytes */

#define TSL2561_REG_CONTROL 0x00
#define TSL2561_REG_TIMING  0x01
#define TSL2561_REG_DATA0   0x0C  /* broadband (visible + IR) */
#define TSL2561_REG_DATA1   0x0E  /* IR only */

#define TSL2561_POWER_ON    0x03
#define TSL2561_TIMING_402MS_1X 0x02  /* 402 ms integration, 1× gain */

HAL_StatusTypeDef tsl2561_init(I2C_HandleTypeDef *hi2c, uint8_t addr)
{
    uint8_t val;
    HAL_StatusTypeDef ret;

    val = TSL2561_POWER_ON;
    ret = HAL_I2C_Mem_Write(hi2c, (uint16_t)(addr << 1),
                            TSL2561_CMD | TSL2561_REG_CONTROL,
                            I2C_MEMADD_SIZE_8BIT, &val, 1, 100);
    if (ret != HAL_OK) return ret;

    val = TSL2561_TIMING_402MS_1X;
    ret = HAL_I2C_Mem_Write(hi2c, (uint16_t)(addr << 1),
                            TSL2561_CMD | TSL2561_REG_TIMING,
                            I2C_MEMADD_SIZE_8BIT, &val, 1, 100);
    return ret;
}

HAL_StatusTypeDef tsl2561_read_raw(I2C_HandleTypeDef *hi2c, uint8_t addr,
                                   uint16_t *ch0, uint16_t *ch1)
{
    uint8_t buf[2];
    HAL_StatusTypeDef ret;

    /* WORD read fetches low byte then high byte in one transaction */
    ret = HAL_I2C_Mem_Read(hi2c, (uint16_t)(addr << 1),
                           TSL2561_CMD_WORD | TSL2561_REG_DATA0,
                           I2C_MEMADD_SIZE_8BIT, buf, 2, 100);
    if (ret != HAL_OK) return ret;
    *ch0 = (uint16_t)((buf[1] << 8) | buf[0]);

    ret = HAL_I2C_Mem_Read(hi2c, (uint16_t)(addr << 1),
                           TSL2561_CMD_WORD | TSL2561_REG_DATA1,
                           I2C_MEMADD_SIZE_8BIT, buf, 2, 100);
    if (ret != HAL_OK) return ret;
    *ch1 = (uint16_t)((buf[1] << 8) | buf[0]);

    return HAL_OK;
}

/*
 * Piecewise approximation of the TSL2561 T-package lux formula.
 * Assumes 402 ms integration time, 1× gain (the values set by tsl2561_init).
 * No floating-point library needed beyond basic float arithmetic.
 *
 * ch0 = broadband channel (visible + IR)
 * ch1 = IR-only channel
 */
float tsl2561_calculate_lux(uint16_t ch0, uint16_t ch1)
{
    if (ch0 == 0) return 0.0f;

    float ratio = (float)ch1 / (float)ch0;
    float b, m;

    if      (ratio <= 0.125f) { b = 0.0304f;  m = 0.0272f;  }
    else if (ratio <= 0.250f) { b = 0.0325f;  m = 0.0440f;  }
    else if (ratio <= 0.375f) { b = 0.0351f;  m = 0.0544f;  }
    else if (ratio <= 0.500f) { b = 0.0381f;  m = 0.0624f;  }
    else if (ratio <= 0.610f) { b = 0.0224f;  m = 0.0310f;  }
    else if (ratio <= 0.800f) { b = 0.0128f;  m = 0.0153f;  }
    else if (ratio <= 1.300f) { b = 0.00146f; m = 0.00112f; }
    else                      { b = 0.0f;     m = 0.0f;     }

    float lux = b * (float)ch0 - m * (float)ch1;
    return lux < 0.0f ? 0.0f : lux;
}
