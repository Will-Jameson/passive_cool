#include "bme280.h"
#include <string.h>

/* ── Internal timeout ──────────────────────────────────────────────────── */
#define BME280_TIMEOUT_MS   100u
#define BME280_MEAS_DELAY   10u     /* ms to wait for measuring flag        */

/* ═══════════════════════════════════════════════════════════════════════
   Low-level SPI helpers
   ═══════════════════════════════════════════════════════════════════════ */

static inline void cs_low(BME280_Handle *dev)
{
    HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_RESET);
}

static inline void cs_high(BME280_Handle *dev)
{
    HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_SET);
}

/**
 * Read one or more consecutive registers.
 * SPI protocol: send (reg | 0x80) then clock in 'len' bytes.
 */
static BME280_Status read_regs(BME280_Handle *dev,
                                uint8_t reg,
                                uint8_t *buf,
                                uint16_t len)
{
    uint8_t addr = reg | 0x80;
    HAL_StatusTypeDef rc;

    cs_low(dev);
    rc  = HAL_SPI_Transmit(dev->hspi, &addr, 1, BME280_TIMEOUT_MS);
    rc |= HAL_SPI_Receive (dev->hspi, buf,   len, BME280_TIMEOUT_MS);
    cs_high(dev);

    return (rc == HAL_OK) ? BME280_OK : BME280_ERR_SPI;
}

/**
 * Write a single register.
 * SPI protocol: send (reg & 0x7F) followed by the value byte.
 */
static BME280_Status write_reg(BME280_Handle *dev, uint8_t reg, uint8_t value)
{
    uint8_t tx[2] = { reg & 0x7F, value };
    HAL_StatusTypeDef rc;

    cs_low(dev);
    rc = HAL_SPI_Transmit(dev->hspi, tx, 2, BME280_TIMEOUT_MS);
    cs_high(dev);

    return (rc == HAL_OK) ? BME280_OK : BME280_ERR_SPI;
}

/* ═══════════════════════════════════════════════════════════════════════
   Calibration parsing
   ═══════════════════════════════════════════════════════════════════════ */

/**
 * The BME280 stores calibration in two separate NVM banks:
 *   Bank A  0x88-0xA1  (26 bytes, covers T1-T3, P1-P9, H1)
 *   Bank B  0xE1-0xE7  (7 bytes,  covers H2-H6)
 */
static BME280_Status load_calibration(BME280_Handle *dev)
{
    uint8_t buf[26];
    BME280_Status rc;

    /* ── Bank A ─────────────────────────────────────────── */
    rc = read_regs(dev, BME280_REG_CALIB00, buf, 26);
    if (rc != BME280_OK) return rc;

    BME280_Calib *c = &dev->calib;

    c->dig_T1 = (uint16_t)(buf[1]  << 8 | buf[0]);
    c->dig_T2 = (int16_t) (buf[3]  << 8 | buf[2]);
    c->dig_T3 = (int16_t) (buf[5]  << 8 | buf[4]);

    c->dig_P1 = (uint16_t)(buf[7]  << 8 | buf[6]);
    c->dig_P2 = (int16_t) (buf[9]  << 8 | buf[8]);
    c->dig_P3 = (int16_t) (buf[11] << 8 | buf[10]);
    c->dig_P4 = (int16_t) (buf[13] << 8 | buf[12]);
    c->dig_P5 = (int16_t) (buf[15] << 8 | buf[14]);
    c->dig_P6 = (int16_t) (buf[17] << 8 | buf[16]);
    c->dig_P7 = (int16_t) (buf[19] << 8 | buf[18]);
    c->dig_P8 = (int16_t) (buf[21] << 8 | buf[20]);
    c->dig_P9 = (int16_t) (buf[23] << 8 | buf[22]);

    c->dig_H1 = buf[25];    /* 0xA1 is at index 25 in this 26-byte read   */

    /* ── Bank B ─────────────────────────────────────────── */
    rc = read_regs(dev, BME280_REG_CALIB26, buf, 7);
    if (rc != BME280_OK) return rc;

    c->dig_H2 = (int16_t)(buf[1] << 8 | buf[0]);
    c->dig_H3 = buf[2];

    /* H4 and H5 share a nibble in buf[4] — see datasheet §4.2.2 */
    c->dig_H4 = (int16_t)((int8_t)buf[3] << 4 | (buf[4] & 0x0F));
    c->dig_H5 = (int16_t)((int8_t)buf[5] << 4 | (buf[4] >> 4));
    c->dig_H6 = (int8_t)buf[6];

    return BME280_OK;
}

/* ═══════════════════════════════════════════════════════════════════════
   Compensation formulas  (Bosch datasheet §8.2 — double precision)
   t_fine is computed once by compensate_temp and reused by pressure/humidity.
   ═══════════════════════════════════════════════════════════════════════ */

static float compensate_temp(BME280_Handle *dev, int32_t adc_T)
{
    BME280_Calib *c = &dev->calib;
    double var1, var2;

    var1 = ((double)adc_T / 16384.0 - (double)c->dig_T1 / 1024.0)
           * (double)c->dig_T2;
    var2 = ((double)adc_T / 131072.0 - (double)c->dig_T1 / 8192.0)
         * ((double)adc_T / 131072.0 - (double)c->dig_T1 / 8192.0)
         * (double)c->dig_T3;

    dev->t_fine = (int32_t)(var1 + var2);
    return (float)((var1 + var2) / 5120.0);
}

static float compensate_pressure(BME280_Handle *dev, int32_t adc_P)
{
    BME280_Calib *c = &dev->calib;
    double var1, var2, p;

    var1 = (double)dev->t_fine / 2.0 - 64000.0;
    var2 = var1 * var1 * (double)c->dig_P6 / 32768.0;
    var2 = var2 + var1 * (double)c->dig_P5 * 2.0;
    var2 = var2 / 4.0 + (double)c->dig_P4 * 65536.0;
    var1 = ((double)c->dig_P3 * var1 * var1 / 524288.0
            + (double)c->dig_P2 * var1) / 524288.0;
    var1 = (1.0 + var1 / 32768.0) * (double)c->dig_P1;

    if (var1 == 0.0) return 0.0f;   /* avoid division by zero */

    p    = 1048576.0 - (double)adc_P;
    p    = (p - var2 / 4096.0) * 6250.0 / var1;
    var1 = (double)c->dig_P9 * p * p / 2147483648.0;
    var2 = p * (double)c->dig_P8 / 32768.0;
    p    = p + (var1 + var2 + (double)c->dig_P7) / 16.0;

    return (float)(p / 100.0);     /* Pa → hPa */
}

static float compensate_humidity(BME280_Handle *dev, int32_t adc_H)
{
    BME280_Calib *c = &dev->calib;
    double h;

    h  = (double)dev->t_fine - 76800.0;
    h  = (adc_H - ((double)c->dig_H4 * 64.0
           + (double)c->dig_H5 / 16384.0 * h))
       * ((double)c->dig_H2 / 65536.0
           * (1.0 + (double)c->dig_H6 / 67108864.0 * h
               * (1.0 + (double)c->dig_H3 / 67108864.0 * h)));
    h  = h * (1.0 - (double)c->dig_H1 * h / 524288.0);

    if (h > 100.0) h = 100.0;
    if (h <   0.0) h =   0.0;

    return (float)h;
}

/* ═══════════════════════════════════════════════════════════════════════
   Raw burst read  (0xF7-0xFC, 6 bytes)
   ═══════════════════════════════════════════════════════════════════════ */

static BME280_Status read_raw(BME280_Handle *dev,
                               int32_t *adc_T,
                               int32_t *adc_P,
                               int32_t *adc_H)
{
    uint8_t raw[8];
    BME280_Status rc;

    /* Wait until chip is not in a measuring cycle */
    uint32_t deadline = HAL_GetTick() + BME280_TIMEOUT_MS;
    uint8_t  status;
    do {
        rc = read_regs(dev, BME280_REG_STATUS, &status, 1);
        if (rc != BME280_OK) return rc;
        if (HAL_GetTick() > deadline) return BME280_ERR_TIMEOUT;
        HAL_Delay(1);
    } while (status & 0x08);   /* bit3: measuring */

    /* Burst read pressure + temperature + humidity (0xF7-0xFE) */
    rc = read_regs(dev, BME280_REG_DATA, raw, 8);
    if (rc != BME280_OK) return rc;

    /* Assemble 20-bit ADC values (datasheet §4.1) */
    *adc_P = (int32_t)(((uint32_t)raw[0] << 12)
                      | ((uint32_t)raw[1] <<  4)
                      | ((uint32_t)raw[2] >>  4));

    *adc_T = (int32_t)(((uint32_t)raw[3] << 12)
                      | ((uint32_t)raw[4] <<  4)
                      | ((uint32_t)raw[5] >>  4));

    /* 16-bit humidity ADC */
    *adc_H = (int32_t)(((uint32_t)raw[6] << 8)
                      |  (uint32_t)raw[7]);

    return BME280_OK;
}

/* ═══════════════════════════════════════════════════════════════════════
   Public API
   ═══════════════════════════════════════════════════════════════════════ */

BME280_Status BME280_Init(BME280_Handle *dev)
{
    BME280_Status rc;
    uint8_t id;

    cs_high(dev);           /* ensure CS deasserted before anything else   */
    HAL_Delay(10);

    /* Soft reset */
    rc = write_reg(dev, BME280_REG_RESET, BME280_SOFT_RESET_CMD);
    if (rc != BME280_OK) return rc;
    HAL_Delay(10);          /* datasheet: startup time ≤ 2 ms              */

    /* Verify chip ID */
    rc = read_regs(dev, BME280_REG_CHIP_ID, &id, 1);
    if (rc != BME280_OK) return rc;
    if (id != BME280_CHIP_ID) return BME280_ERR_ID;

    /* Load NVM calibration */
    rc = load_calibration(dev);
    if (rc != BME280_OK) return rc;

    /*
     * Configuration sequence (datasheet §5.4.5):
     *  1. Write ctrl_hum  FIRST — only takes effect after ctrl_meas write.
     *  2. Write config    (standby + filter) while in sleep mode.
     *  3. Write ctrl_meas (oversampling + mode=normal) LAST.
     */

    /* 1. Humidity oversampling */
    rc = write_reg(dev, BME280_REG_CTRL_HUM, (uint8_t)dev->cfg.os_hum);
    if (rc != BME280_OK) return rc;

    /* 2. config: t_sb[7:5] | filter[4:2] | spi3w_en[0] */
    uint8_t config = (uint8_t)((dev->cfg.standby << 5)
                               | (dev->cfg.filter << 2));
    rc = write_reg(dev, BME280_REG_CONFIG, config);
    if (rc != BME280_OK) return rc;

    /* 3. ctrl_meas: osrs_t[7:5] | osrs_p[4:2] | mode[1:0]=11 (normal) */
    uint8_t ctrl_meas = (uint8_t)((dev->cfg.os_temp  << 5)
                                  | (dev->cfg.os_press << 2)
                                  | 0x03);              /* normal mode      */
    rc = write_reg(dev, BME280_REG_CTRL_MEAS, ctrl_meas);
    if (rc != BME280_OK) return rc;

    HAL_Delay(BME280_MEAS_DELAY);   /* let first measurement complete      */

    return BME280_OK;
}

BME280_Status BME280_Read(BME280_Handle *dev, BME280_Data *out)
{
    int32_t adc_T, adc_P, adc_H;

    BME280_Status rc = read_raw(dev, &adc_T, &adc_P, &adc_H);
    if (rc != BME280_OK) return rc;

    /* Temperature must be compensated first — it sets t_fine */
    out->temperature = compensate_temp    (dev, adc_T);
    out->pressure    = compensate_pressure(dev, adc_P);
    out->humidity    = compensate_humidity(dev, adc_H);

    return BME280_OK;
}