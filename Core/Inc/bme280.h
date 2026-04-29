#ifndef BME280_H
#define BME280_H

#include "stm32f3xx_hal.h"
#include <stdint.h>

/* ── Device identity ───────────────────────────────────────────────────── */
#define BME280_CHIP_ID          0x60

/* ── Register map ──────────────────────────────────────────────────────── */
#define BME280_REG_CALIB00      0x88    /* trim T1..H1  (0x88-0x9F + 0xA1) */
#define BME280_REG_CALIB26      0xE1    /* trim H2..H6  (0xE1-0xE7)        */
#define BME280_REG_CHIP_ID      0xD0
#define BME280_REG_RESET        0xE0
#define BME280_REG_CTRL_HUM     0xF2
#define BME280_REG_STATUS       0xF3
#define BME280_REG_CTRL_MEAS    0xF4
#define BME280_REG_CONFIG       0xF5
#define BME280_REG_DATA         0xF7    /* 0xF7-0xFC: press, temp, hum raw  */

/* ── Reset ─────────────────────────────────────────────────────────────── */
#define BME280_SOFT_RESET_CMD   0xB6

/* ── Oversampling ──────────────────────────────────────────────────────── */
typedef enum {
    BME280_OS_SKIPPED = 0x00,
    BME280_OS_1X      = 0x01,
    BME280_OS_2X      = 0x02,
    BME280_OS_4X      = 0x03,
    BME280_OS_8X      = 0x04,
    BME280_OS_16X     = 0x05
} BME280_Oversampling;

/* ── Standby time (normal mode) ────────────────────────────────────────── */
typedef enum {
    BME280_STANDBY_0_5MS  = 0x00,
    BME280_STANDBY_62_5MS = 0x01,
    BME280_STANDBY_125MS  = 0x02,
    BME280_STANDBY_250MS  = 0x03,
    BME280_STANDBY_500MS  = 0x04,
    BME280_STANDBY_1000MS = 0x05,
    BME280_STANDBY_10MS   = 0x06,
    BME280_STANDBY_20MS   = 0x07
} BME280_Standby;

/* ── IIR filter coefficient ────────────────────────────────────────────── */
typedef enum {
    BME280_FILTER_OFF = 0x00,
    BME280_FILTER_2   = 0x01,
    BME280_FILTER_4   = 0x02,
    BME280_FILTER_8   = 0x03,
    BME280_FILTER_16  = 0x04
} BME280_Filter;

/* ── Compensation trim data (loaded from NVM) ──────────────────────────── */
typedef struct {
    uint16_t dig_T1;
    int16_t  dig_T2;
    int16_t  dig_T3;

    uint16_t dig_P1;
    int16_t  dig_P2;
    int16_t  dig_P3;
    int16_t  dig_P4;
    int16_t  dig_P5;
    int16_t  dig_P6;
    int16_t  dig_P7;
    int16_t  dig_P8;
    int16_t  dig_P9;

    uint8_t  dig_H1;
    int16_t  dig_H2;
    uint8_t  dig_H3;
    int16_t  dig_H4;
    int16_t  dig_H5;
    int8_t   dig_H6;
} BME280_Calib;

/* ── User configuration ────────────────────────────────────────────────── */
typedef struct {
    BME280_Oversampling os_temp;
    BME280_Oversampling os_press;
    BME280_Oversampling os_hum;
    BME280_Standby      standby;
    BME280_Filter       filter;
} BME280_Config;

/* ── Main device handle ────────────────────────────────────────────────── */
typedef struct {
    SPI_HandleTypeDef  *hspi;
    GPIO_TypeDef       *cs_port;
    uint16_t            cs_pin;
    BME280_Calib        calib;
    BME280_Config       cfg;
    int32_t             t_fine;     /* shared between compensation formulas */
} BME280_Handle;

/* ── Compensated output ────────────────────────────────────────────────── */
typedef struct {
    float temperature;  /* °C  */
    float pressure;     /* hPa */
    float humidity;     /* %RH */
} BME280_Data;

/* ── Return codes ──────────────────────────────────────────────────────── */
typedef enum {
    BME280_OK            =  0,
    BME280_ERR_SPI       = -1,
    BME280_ERR_ID        = -2,
    BME280_ERR_TIMEOUT   = -3
} BME280_Status;

/* ── Public API ────────────────────────────────────────────────────────── */

/**
 * @brief  Initialise the BME280: reset, verify chip ID, read calibration,
 *         apply config, and start normal mode.
 * @param  dev     Pointer to a BME280_Handle; caller must set hspi, cs_port,
 *                 cs_pin, and cfg before calling.
 * @retval BME280_OK or error code
 */
BME280_Status BME280_Init(BME280_Handle *dev);

/**
 * @brief  Read all three channels and return compensated float values.
 *         Call at any time after Init; driver waits for the chip to be
 *         out of measuring state before reading.
 * @param  dev     Initialised BME280_Handle
 * @param  out     Pointer to BME280_Data to fill
 * @retval BME280_OK or error code
 */
BME280_Status BME280_Read(BME280_Handle *dev, BME280_Data *out);

#endif /* BME280_H */