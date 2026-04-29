/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "spi.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <unistd.h>

#include "tsl2561.h"
#include "encoder.h"
#include "servo.h"
#include "bme280.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
extern TIM_HandleTypeDef htim1; // Servo timer
extern TIM_HandleTypeDef htim3; // Encoder timer

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
BME280_Handle bme1, bme2;
BME280_Data   data1, data2;

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

// /* USER CODE END PV */

// /* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
// /* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/

/* USER CODE BEGIN 0 */
#ifdef __GNUC__

int _write(int file, char *ptr, int len)
{
    HAL_UART_Transmit(&huart2, (uint8_t *)ptr, len, HAL_MAX_DELAY);
    return len;
}
#endif
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  MX_GPIO_Init();
  MX_USART2_UART_Init();
  HAL_Delay(500);
  printf("Before I2C init\r\n");
  MX_I2C1_Init();
  printf("After I2C init\r\n");
  MX_SPI1_Init();

  MX_TIM1_Init();
  MX_TIM3_Init();

  // BME280 setup
bme1.hspi    = &hspi1;
bme1.cs_port = GPIOB;
bme1.cs_pin  = GPIO_PIN_0;

bme1.cfg.os_temp  = BME280_OS_2X;
bme1.cfg.os_press = BME280_OS_4X;
bme1.cfg.os_hum   = BME280_OS_1X;
bme1.cfg.standby  = BME280_STANDBY_500MS;
bme1.cfg.filter   = BME280_FILTER_4;
// Sensor 2 Config - same settings but different CS pin
bme2.hspi    = &hspi1;
bme2.cs_port = GPIOB;
bme2.cs_pin  = GPIO_PIN_1;   // whatever pin you chose for CS2
bme2.cfg.os_temp  = BME280_OS_2X;
bme2.cfg.os_press = BME280_OS_4X;
bme2.cfg.os_hum   = BME280_OS_1X;
bme2.cfg.standby  = BME280_STANDBY_500MS;
bme2.cfg.filter   = BME280_FILTER_4;


printf("BME280 #1 init...\r\n");
if (BME280_Init(&bme1) != BME280_OK) {
    printf("BME280 #1 FAILED\r\n");
    Error_Handler();
}
printf("BME280 #1 OK\r\n");

printf("BME280 #2 init...\r\n");
if (BME280_Init(&bme2) != BME280_OK) {
    printf("BME280 #2 FAILED\r\n");
    Error_Handler();
}
printf("BME280 #2 OK\r\n");
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */

  /* USER CODE BEGIN 2 */
  float max_delta = 0.5f;
  #define MAX_DELTA_MIN  0.5f
  #define MAX_DELTA_MAX  20.0f
  EncoderMode_t mode = ENCODER_MODE_DISABLED;
  int32_t enc = 0;
  uint8_t btn_last = 1;

  encoder_init(&htim3);
  servo_init(&htim1);
  servo_set_angle(90);
   
    HAL_GPIO_TogglePin(usr_LED_GPIO_Port, usr_LED_Pin);
    printf("Setup complete\r\n");
    HAL_StatusTypeDef st = HAL_I2C_IsDeviceReady(&hi2c1, 0x01 << 1, 3, 100);
    printf("TSL2561 @0x49 status = %d\r\n", st);
  

  

        HAL_Delay(5000);
        
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

while (1) {

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    // Read both sensors
    float temp_delta = 0.0f;

    if (BME280_Read(&bme1, &data1) == BME280_OK &&
        BME280_Read(&bme2, &data2) == BME280_OK) {

        temp_delta = data1.temperature - data2.temperature;
        if (temp_delta < 0) temp_delta = -temp_delta;

        printf("S1 T:%.2f  S2 T:%.2f  Delta:%.2f C\r\n",
               data1.temperature, data2.temperature, temp_delta);
    } else {
        printf("BME280 read error\r\n");
    }

    HAL_Delay(500);
    encoder_update();

uint8_t btn_raw = HAL_GPIO_ReadPin(ENCODER_SW_PORT, ENCODER_SW_PIN);


// Trigger on rising edge (release) — more reliable than press edge
if (btn_raw == 1 && btn_last == 0) {
    HAL_Delay(20);   // debounce the release
    if (HAL_GPIO_ReadPin(ENCODER_SW_PORT, ENCODER_SW_PIN) == 1) {
        mode = (mode == ENCODER_MODE_ENABLED)
             ? ENCODER_MODE_DISABLED
             : ENCODER_MODE_ENABLED;
        printf("BTN PRESS -> %s\r\n",
               mode == ENCODER_MODE_ENABLED ? "ENABLED" : "DISABLED");
    }
}
btn_last = btn_raw;

    if (mode == ENCODER_MODE_ENABLED) {
        // Encoder knob maps to max_delta range
        // encoder_get_angle() returns 0-180, rescale to MAX_DELTA_MIN-MAX_DELTA_MAX
        float knob = encoder_get_angle();  // 0.0 - 180.0
        max_delta  = MAX_DELTA_MIN
                   + (knob / 180.0f) * (MAX_DELTA_MAX - MAX_DELTA_MIN);

        // Subtract natural sensor offset so baseline reads as zero deflection
        #define BASELINE_DELTA  1.0f
        float adjusted = temp_delta - BASELINE_DELTA;
        if (adjusted < 0.0f) adjusted = 0.0f;

        float clamped    = adjusted > max_delta ? max_delta : adjusted;
        uint8_t servo_angle = (uint8_t)((clamped / max_delta) * 180.0f);

        servo_set_angle_damped(servo_angle);
        printf("ON  Delta:%.2f  MaxDelta:%.1f  Servo:%d  CNT:%ld  btn=%d\r\n",
               temp_delta, max_delta, servo_angle, enc, btn_raw);
        } else {
          float angle = encoder_get_angle();
          servo_set_angle_damped((uint8_t)angle);
          printf("OFF Delta:%.2f  Angle:%.1f  CNT:%ld  btn=%d\r\n",
          temp_delta, angle, enc, btn_raw);
}
}

   
}

  /* USER CODE END 3 */


/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_I2C1|RCC_PERIPHCLK_TIM1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  PeriphClkInit.Tim1ClockSelection = RCC_TIM1CLK_HCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

// void BME280_ReadCalibration(void)
// {
//     uint8_t calib[24];
//     uint8_t reg = 0x88;
//     HAL_I2C_Master_Transmit(&hi2c1, 0x76 << 1, &reg, 1, 100);
//     HAL_I2C_Master_Receive(&hi2c1, 0x76 << 1, calib, 24, 100);

//     bme280_calib.dig_T1 = ((uint16_t)calib[1] << 8) | calib[0];
//     bme280_calib.dig_T2 = (int16_t)(((uint16_t)calib[3] << 8) | calib[2]);
//     bme280_calib.dig_T3 = (int16_t)(((uint16_t)calib[5] << 8) | calib[4]);
//     bme280_calib.dig_P1 = ((uint16_t)calib[7] << 8) | calib[6];
//     bme280_calib.dig_P2 = (int16_t)(((uint16_t)calib[9] << 8)  | calib[8]);
//     bme280_calib.dig_P3 = (int16_t)(((uint16_t)calib[11] << 8) | calib[10]);
//     bme280_calib.dig_P4 = (int16_t)(((uint16_t)calib[13] << 8) | calib[12]);
//     bme280_calib.dig_P5 = (int16_t)(((uint16_t)calib[15] << 8) | calib[14]);
//     bme280_calib.dig_P6 = (int16_t)(((uint16_t)calib[17] << 8) | calib[16]);
//     bme280_calib.dig_P7 = (int16_t)(((uint16_t)calib[19] << 8) | calib[18]);
//     bme280_calib.dig_P8 = (int16_t)(((uint16_t)calib[21] << 8) | calib[20]);
//     bme280_calib.dig_P9 = (int16_t)(((uint16_t)calib[23] << 8) | calib[22]);

//     reg = 0xA1;
//     HAL_I2C_Master_Transmit(&hi2c1, 0x76 << 1, &reg, 1, 100);
//     HAL_I2C_Master_Receive(&hi2c1, 0x76 << 1, &bme280_calib.dig_H1, 1, 100);

//     uint8_t calib_h[7];
//     reg = 0xE1;
//     HAL_I2C_Master_Transmit(&hi2c1, 0x76 << 1, &reg, 1, 100);
//     HAL_I2C_Master_Receive(&hi2c1, 0x76 << 1, calib_h, 7, 100);

//     bme280_calib.dig_H2 = (int16_t)(((uint16_t)calib_h[1] << 8) | calib_h[0]);
//     bme280_calib.dig_H3 = calib_h[2];
//     bme280_calib.dig_H4 = (int16_t)((calib_h[3] << 4) | (calib_h[4] & 0x0F));
//     bme280_calib.dig_H5 = (int16_t)((calib_h[5] << 4) | (calib_h[4] >> 4));
//     bme280_calib.dig_H6 = (int8_t)calib_h[6];
// }

// void BME280_Init(void)
// {
//     uint8_t data[2];

//     // Humidity oversampling x1 — must be written before ctrl_meas
//     data[0] = 0xF2; data[1] = 0x01;
//     HAL_I2C_Master_Transmit(&hi2c1, 0x76 << 1, data, 2, 100);
//     HAL_Delay(10);

//     // Temp+pressure oversampling x1, normal mode
//     data[0] = 0xF4; data[1] = 0x27;
//     HAL_I2C_Master_Transmit(&hi2c1, 0x76 << 1, data, 2, 100);
//     HAL_Delay(10);

//     // Standby 1000ms, filter off
//     data[0] = 0xF5; data[1] = 0xA0;
//     HAL_I2C_Master_Transmit(&hi2c1, 0x76 << 1, data, 2, 100);
// }

// float BME280_ReadTemperature(void)
// {
//     uint8_t data[3];
//     uint8_t reg = 0xFA;
//     HAL_I2C_Master_Transmit(&hi2c1, 0x76 << 1, &reg, 1, 100);
//     HAL_I2C_Master_Receive(&hi2c1, 0x76 << 1, data, 3, 100);

//     int32_t adc_T = ((int32_t)data[0] << 12) | ((int32_t)data[1] << 4) | (data[2] >> 4);

//     int32_t var1 = ((((adc_T >> 3) - ((int32_t)bme280_calib.dig_T1 << 1))) *
//                    ((int32_t)bme280_calib.dig_T2)) >> 11;
//     int32_t var2 = (((((adc_T >> 4) - ((int32_t)bme280_calib.dig_T1)) *
//                    ((adc_T >> 4) - ((int32_t)bme280_calib.dig_T1))) >> 12) *
//                    ((int32_t)bme280_calib.dig_T3)) >> 14;

//     t_fine = var1 + var2;
//     return (float)((t_fine * 5 + 128) >> 8) / 100.0f;
// }

// float BME280_ReadPressure(void)
// {
//     uint8_t data[3];
//     uint8_t reg = 0xF7;
//     HAL_I2C_Master_Transmit(&hi2c1, 0x76 << 1, &reg, 1, 100);
//     HAL_I2C_Master_Receive(&hi2c1, 0x76 << 1, data, 3, 100);

//     int32_t adc_P = ((int32_t)data[0] << 12) | ((int32_t)data[1] << 4) | (data[2] >> 4);

//     int64_t var1 = ((int64_t)t_fine) - 128000;
//     int64_t var2 = var1 * var1 * (int64_t)bme280_calib.dig_P6;
//     var2 = var2 + ((var1 * (int64_t)bme280_calib.dig_P5) << 17);
//     var2 = var2 + (((int64_t)bme280_calib.dig_P4) << 35);
//     var1 = ((var1 * var1 * (int64_t)bme280_calib.dig_P3) >> 8) +
//            ((var1 * (int64_t)bme280_calib.dig_P2) << 12);
//     var1 = (((((int64_t)1) << 47) + var1)) * ((int64_t)bme280_calib.dig_P1) >> 33;
//     if (var1 == 0) return 0;
//     int64_t p = 1048576 - adc_P;
//     p = (((p << 31) - var2) * 3125) / var1;
//     var1 = (((int64_t)bme280_calib.dig_P9) * (p >> 13) * (p >> 13)) >> 25;
//     var2 = (((int64_t)bme280_calib.dig_P8) * p) >> 19;
//     p = ((p + var1 + var2) >> 8) + (((int64_t)bme280_calib.dig_P7) << 4);
//     return (float)p / 25600.0f;
// }

// float BME280_ReadHumidity(void)
// {
//     uint8_t data[2];
//     uint8_t reg = 0xFD;
//     HAL_I2C_Master_Transmit(&hi2c1, 0x76 << 1, &reg, 1, 100);
//     HAL_I2C_Master_Receive(&hi2c1, 0x76 << 1, data, 2, 100);

//     int32_t adc_H = ((int32_t)data[0] << 8) | data[1];

//     int32_t var1 = t_fine - 76800;
//     var1 = (((((adc_H << 14) - ((int32_t)bme280_calib.dig_H4 << 20) -
//              ((int32_t)bme280_calib.dig_H5 * var1)) + 16384) >> 15) *
//             (((((((var1 * (int32_t)bme280_calib.dig_H6) >> 10) *
//              (((var1 * (int32_t)bme280_calib.dig_H3) >> 11) + 32768)) >> 10) +
//              2097152) * (int32_t)bme280_calib.dig_H2 + 8192) >> 14));
//     var1 = var1 - (((((var1 >> 15) * (var1 >> 15)) >> 7) *
//                   (int32_t)bme280_calib.dig_H1) >> 4);
//     var1 = (var1 < 0) ? 0 : var1;
//     var1 = (var1 > 419430400) ? 419430400 : var1;
//     return (float)(var1 >> 12) / 1024.0f;
// }
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
      printf("Error occurred!\r\n");
    HAL_Delay(1000);
  __disable_irq();
  while (1)
  {}

  
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
