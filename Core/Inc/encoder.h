#ifndef ENCODER_H
#define ENCODER_H

#include "stm32f3xx_hal.h"
#include <stdint.h>

// _____________________________________
// PIN CONFIGURATION



// _____________________________________

#define ENCODER_TIMER TIM3  // PLACEHOLDER: Change to the timer you are using for the encoder
#define ENCODER_SW_PORT GPIOA  // PLACEHOLDER: Change to the pin you are using for the encoder switch
#define ENCODER_SW_PIN GPIO_PIN_10  // PLACEHOLDER: Change to the pin you are using for the encoder switch

// _______________________________________
// ENCODER CONFIGURATION
// The KY-040 encoder has 20 detents per revolution.
// In quadrature mode, the timer counts 4 edges per detent, so 80 counts per revolution.

// We will track half those counts since we count edges on  one channel only in TIM_ENCODERMODE_TI1 mode, giving us 40 counts per revolution.
#define ENCODER_MAX_COUNT 40
#define ENCODER_COUNTS_PER_REV 40
#define ANGLE_STEP_PER_COUNT 1.0f
 
// _______________________________________
// MODES
//_________________________________________
typedef enum {
    ENCODER_MODE_DISABLED = 0,
    ENCODER_MODE_ENABLED = 1,
} EncoderMode_t;

// _______________________________________
// PUBLIC API
// _______________________________________

// Pass in the timer handle configured for encodermode_ti1 in CUBEMX
void encoder_init(TIM_HandleTypeDef *htim);

// Function  call for main loop - with debounce and count updates
void encoder_update(void);

// Get angle in degrees (0-180)
float encoder_get_angle(void);

// Get encoder state:  returns ENCODER_MODE_ENABLED if the encoder is active, ENCODER_MODE_DISABLED if not

EncoderMode_t encoder_get_mode(void);

// encoder mode changed:  returns 1 if the mode changed since last call, 0 if not
uint8_t encoder_mode_changed(void);

#endif // ENCODER_H

