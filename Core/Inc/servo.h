#ifndef SERVO_H
#define SERVO_H
#include "stm32f3xx_hal.h" // Change to match your STM32 family
#include <stdint.h>
// ─────────────────────────────────────────────
// TIMER CONFIGURATION — edit to match your setup
// ─────────────────────────────────────────────
#define SERVO_TIMER TIM1
#define SERVO_CHANNEL TIM_CHANNEL_1
// ─────────────────────────────────────────────
// SERVO PULSE CONFIGURATION
// Standard servo: 1ms = full left, 2ms = full right
// At 50Hz (20ms period), timer counts determine pulse width.
// These values assume a timer configured for 1MHz tick (1us per count).
// ─────────────────────────────────────────────
#define SERVO_MIN_PULSE_US 1000 // 1ms = 0 degrees
#define SERVO_MAX_PULSE_US 2000 // 2ms = 180 degrees
#define SERVO_PERIOD_US 20000 // 20ms = 50Hz


// ─────────────────────────────────────────────
// PUBLIC API
// ─────────────────────────────────────────────
void servo_init(TIM_HandleTypeDef *htim);
void servo_set_angle(uint8_t angle); // 0 to 180 degrees

// Damping factor: 0.0 (never moves) to 1.0 (instant, no damping)
// 0.15–0.25 gives a natural feel over ~10–15 ticks

#define SERVO_DAMP_ALPHA_Q8   6 // alpha = 38/256 ≈ 0.15 (Q8 fixed-point)
#define SERVO_DAMP_THRESHOLD  1    // stop updating when |error| <= 1 degree

void servo_set_angle_damped(uint8_t target_angle);
void servo_damp_tick(void);        // call from SysTick or a periodic timer ISR
#endif // SERVO_H