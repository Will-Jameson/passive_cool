#include "servo.h"
#include "pid.h"
#include <stdio.h>
// Store the timer handle passed in at init
static TIM_HandleTypeDef *servo_htim;
// ─────────────────────────────────────────────
// CUBEMX TIMER SETUP REMINDER
// In CubeMX, configure TIM2 as follows:
// - Prescaler: (SystemCoreClock / 1000000) - 1 → gives 1MHz tick (1us per count)
// - Counter Period (ARR): 19999 → 20ms period = 50Hz
// - Channel 1: PWM Generation CH1
// ─────────────────────────────────────────────
static int32_t servo_current_q8 = 0;
static int32_t servo_target_q8  = 0;
static uint8_t servo_damping_active = 0;

void servo_init(TIM_HandleTypeDef *htim) {
    servo_htim = htim;
    HAL_TIM_PWM_Start(servo_htim, SERVO_CHANNEL);
    servo_current_q8 = (int32_t)90 << 8;
    servo_set_angle(90);
    printf("Servo initialized on timer %p\r\n", (void*)servo_htim);
}

void servo_set_angle(uint8_t angle) {
// Clamp angle to valid range
if (angle > 180) angle = 180;

// Map angle (0-180) to pulse width (1000us-2000us)
// Linear interpolation:
// pulse = min + (angle / 180.0) * (max - min)
// Done with integer arithmetic to avoid floating point:
// pulse = min + (angle * (max - min)) / 180
uint32_t pulse = SERVO_MIN_PULSE_US +
((uint32_t)angle * (SERVO_MAX_PULSE_US - SERVO_MIN_PULSE_US)) / 180;
// Write the pulse width to the compare register
__HAL_TIM_SET_COMPARE(servo_htim, SERVO_CHANNEL, pulse);
}
// Fixed-point Q8: values are scaled by 256 to avoid floats
// current_angle_q8 = actual_angle * 256


void servo_set_angle_damped(uint8_t target_angle) {
    if (target_angle > 180) target_angle = 180;
    servo_target_q8 = (int32_t)target_angle << 8;  // convert to Q8
    servo_damping_active = 1;
}

// Call this at a fixed rate — 20ms (50Hz) matches the servo PWM period,
// or use a slower rate (50–100ms) for a lazier, more mechanical feel.
void servo_damp_tick(void) {
    if (!servo_damping_active) return;

    int32_t error = servo_target_q8 - servo_current_q8;

    // Stop updating once close enough to avoid buzzing/jitter
    if (error < 0) error = -error;
    if (error <= (SERVO_DAMP_THRESHOLD << 8)) {
        // Snap to exact target and stop
        servo_current_q8 = servo_target_q8;
        servo_set_angle((uint8_t)(servo_current_q8 >> 8));
        servo_damping_active = 0;
        return;
    }

    // current += alpha * (target - current)
    // In Q8: current_q8 += (ALPHA_Q8 * error) >> 8
    int32_t step = (SERVO_DAMP_ALPHA_Q8 * (servo_target_q8 - servo_current_q8)) >> 8;
    servo_current_q8 += step;

    servo_set_angle((uint8_t)(servo_current_q8 >> 8));
}