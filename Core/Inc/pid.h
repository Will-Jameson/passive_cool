#ifndef PID_H
#define PID_H
#include <stdint.h>
// ─────────────────────────────────────────────
// OUTPUT LIMITS
// Clamp PID output to valid servo angle range
// ─────────────────────────────────────────────
#define PID_OUTPUT_MIN 0.0f // degrees
#define PID_OUTPUT_MAX 180.0f // degrees
// ─────────────────────────────────────────────
// PID STATE
// One instance per controller. Zero-initialise
// before calling pid_init().
// ─────────────────────────────────────────────
typedef struct {
// Tuning parameters
float kp; // Proportional gain
float ki; // Integral gain
float kd; // Derivative gain
// Internal state
float integral; // Accumulated integral term
float prev_error; // Last error, used for derivative
float output; // Last computed output
// Anti-windup: clamp integral accumulation
float integral_min;
float integral_max;
} PID_t;
// ─────────────────────────────────────────────
// PUBLIC API
// ─────────────────────────────────────────────
// Initialise controller with gains and integral clamp limits
void pid_init(PID_t *pid, float kp, float ki, float kd,
float integral_min, float integral_max);
// Update controller. Call once per fixed time step.
// setpoint — desired angle (degrees)
// measured — current angle from your sensor (degrees)// dt — time since last call (seconds)
// Returns the output angle to pass to servo_set_angle()
float pid_update(PID_t *pid, float setpoint, float measured, float dt);
// Reset integral and derivative state (e.g. on mode change)
void pid_reset(PID_t *pid);
// Retune on the fly without losing integral state
void pid_set_gains(PID_t *pid, float kp, float ki, float kd);
#endif // PID_H
