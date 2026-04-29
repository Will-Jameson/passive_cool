#include "pid.h"
// ─────────────────────────────────────────────
// INTERNAL HELPERS
// ─────────────────────────────────────────────
static float clamp(float value, float min, float max) {
if (value < min) return min;
if (value > max) return max;
return value;
}
// ─────────────────────────────────────────────
// PUBLIC IMPLEMENTATION
// ─────────────────────────────────────────────
void pid_init(PID_t *pid, float kp, float ki, float kd,
float integral_min, float integral_max) {
pid->kp = kp;
pid->ki = ki;
pid->kd = kd;
pid->integral = 0.0f;
pid->prev_error = 0.0f;
pid->output = 0.0f;
pid->integral_min = integral_min;
pid->integral_max = integral_max;
}
void pid_reset(PID_t *pid) {
pid->integral = 0.0f;
pid->prev_error = 0.0f;
pid->output = 0.0f;
}
void pid_set_gains(PID_t *pid, float kp, float ki, float kd) {
pid->kp = kp;
pid->ki = ki;
pid->kd = kd;
}
float pid_update(PID_t *pid, float setpoint, float measured, float dt) {
// Guard against zero or nonsensical dt
if (dt <= 0.0f) return pid->output;
// ── Proportional ──────────────────────────
// How far are we from the target right now
float error = setpoint - measured;
float p_term = pid->kp * error;
// ── Integral ──────────────────────────────
// Accumulate error over time to eliminate steady-state offset.
// Anti-windup: clamp the integral before multiplying by ki,
// so it can't accumulate beyond what the output can actually use.
pid->integral += error * dt;
pid->integral = clamp(pid->integral, pid->integral_min, pid->integral_max);
float i_term = pid->ki * pid->integral;
// ── Derivative ────────────────────────────
// React to how fast the error is changing.
// Dividing by dt gives rate of change per second,
// making kd independent of your update frequency.
float derivative = (error - pid->prev_error) / dt;
float d_term = pid->kd * derivative;
// ── Combine & clamp output ────────────────
float output = p_term + i_term + d_term;
pid->output = clamp(output, PID_OUTPUT_MIN, PID_OUTPUT_MAX);
// Store error for next derivative calculation
pid->prev_error = error;
return pid->output;
}