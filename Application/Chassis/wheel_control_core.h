/* Original, allocation-free FF+PI kernel. No hardware, RTOS or global state.
 * Units: counts/s, milliseconds, PWM permille. No uncalibrated voltage model. */
#ifndef WHEEL_CONTROL_CORE_H_
#define WHEEL_CONTROL_CORE_H_
#include <math.h>
#include <stdbool.h>
#include <stdint.h>

enum {
  WHEEL_LIMIT_TARGET = 1U, WHEEL_LIMIT_MAGNITUDE = 2U,
  WHEEL_LIMIT_SLEW = 4U, WHEEL_LIMIT_HANDOFF = 8U,
  WHEEL_LIMIT_ACTUATOR = 16U
};

typedef struct {
  float integral, output, target;
  uint8_t mode;
  bool active;
} WheelControlMemory;

typedef struct {
  float kp, ki, integral_limit, output_limit, slew_per_second, target_limit;
  float zero_target;
} WheelControlParameters;

typedef struct {
  WheelControlMemory memory;
  float target, error, feedforward, proportional;
  uint32_t reasons;
} WheelControlResult;

static inline float WheelControl_Clamp(float x, float lo, float hi)
{ return x < lo ? lo : (x > hi ? hi : x); }

/* Only a completed result is returned. Mode changes track the last applied
 * output, bounded by the calibrated integral allowance. The residual handoff
 * is slew-limited; no claim of exact bumplessness beyond that allowance. */
static inline bool WheelControl_Step(const WheelControlMemory *previous,
  const WheelControlParameters *parameters, float target, float measured,
  float feedforward, uint8_t mode, uint32_t dt_ms, WheelControlResult *result)
{
  if (!previous || !parameters || !result || mode > 1U || !dt_ms || dt_ms > 100U ||
      !isfinite(target) || !isfinite(measured) || fabsf(measured) > 1000000.0f ||
      !isfinite(feedforward) || fabsf(feedforward) > 10000.0f ||
      !isfinite(previous->integral) || !isfinite(previous->output) ||
      !isfinite(previous->target) || !isfinite(parameters->kp) ||
      !isfinite(parameters->ki) || !isfinite(parameters->integral_limit) ||
      !isfinite(parameters->output_limit) || !isfinite(parameters->slew_per_second) ||
      !isfinite(parameters->target_limit) || !isfinite(parameters->zero_target) ||
      parameters->kp < 0 || parameters->ki < 0 ||
      parameters->integral_limit <= 0 || parameters->output_limit <= 0 ||
      parameters->slew_per_second <= 0 || parameters->target_limit <= 0 ||
      parameters->zero_target <= 0 ||
      parameters->integral_limit > 10000.0f || parameters->output_limit > 1000.0f ||
      parameters->kp > 1000.0f || parameters->ki > 1000.0f ||
      parameters->slew_per_second > 1000000.0f || parameters->target_limit > 1000000.0f ||
      fabsf(previous->output) > parameters->output_limit)
    return false;

  WheelControlResult next = {0};
  next.memory = *previous;
  next.target = WheelControl_Clamp(target, -parameters->target_limit,
                                  parameters->target_limit);
  if (next.target != target) next.reasons |= WHEEL_LIMIT_TARGET;
  const float step = parameters->slew_per_second * ((float)dt_ms / 1000.0f);
  if (fabsf(next.target) < parameters->zero_target) {
    next.target = 0;
    next.error = -measured;
    next.memory = (WheelControlMemory){0};
    next.memory.output = WheelControl_Clamp(0, previous->output-step,
                                             previous->output+step);
    if (next.memory.output != 0) next.reasons |= WHEEL_LIMIT_SLEW;
    *result = next;
    return true;
  }

  const bool reversed = previous->active && previous->target * next.target < 0;
  const bool handoff = previous->active && previous->mode != mode && !reversed;
  const float limit = parameters->integral_limit;
  const float ceiling = parameters->output_limit;
  next.error = next.target - measured;
  next.feedforward = feedforward;
  next.proportional = parameters->kp * next.error;
  float old_i = previous->active && !reversed ? previous->integral : 0;
  if (handoff) {
    old_i = previous->output - next.feedforward - next.proportional;
    next.reasons |= WHEEL_LIMIT_HANDOFF;
  }
  old_i = WheelControl_Clamp(old_i, -limit, limit);
  float candidate_i = WheelControl_Clamp(old_i + parameters->ki * next.error *
                                       ((float)dt_ms/1000.0f), -limit, limit);
  /* NORMAL retains bipolar output. PIVOT retains coast-only overspeed removal.
   * A direction change may coast through zero via the driver interlock. */
  const float lo = mode == 1U && next.target > 0 ? 0 : -ceiling;
  const float hi = mode == 1U && next.target < 0 ? 0 : ceiling;
  float wanted = next.feedforward + next.proportional + candidate_i;
  float desired = WheelControl_Clamp(wanted, lo, hi);
  float applied = WheelControl_Clamp(desired, previous->output-step,
                                             previous->output+step);
  if ((candidate_i-old_i) * (wanted-applied) > 0) {
    candidate_i = old_i;
    wanted = next.feedforward + next.proportional + candidate_i;
    desired = WheelControl_Clamp(wanted, lo, hi);
    applied = WheelControl_Clamp(desired, previous->output-step,
                                         previous->output+step);
  }
  if (wanted < lo || wanted > hi) next.reasons |= WHEEL_LIMIT_MAGNITUDE;
  if (applied != desired) next.reasons |= WHEEL_LIMIT_SLEW;
  next.memory = (WheelControlMemory){candidate_i, applied, next.target, mode, true};
  *result = next;
  return true;
}
#endif
