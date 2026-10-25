#ifndef CONTROL_RUN_METRICS_H_
#define CONTROL_RUN_METRICS_H_
#include <math.h>
#include <stdint.h>
#include <stdbool.h>

/* Entire active run, independent of the short diagnostic trace ring.
 * sums are in counts/s and (counts/s)^2; they are not wire-format fields. */
typedef struct {
  uint32_t samples, pwm_limited_samples, slew_limited_samples, actuator_limited_samples;
  float squared_error_sum, absolute_error_sum, peak_absolute_error, peak_integral;
  uint32_t zero_net_count_samples, zero_net_count_ms, longest_zero_net_count_ms;
} WheelRunMetrics;
typedef struct {
  uint32_t generation, started_ms, last_sample_ms, samples;
  WheelRunMetrics wheel[4];
} ChassisRunMetrics;

static inline void ControlRunMetrics_Add(WheelRunMetrics *s,float error,float integral,
  uint32_t reasons,bool zero_net_count,uint32_t sample_dt)
{
  if (s->samples==UINT32_MAX) return;
  ++s->samples;
  const float magnitude=fabsf(error);
  s->squared_error_sum+=error*error;
  s->absolute_error_sum+=magnitude;
  s->peak_absolute_error=fmaxf(s->peak_absolute_error,magnitude);
  s->peak_integral=fmaxf(s->peak_integral,fabsf(integral));
  if (reasons&2U) ++s->pwm_limited_samples;
  if (reasons&4U) ++s->slew_limited_samples;
  if (reasons&16U) ++s->actuator_limited_samples;
  if (zero_net_count) {
    ++s->zero_net_count_samples;
    if (UINT32_MAX-s->zero_net_count_ms<sample_dt) s->zero_net_count_ms=UINT32_MAX;
    else s->zero_net_count_ms+=sample_dt;
    if (s->zero_net_count_ms>s->longest_zero_net_count_ms)
      s->longest_zero_net_count_ms=s->zero_net_count_ms;
  } else s->zero_net_count_ms=0;
}
#endif
