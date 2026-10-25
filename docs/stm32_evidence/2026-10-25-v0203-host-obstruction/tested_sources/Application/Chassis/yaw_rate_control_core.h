#ifndef YAW_RATE_CONTROL_CORE_H_
#define YAW_RATE_CONTROL_CORE_H_

#include <stdbool.h>
#include <stdint.h>
#include <math.h>

/* Conservative candidate gains; keep wheel PI and electrical limits intact.
 * All rates are rad/s, elapsed time is ms. No HAL, RTOS, heap or hardware. */
#define YAW_RATE_KP 0.35f
#define YAW_RATE_KI 0.35f
#define YAW_RATE_FILTER_SECONDS 0.12f
#define YAW_RATE_CORRECTION_MAX 0.10f
#define YAW_RATE_CORRECTION_FRACTION 0.25f
#define YAW_RATE_CORRECTION_SLEW 0.08f
#define YAW_RATE_SETTLE_MS 600U

typedef struct {
  float filtered_rate, integral, correction, reference;
  uint32_t warm_ms;
  bool active;
} YawRateControlState;

static inline float YawRate_Clamp(float x,float lo,float hi)
{ return fminf(hi,fmaxf(lo,x)); }

static inline void YawRateControl_Reset(YawRateControlState *s)
{ if (s) *s=(YawRateControlState){0}; }

static inline float YawRateControl_Step(YawRateControlState *s,
  float linear,float reference,float measured,bool feedback_valid,
  bool actuator_limited,uint32_t elapsed_ms)
{
  if (!s) return 0.0f;
  if (!isfinite(linear) || !isfinite(reference) || elapsed_ms==0U || elapsed_ms>100U ||
      fabsf(linear)>0.001f || fabsf(reference)<0.12f || fabsf(reference)>0.60f ||
      !isfinite(s->correction) || !isfinite(s->integral) || !isfinite(s->filtered_rate)) {
    YawRateControl_Reset(s); return 0.0f;
  }
  const float dt=(float)elapsed_ms*.001f;
  const float limit=fminf(YAW_RATE_CORRECTION_MAX,
                         YAW_RATE_CORRECTION_FRACTION*fabsf(reference));
  const float step=YAW_RATE_CORRECTION_SLEW*dt;
  s->correction=YawRate_Clamp(s->correction,-limit,limit);
  s->integral=YawRate_Clamp(s->integral,-limit,limit);
  if (!feedback_valid || !isfinite(measured) || fabsf(measured)>3.0f) {
    s->integral=0.0f; s->active=false; s->warm_ms=0U;
    s->correction+=YawRate_Clamp(-s->correction,-step,step);
    s->reference=reference;
    return s->correction;
  }
  if (!s->active || s->reference*reference<0.0f) {
    s->filtered_rate=measured; s->integral=0.0f; s->warm_ms=0U;
    /* A direction reversal must never inherit the previous correction. */
    if (s->reference*reference<0.0f) s->correction=0.0f;
    s->active=true;
  }
  s->reference=reference;
  s->filtered_rate+=dt/(YAW_RATE_FILTER_SECONDS+dt)*(measured-s->filtered_rate);
  if (s->warm_ms<YAW_RATE_SETTLE_MS) {
    s->warm_ms+=elapsed_ms;
    s->correction+=YawRate_Clamp(-s->correction,-step,step);
    return s->correction;
  }
  const float error=reference-s->filtered_rate;
  float trial=YawRate_Clamp(s->integral+YAW_RATE_KI*error*dt,-limit,limit);
  float requested=YawRate_Clamp(YAW_RATE_KP*error+trial,-limit,limit);
  float applied=s->correction+YawRate_Clamp(requested-s->correction,-step,step);
  if ((error*(YAW_RATE_KP*error+trial-requested)>0.0f) ||
      (error*(requested-applied)>0.0f) ||
      (actuator_limited && error*trial>0.0f)) {
    trial=s->integral;
    requested=YawRate_Clamp(YAW_RATE_KP*error+trial,-limit,limit);
    applied=s->correction+YawRate_Clamp(requested-s->correction,-step,step);
  }
  s->integral=trial;
  s->correction=YawRate_Clamp(applied,-limit,limit);
  return s->correction;
}
#endif
