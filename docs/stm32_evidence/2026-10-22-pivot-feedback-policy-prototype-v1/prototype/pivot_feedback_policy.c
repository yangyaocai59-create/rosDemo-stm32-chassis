#include "pivot_feedback_policy.h"
#include <math.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>

void Pfp_Init(PfpState *s)
{
  if (s != NULL) { memset(s, 0, sizeof(*s)); }
}

bool Pfp_IsFault(PfpResult result)
{
  return result == PFP_FAULT_NO_PROGRESS || result == PFP_FAULT_SEVERE ||
         result == PFP_FAULT_EVIDENCE;
}

static PfpResult Fail(PfpState *s, PfpResult result)
{
  s->result = result;
  return result;
}

PfpResult Pfp_Update(PfpState *s, const PfpInput *in)
{
  if (s == NULL || in == NULL) { return PFP_FAULT_EVIDENCE; }
  /* A fault is sticky: an idle sample is not authorization to clear it. */
  if (Pfp_IsFault(s->result)) { return s->result; }
  if (!isfinite(in->target_cps) || !isfinite(in->measured_cps))
  { return Fail(s, PFP_FAULT_EVIDENCE); }
  if (!in->active || fabsf(in->target_cps) < 200.0f)
  {
    Pfp_Init(s);
    s->result = PFP_NOT_SUPERVISED;
    return s->result; /* Needs existing lower-target safety, NOT permission. */
  }
  if (!in->sample_valid || in->sample_period_ms == 0U ||
      in->sample_period_ms > 100U || abs(in->applied_pwm) > 300)
  { return Fail(s, PFP_FAULT_EVIDENCE); }
  /* A producer may publish <=20ms newer than the caller's sampled tick.
   * Do not reinterpret that as an unsigned, multi-week data timeout. */
  const int32_t age = (int32_t)(in->now_ms - in->sample_ms);
  if (age > (int32_t)PFP_MAX_AGE_MS || age < -20)
  { return Fail(s, PFP_FAULT_EVIDENCE); }
  const int8_t sign = in->target_cps > 0.0f ? 1 : -1;
  if (!s->started || s->sign != sign)
  {
    Pfp_Init(s);
    s->started = true;
    s->sign = sign;
    s->started_ms = in->now_ms;
    s->last_progress_ms = in->now_ms;
  }
  bool fresh = !s->have_sample;
  bool count_progress = sign > 0 ? in->delta > 0 : in->delta < 0;
  if (s->have_sample)
  {
    const uint32_t sample_step = in->sample_ms - s->sample_ms;
    if (sample_step > 0x7fffffffU)
    { return Fail(s, PFP_FAULT_EVIDENCE); }
    if (sample_step == 0U)
    {
      if (in->count != s->count || in->delta != s->delta ||
          in->sample_period_ms != s->sample_period_ms)
      { return Fail(s, PFP_FAULT_EVIDENCE); }
    }
    else
    {
      if (sample_step < in->sample_period_ms || sample_step > 100U)
      { return Fail(s, PFP_FAULT_EVIDENCE); }
      /* Check the signed delta against the position snapshot without an
       * overflowing int64 subtraction. This is consistency, not edge truth. */
      if (sample_step == in->sample_period_ms &&
          ((in->delta > 0 && s->count > INT64_MAX - in->delta) ||
          (in->delta < 0 && s->count < INT64_MIN - in->delta) ||
          in->count != s->count + in->delta))
      { return Fail(s, PFP_FAULT_EVIDENCE); }
      /* A consumer can skip a fresh producer sample. The producer's delta
       * then covers only its last interval, not the whole consumer gap.
       * Use cumulative net change without subtracting potentially extreme
       * int64 positions. Do not invent missing interval edge evidence. */
      count_progress = sign > 0 ? in->count > s->count : in->count < s->count;
      fresh = true;
    }
  }
  if (fresh)
  {
    s->have_sample = true;
    s->sample_ms = in->sample_ms;
    s->sample_period_ms = in->sample_period_ms;
    s->count = in->count;
    s->delta = in->delta;
  }
  const bool forward_progress = fresh && count_progress;
  if (forward_progress)
  {
    /* A duplicate sample must never refresh progress. Slightly future source
     * times are clamped to now, rather than extending the deadline. */
    s->last_progress_ms = age < 0 ? in->now_ms : in->sample_ms;
  }
  if (in->now_ms - s->started_ms < PFP_STARTUP_MS)
  {
    s->result = PFP_NORMAL;
    return s->result;
  }
  if (!s->watching)
  {
    s->watching = true;
    s->last_progress_ms = in->now_ms;
  }
  /* Preserve a finite deadline even if controller support fades or PWM=0:
   * a nonzero active target is still supposed to have credible feedback.
   * This never creates, changes or increases drive output. */
  if (in->now_ms - s->last_progress_ms >= PFP_NO_PROGRESS_MS)
  { return Fail(s, PFP_FAULT_NO_PROGRESS); }
  const float speed = in->measured_cps * (float)sign;
  const bool low = speed < fabsf(in->target_cps) * 0.15f;
  if (low)
  {
    if (!s->severe) { s->severe = true; s->severe_since_ms = in->now_ms; }
    s->recovering = false;
  }
  else if (s->severe)
  {
    const bool recently_progressed = in->now_ms - s->last_progress_ms <= 100U;
    if (!recently_progressed) { s->recovering = false; }
    else if (!s->recovering)
    { s->recovering = true; s->recovering_since_ms = in->now_ms; }
    else if (in->now_ms - s->recovering_since_ms >= PFP_RECOVER_MS)
    { s->severe = false; s->recovering = false; }
  }
  if (s->severe)
  {
    const uint32_t duration = in->now_ms - s->severe_since_ms;
    /* Sparse noise/chatter can refresh last_progress, but cannot prolong a
     * continuous severe episode indefinitely. A single speed spike cannot
     * reset this timer; healthy evidence must persist for 100ms. */
    if (duration >= PFP_SEVERE_MS)
    { return Fail(s, PFP_FAULT_SEVERE); }
    s->result = duration >= PFP_WARNING_MS ? PFP_WARNING : PFP_NORMAL;
  }
  else { s->result = PFP_NORMAL; }
  return s->result;
}
