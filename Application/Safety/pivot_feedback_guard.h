/* Original speed-only guard. Timing is a provisional chassis policy, NOT an
 * electrical/thermal rating. It does not create commands or change feedback. */
#ifndef PIVOT_FEEDBACK_GUARD_H_
#define PIVOT_FEEDBACK_GUARD_H_
#include <stdbool.h>
#include <stdint.h>
#include <math.h>
#define PIVOT_GUARD_GRACE_MS 1000U
#define PIVOT_GUARD_WARN_MS 250U
#define PIVOT_GUARD_STOP_MS 2000U
#define PIVOT_GUARD_RECOVER_MS 100U
typedef struct
{
  uint32_t started_ms, low_since_ms, good_since_ms;
  int8_t sign;
  bool active, low, good, warning, failed;
} PivotFeedbackGuard;

static inline bool PivotFeedbackGuard_Update(PivotFeedbackGuard *s,
  bool owned,float target,float speed,uint32_t now)
{
  if (!owned || !isfinite(target) || !isfinite(speed) || fabsf(target)<20.0f)
  { *s=(PivotFeedbackGuard){0}; return false; }
  const int8_t sign=target>0?1:-1;
  if (!s->active || s->sign!=sign)
    *s=(PivotFeedbackGuard){.active=true,.sign=sign,.started_ms=now};
  if (s->failed) return true;
  if ((uint32_t)(now-s->started_ms)<PIVOT_GUARD_GRACE_MS) return false;
  const float magnitude=fabsf(target), directed=speed*sign;
  /* Cover near-rest even at low targets, as well as severe tracking loss.
   * At 240 cps: low<=36, confirmed recovery>=60 cps. */
  const float low=fmaxf(fminf(20.0f,.25f*magnitude),.15f*magnitude);
  const float recovered=fmaxf(low+1.0f,.25f*magnitude);
  if (!s->low && directed<=low)
  { s->low=true;s->low_since_ms=now; }
  if (s->low)
  {
    if (directed>=recovered)
    {
      if (!s->good) {s->good=true;s->good_since_ms=now;}
      if ((uint32_t)(now-s->good_since_ms)>=PIVOT_GUARD_RECOVER_MS)
      {s->low=false;s->good=false;s->warning=false;return false;}
    }
    else s->good=false;
    const uint32_t age=(uint32_t)(now-s->low_since_ms);
    s->warning=age>=PIVOT_GUARD_WARN_MS;
    if (age>=PIVOT_GUARD_STOP_MS) s->failed=true;
  }
  return s->failed;
}
#endif
