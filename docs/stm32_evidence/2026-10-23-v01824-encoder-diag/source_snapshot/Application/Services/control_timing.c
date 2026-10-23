#include "control_timing.h"
#include <stddef.h>
#include "FreeRTOS.h"
#include "task.h"

static ControlTimingStatus s_stats[CONTROL_TIMING_COUNT];
static uint32_t s_previous[CONTROL_TIMING_COUNT];

void ControlTiming_RecordStart(ControlTimingTask task, uint32_t now_tick)
{
  if ((uint32_t)task >= CONTROL_TIMING_COUNT) { return; }
  taskENTER_CRITICAL();
  ControlTimingStatus *s = &s_stats[task];
  if (s->iterations != 0U)
  {
    const uint32_t dt = now_tick - s_previous[task];
    if ((s->iterations == 1U) || (dt < s->period_min_ms))
    { s->period_min_ms = dt; }
    if (dt > s->period_max_ms) { s->period_max_ms = dt; }
  }
  s_previous[task] = now_tick;
  ++s->iterations;
  taskEXIT_CRITICAL();
}

uint32_t ControlTiming_NextWake(ControlTimingTask task,
  uint32_t previous_deadline, uint32_t now_tick, uint32_t period_ms)
{
  if ((uint32_t)task >= CONTROL_TIMING_COUNT || period_ms == 0U)
  { return now_tick + 1U; }
  uint32_t next = previous_deadline + period_ms;
  if ((int32_t)(now_tick - next) >= 0)
  {
    taskENTER_CRITICAL();
    ++s_stats[task].overrun_count;
    s_stats[task].missed_deadlines += (now_tick - next) / period_ms + 1U;
    taskEXIT_CRITICAL();
    next = now_tick + period_ms;
  }
  return next;
}

bool ControlTiming_GetStatus(ControlTimingTask task, ControlTimingStatus *status)
{
  if ((uint32_t)task >= CONTROL_TIMING_COUNT || status == NULL) { return false; }
  taskENTER_CRITICAL();
  *status = s_stats[task];
  taskEXIT_CRITICAL();
  return true;
}
