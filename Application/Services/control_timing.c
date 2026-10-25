#include "control_timing.h"
#include <stddef.h>
#include "FreeRTOS.h"
#include "task.h"

static ControlTimingStatus s_stats[CONTROL_TIMING_COUNT];
static uint32_t s_previous[CONTROL_TIMING_COUNT];
static ControlSectionTiming s_sections[CONTROL_SECTION_COUNT];

#if defined(__arm__) || defined(__thumb__)
#include "stm32f4xx.h"
#endif
void ControlTiming_EnableCycleCounter(void)
{
#if defined(__arm__) || defined(__thumb__)
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  /* Do not reset a counter possibly already used by another profiler. */
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
  __DSB();
  __ISB();
#endif
}
uint32_t ControlTiming_ReadCycles(void)
{
#if defined(__arm__) || defined(__thumb__)
  return DWT->CYCCNT;
#else
  return 0U;
#endif
}
static uint32_t CyclesToMicroseconds(uint32_t cycles,uint32_t cpu_hz)
{
  /* Divide outside the publication lock, preserve sub-microsecond rounding. */
  return (uint32_t)(((uint64_t)cycles*1000000U+cpu_hz/2U)/cpu_hz);
}
void ControlTiming_RecordExecution(ControlTimingTask task,uint32_t cycles,uint32_t cpu_hz)
{
  if ((uint32_t)task>=CONTROL_TIMING_COUNT || cpu_hz<1000000U) return;
  const uint32_t us=CyclesToMicroseconds(cycles,cpu_hz);
  taskENTER_CRITICAL();
  ControlTimingStatus *s=&s_stats[task];
  s->execution_last_us=us;
  if (us>s->execution_max_us) s->execution_max_us=us;
  ++s->execution_samples;
  taskEXIT_CRITICAL();
}
void ControlTiming_RecordSection(ControlTimingSection section,uint32_t cycles,uint32_t cpu_hz)
{
  if ((uint32_t)section>=CONTROL_SECTION_COUNT || cpu_hz<1000000U) return;
  const uint32_t us=CyclesToMicroseconds(cycles,cpu_hz);
  taskENTER_CRITICAL();
  ControlSectionTiming *s=&s_sections[section];
  s->last_us=us;
  if (us>s->max_us) s->max_us=us;
  ++s->samples;
  taskEXIT_CRITICAL();
}
bool ControlTiming_GetSection(ControlTimingSection section,ControlSectionTiming *status)
{
  if ((uint32_t)section>=CONTROL_SECTION_COUNT || status==NULL) return false;
  taskENTER_CRITICAL(); *status=s_sections[section]; taskEXIT_CRITICAL();
  return true;
}

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
