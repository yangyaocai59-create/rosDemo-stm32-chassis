#ifndef APPLICATION_SERVICES_CONTROL_TIMING_H_
#define APPLICATION_SERVICES_CONTROL_TIMING_H_
#include <stdbool.h>
#include <stdint.h>

typedef enum { CONTROL_TIMING_MOTOR, CONTROL_TIMING_SAFETY,
               CONTROL_TIMING_COUNT } ControlTimingTask;
typedef struct
{
  uint32_t iterations;
  uint32_t period_min_ms;
  uint32_t period_max_ms;
  uint32_t overrun_count;
  uint32_t missed_deadlines;
} ControlTimingStatus;

void ControlTiming_RecordStart(ControlTimingTask task, uint32_t now_tick);
/* Millisecond kernel tick. Skip missed slots instead of busy catch-up loops. */
uint32_t ControlTiming_NextWake(ControlTimingTask task,
  uint32_t previous_deadline, uint32_t now_tick, uint32_t period_ms);
bool ControlTiming_GetStatus(ControlTimingTask task, ControlTimingStatus *status);
#endif
