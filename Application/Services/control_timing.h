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
  uint32_t execution_last_us;
  uint32_t execution_max_us;
  uint32_t execution_samples;
} ControlTimingStatus;

typedef enum { CONTROL_SECTION_ACQUISITION, CONTROL_SECTION_CONTROLLER,
               CONTROL_SECTION_TRACE, CONTROL_SECTION_COUNT } ControlTimingSection;
typedef struct { uint32_t last_us, max_us, samples; } ControlSectionTiming;

/* ARM DWT counter measures elapsed CPU cycles, including preemption. */
void ControlTiming_EnableCycleCounter(void);
uint32_t ControlTiming_ReadCycles(void);
void ControlTiming_RecordExecution(ControlTimingTask task, uint32_t cycles, uint32_t cpu_hz);
void ControlTiming_RecordSection(ControlTimingSection section, uint32_t cycles, uint32_t cpu_hz);
bool ControlTiming_GetSection(ControlTimingSection section, ControlSectionTiming *status);

void ControlTiming_RecordStart(ControlTimingTask task, uint32_t now_tick);
/* Millisecond kernel tick. Skip missed slots instead of busy catch-up loops. */
uint32_t ControlTiming_NextWake(ControlTimingTask task,
  uint32_t previous_deadline, uint32_t now_tick, uint32_t period_ms);
bool ControlTiming_GetStatus(ControlTimingTask task, ControlTimingStatus *status);
#endif
