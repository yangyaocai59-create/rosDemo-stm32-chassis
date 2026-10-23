#ifndef APPLICATION_DIAGNOSTICS_MOTION_TRACE_H_
#define APPLICATION_DIAGNOSTICS_MOTION_TRACE_H_
#include <stdbool.h>
#include <stdint.h>

/* Only motorTask, after output processing. Diagnostic-only first-anomaly
 * pre/post capture; never submits commands or owns outputs. Frozen data survive
 * later motion until dumped (or MCU resets). */
void MotionTrace_Record(uint32_t now_ms);
/* Call only while stopped. Recording freezes until telemetry finishes dumping. */
bool MotionTrace_RequestDump(void);
/* Single telemetry-task consumer: one sample per call, UART only here. */
bool MotionTrace_ProcessDump(void);
#endif
