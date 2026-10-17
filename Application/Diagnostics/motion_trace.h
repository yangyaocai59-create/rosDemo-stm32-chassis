#ifndef APPLICATION_DIAGNOSTICS_MOTION_TRACE_H_
#define APPLICATION_DIAGNOSTICS_MOTION_TRACE_H_
#include <stdbool.h>
#include <stdint.h>

/* Read-only bounded diagnostics. Never submits commands or owns outputs. */
void MotionTrace_Record(uint32_t now_ms);
/* Call only while stopped. Recording freezes until telemetry finishes dumping. */
bool MotionTrace_RequestDump(void);
/* Single telemetry-task consumer: one sample per call, UART only here. */
bool MotionTrace_ProcessDump(void);
#endif
