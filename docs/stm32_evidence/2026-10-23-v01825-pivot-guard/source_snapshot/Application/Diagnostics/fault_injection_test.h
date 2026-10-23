/**
 * @file fault_injection_test.h
 * @brief Debug-only, software fault injection for the health manager.
 */

#ifndef APPLICATION_DIAGNOSTICS_FAULT_INJECTION_TEST_H_
#define APPLICATION_DIAGNOSTICS_FAULT_INJECTION_TEST_H_

#include <stdbool.h>
#include <stdint.h>

#include "fault_manager.h"
#include "motor_driver.h"

#ifndef FAULT_INJECTION_TEST_ENABLED
#define FAULT_INJECTION_TEST_ENABLED 0
#endif

#ifdef __cplusplus
extern "C" {
#endif

/** Reset every injection source to its inactive, fail-safe state. */
void FaultInjectionTest_Init(void);

/** Expire an unfinished two-character command prefix. */
void FaultInjectionTest_Process(uint32_t now_ms);

/**
 * Consume Debug UART commands. Commands use the prefix 'z', followed by:
 * m=motor heartbeat, u=USB-RX heartbeat, 1..4=LF..RR no-feedback,
 * 0=disable all injection, ?=status.
 */
bool FaultInjectionTest_HandleCommand(uint8_t command, uint32_t now_ms);

/** False only when the selected heartbeat is intentionally suppressed. */
bool FaultInjectionTest_ShouldReportHeartbeat(FaultManagerTask task);

/** Copy measured wheel speeds and apply any selected no-feedback injection. */
void FaultInjectionTest_FilterMeasuredSpeed(
  const float source[MOTOR_DRIVER_WHEEL_COUNT],
  float filtered[MOTOR_DRIVER_WHEEL_COUNT]);

/** True while a heartbeat or wheel-feedback injection is selected. */
bool FaultInjectionTest_IsEnabled(void);

#ifdef __cplusplus
}
#endif

#endif /* APPLICATION_DIAGNOSTICS_FAULT_INJECTION_TEST_H_ */
