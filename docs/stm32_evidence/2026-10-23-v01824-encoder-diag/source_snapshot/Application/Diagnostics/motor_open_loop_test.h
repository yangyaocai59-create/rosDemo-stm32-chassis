/**
 * @file motor_open_loop_test.h
 * @brief Debug-only, explicitly armed four-wheel motor open-loop test.
 */

#ifndef APPLICATION_DIAGNOSTICS_MOTOR_OPEN_LOOP_TEST_H_
#define APPLICATION_DIAGNOSTICS_MOTOR_OPEN_LOOP_TEST_H_

#include <stdbool.h>
#include <stdint.h>

#ifndef MOTOR_OPEN_LOOP_TEST_ENABLED
#define MOTOR_OPEN_LOOP_TEST_ENABLED 0
#endif

#ifdef __cplusplus
extern "C" {
#endif

/** Initialize the test in a disarmed, motor-disabled state. */
void MotorOpenLoopTest_Init(void);

/** Poll debug UART commands and report completed tests. Called by motorTask. */
void MotorOpenLoopTest_Process(void);

/** Independent timeout enforcement. Called periodically by safetyTask. */
void MotorOpenLoopTest_SafetyProcess(void);

/** Stop every debug motion owner. Safe to call from USB task context. */
void MotorOpenLoopTest_StopAll(uint32_t now_ms);

/** True while any Debug motor diagnostic owns or has armed the motor path. */
bool MotorOpenLoopTest_IsActive(void);

#ifdef __cplusplus
}
#endif

#endif /* APPLICATION_DIAGNOSTICS_MOTOR_OPEN_LOOP_TEST_H_ */
