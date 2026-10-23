/**
 * @file motor_speed_control_test.h
 * @brief Debug-only, supervised single-wheel PI speed-control validation.
 */

#ifndef APPLICATION_DIAGNOSTICS_MOTOR_SPEED_CONTROL_TEST_H_
#define APPLICATION_DIAGNOSTICS_MOTOR_SPEED_CONTROL_TEST_H_

#include <stdbool.h>
#include <stdint.h>

#include "motor_driver.h"

#ifndef MOTOR_OPEN_LOOP_TEST_ENABLED
#define MOTOR_OPEN_LOOP_TEST_ENABLED 0
#endif

#ifdef __cplusplus
extern "C" {
#endif

void MotorSpeedControlTest_Init(void);
bool MotorSpeedControlTest_Arm(MotorDriverWheel wheel, uint32_t now_ms);
bool MotorSpeedControlTest_Start(uint32_t now_ms);
bool MotorSpeedControlTest_IsActive(void);
void MotorSpeedControlTest_Process(uint32_t now_ms);
void MotorSpeedControlTest_Stop(void);
void MotorSpeedControlTest_SafetyProcess(uint32_t now_ms);

#ifdef __cplusplus
}
#endif

#endif /* APPLICATION_DIAGNOSTICS_MOTOR_SPEED_CONTROL_TEST_H_ */
