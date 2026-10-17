/**
 * @file four_wheel_speed_control_test.h
 * @brief Debug-only, supervised four-wheel PI synchronization validation.
 */

#ifndef APPLICATION_DIAGNOSTICS_FOUR_WHEEL_SPEED_CONTROL_TEST_H_
#define APPLICATION_DIAGNOSTICS_FOUR_WHEEL_SPEED_CONTROL_TEST_H_

#include <stdbool.h>
#include <stdint.h>

#ifndef MOTOR_OPEN_LOOP_TEST_ENABLED
#define MOTOR_OPEN_LOOP_TEST_ENABLED 0
#endif

#ifdef __cplusplus
extern "C" {
#endif

void FourWheelSpeedControlTest_Init(void);
bool FourWheelSpeedControlTest_Arm(uint32_t now_ms);
bool FourWheelSpeedControlTest_Start(uint32_t now_ms);
bool FourWheelSpeedControlTest_IsActive(void);
void FourWheelSpeedControlTest_Process(uint32_t now_ms);
void FourWheelSpeedControlTest_Stop(void);
void FourWheelSpeedControlTest_SafetyProcess(uint32_t now_ms);

#ifdef __cplusplus
}
#endif

#endif /* APPLICATION_DIAGNOSTICS_FOUR_WHEEL_SPEED_CONTROL_TEST_H_ */
