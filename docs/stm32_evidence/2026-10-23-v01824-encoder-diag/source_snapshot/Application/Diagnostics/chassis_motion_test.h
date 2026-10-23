/**
 * @file chassis_motion_test.h
 * @brief Debug-only lifted-wheel chassis kinematics validation.
 */

#ifndef APPLICATION_DIAGNOSTICS_CHASSIS_MOTION_TEST_H_
#define APPLICATION_DIAGNOSTICS_CHASSIS_MOTION_TEST_H_

#include <stdbool.h>
#include <stdint.h>

#ifndef MOTOR_OPEN_LOOP_TEST_ENABLED
#define MOTOR_OPEN_LOOP_TEST_ENABLED 0
#endif

#ifdef __cplusplus
extern "C" {
#endif

void ChassisMotionTest_Init(void);
bool ChassisMotionTest_Arm(uint32_t now_ms);
bool ChassisMotionTest_Start(uint32_t now_ms);
bool ChassisMotionTest_IsActive(void);
void ChassisMotionTest_Process(uint32_t now_ms);
void ChassisMotionTest_Stop(void);
void ChassisMotionTest_SafetyProcess(uint32_t now_ms);

#ifdef __cplusplus
}
#endif

#endif /* APPLICATION_DIAGNOSTICS_CHASSIS_MOTION_TEST_H_ */
