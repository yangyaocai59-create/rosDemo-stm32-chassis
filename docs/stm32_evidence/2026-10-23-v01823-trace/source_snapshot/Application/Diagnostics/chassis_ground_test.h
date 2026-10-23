/**
 * @file chassis_ground_test.h
 * @brief Debug-only, supervised single-action low-speed ground validation.
 */

#ifndef APPLICATION_DIAGNOSTICS_CHASSIS_GROUND_TEST_H_
#define APPLICATION_DIAGNOSTICS_CHASSIS_GROUND_TEST_H_

#include <stdbool.h>
#include <stdint.h>

#ifndef MOTOR_OPEN_LOOP_TEST_ENABLED
#define MOTOR_OPEN_LOOP_TEST_ENABLED 0
#endif

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
  CHASSIS_GROUND_ACTION_FORWARD = 0,
  CHASSIS_GROUND_ACTION_REVERSE,
  CHASSIS_GROUND_ACTION_TURN_LEFT,
  CHASSIS_GROUND_ACTION_TURN_RIGHT,
  CHASSIS_GROUND_ACTION_ARC_LEFT,
  CHASSIS_GROUND_ACTION_ARC_RIGHT,
  CHASSIS_GROUND_ACTION_COUNT
} ChassisGroundTestAction;

void ChassisGroundTest_Init(void);
bool ChassisGroundTest_Arm(uint32_t now_ms);
bool ChassisGroundTest_Start(ChassisGroundTestAction action,
                             uint32_t now_ms);
bool ChassisGroundTest_IsArmed(void);
bool ChassisGroundTest_IsActive(void);
void ChassisGroundTest_Process(uint32_t now_ms);
void ChassisGroundTest_Stop(void);
void ChassisGroundTest_SafetyProcess(uint32_t now_ms);

#ifdef __cplusplus
}
#endif

#endif /* APPLICATION_DIAGNOSTICS_CHASSIS_GROUND_TEST_H_ */
