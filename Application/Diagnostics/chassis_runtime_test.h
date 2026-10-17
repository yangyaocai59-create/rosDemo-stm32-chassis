/**
 * @file chassis_runtime_test.h
 * @brief Debug-only supervised validation of production command ownership.
 */

#ifndef APPLICATION_DIAGNOSTICS_CHASSIS_RUNTIME_TEST_H_
#define APPLICATION_DIAGNOSTICS_CHASSIS_RUNTIME_TEST_H_

#include <stdbool.h>
#include <stdint.h>

#ifndef MOTOR_OPEN_LOOP_TEST_ENABLED
#define MOTOR_OPEN_LOOP_TEST_ENABLED 0
#endif

#ifdef __cplusplus
extern "C" {
#endif

void ChassisRuntimeTest_Init(void);
bool ChassisRuntimeTest_Arm(uint32_t now_ms);
bool ChassisRuntimeTest_Start(uint32_t now_ms);
void ChassisRuntimeTest_Process(uint32_t now_ms);
void ChassisRuntimeTest_SafetyProcess(uint32_t now_ms);
void ChassisRuntimeTest_Stop(void);
bool ChassisRuntimeTest_IsArmed(void);
bool ChassisRuntimeTest_IsActive(void);

#ifdef __cplusplus
}
#endif

#endif /* APPLICATION_DIAGNOSTICS_CHASSIS_RUNTIME_TEST_H_ */
