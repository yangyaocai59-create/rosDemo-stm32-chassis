/**
 * @file motor_characterization_test.h
 * @brief Debug-only, supervised PWM-to-speed characterization sequence.
 */

#ifndef APPLICATION_DIAGNOSTICS_MOTOR_CHARACTERIZATION_TEST_H_
#define APPLICATION_DIAGNOSTICS_MOTOR_CHARACTERIZATION_TEST_H_

#include <stdbool.h>
#include <stdint.h>

#ifndef MOTOR_OPEN_LOOP_TEST_ENABLED
#define MOTOR_OPEN_LOOP_TEST_ENABLED 0
#endif

#ifdef __cplusplus
extern "C" {
#endif

/** Reset the characterization state without enabling the motor driver. */
void MotorCharacterizationTest_Init(void);

/** Arm the sequence for five seconds. Motors remain disabled. */
bool MotorCharacterizationTest_Arm(uint32_t now_ms);

/**
 * Start a three-second countdown for a forward (+1) or reverse (-1) scan.
 * The call succeeds only while the characterization sequence is armed.
 */
bool MotorCharacterizationTest_Start(int8_t direction, uint32_t now_ms);

/** Return true for every state other than fully disarmed. */
bool MotorCharacterizationTest_IsActive(void);

/** Return true only during the five-second direction-selection window. */
bool MotorCharacterizationTest_IsArmed(void);

/** Advance the non-blocking sequence. Called by motorTask every 10 ms. */
void MotorCharacterizationTest_Process(uint32_t now_ms);

/** Cancel immediately and disable the common D24A standby output. */
void MotorCharacterizationTest_Stop(void);

/** Independently enforce per-point and whole-sequence timeouts. */
void MotorCharacterizationTest_SafetyProcess(uint32_t now_ms);

#ifdef __cplusplus
}
#endif

#endif /* APPLICATION_DIAGNOSTICS_MOTOR_CHARACTERIZATION_TEST_H_ */
