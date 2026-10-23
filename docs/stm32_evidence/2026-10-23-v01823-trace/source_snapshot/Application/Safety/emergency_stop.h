/**
 * @file emergency_stop.h
 * @brief Reset-latched, active-low emergency-stop input handling.
 */

#ifndef APPLICATION_SAFETY_EMERGENCY_STOP_H_
#define APPLICATION_SAFETY_EMERGENCY_STOP_H_

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Sample PC13 after GPIO initialization and latch an already-active input. */
void EmergencyStop_Init(void);

/**
 * Latch the emergency stop and immediately remove all motor drive.
 * This routine is safe to call from the GPIO EXTI callback.
 */
void EmergencyStop_Latch(void);

/** Return true once triggered; only an MCU reset clears this latch. */
bool EmergencyStop_IsLatched(void);

/** Return true while the active-low PC13 input is physically asserted. */
bool EmergencyStop_IsInputActive(void);

/** Return true only when neither the input nor its reset-only latch is active. */
bool EmergencyStop_CanEnableMotor(void);

/** Consume the one-shot task-context notification without clearing the latch. */
bool EmergencyStop_TakeTripNotice(void);

#ifdef __cplusplus
}
#endif

#endif /* APPLICATION_SAFETY_EMERGENCY_STOP_H_ */
