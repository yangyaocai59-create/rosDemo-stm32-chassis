/**
 * @file wheel_speed_controller.h
 * @brief Reusable per-wheel speed feedforward and PI controller.
 */

#ifndef APPLICATION_CHASSIS_WHEEL_SPEED_CONTROLLER_H_
#define APPLICATION_CHASSIS_WHEEL_SPEED_CONTROLLER_H_

#include <stdbool.h>
#include <stdint.h>

#include "motor_driver.h"
#include "chassis_control_config.h"
#if CHASSIS_CONTROL_V2
#include "wheel_control_core.h"
#endif

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
  float target_counts_per_second;
  float measured_counts_per_second;
  float error_counts_per_second;
  float feedforward_permille;
  float proportional_permille;
  float integrator_permille;
  int16_t command_permille;
  bool limited;
  /** Detailed limit bits in revision 200; legacy callers retain limited. */
  uint32_t limit_reasons;
} WheelSpeedControllerState;

typedef enum
{
  WHEEL_SPEED_CONTROL_MODE_NORMAL = 0,
  WHEEL_SPEED_CONTROL_MODE_PIVOT
} WheelSpeedControlMode;

#if CHASSIS_CONTROL_V2
/* Single motor-task producer. Caller serializes Begin/Commit with Stop/Reset;
 * Step only touches the private transaction, with interrupts enabled. */
typedef struct {
  WheelControlMemory memory[MOTOR_DRIVER_WHEEL_COUNT];
  WheelSpeedControllerState state[MOTOR_DRIVER_WHEEL_COUNT];
  uint32_t revision;
} WheelSpeedTransaction;
bool WheelSpeedController_Begin(WheelSpeedTransaction *transaction);
bool WheelSpeedController_Step(WheelSpeedTransaction *transaction,
  MotorDriverWheel wheel, float target, float measured, float bias,
  WheelSpeedControlMode mode, uint32_t elapsed_ms, int16_t *command);
bool WheelSpeedController_Commit(const WheelSpeedTransaction *transaction);
/** Static FF envelope only, not a measured loaded motor capability. */
float WheelSpeedController_GetFeedforwardTargetLimit(MotorDriverWheel wheel,
  bool forward, WheelSpeedControlMode mode);
#endif

/** Initialize four independent controller instances. */
void WheelSpeedController_Init(void);
/** Mode observation only; caller serializes with updates/reset. Safety owns timers. */
bool WheelSpeedController_IsPivotMode(MotorDriverWheel wheel);

/** Clear one wheel controller before a new enable, stop, or direction phase. */
bool WheelSpeedController_Reset(MotorDriverWheel wheel);

/**
 * Seed the output history before a supervised bumpless closed-loop takeover.
 * The next update still applies the configured magnitude and slew limits.
 */
bool WheelSpeedController_PrimeOutput(MotorDriverWheel wheel,
                                      int16_t command_permille);

/** Fixed, target-ramped pivot feedforward. Pure lookup; no recovery state. */
float WheelSpeedController_GetPivotLoadFeedforward(
  MotorDriverWheel wheel, float target_counts_per_second);

/**
 * Calculate one signed motor command.
 *
 * This function performs no GPIO or timer access. The caller owns motor
 * enabling and applies the returned command through MotorDriver_SetCommand().
 */
bool WheelSpeedController_Update(MotorDriverWheel wheel,
                                 float target_counts_per_second,
                                 float measured_counts_per_second,
                                 uint32_t elapsed_ms,
                                 int16_t *command_permille);

/** Update with an additional signed, maneuver-specific feedforward term. */
bool WheelSpeedController_UpdateWithFeedforwardBias(
  MotorDriverWheel wheel,
  float target_counts_per_second,
  float measured_counts_per_second,
  float feedforward_bias_permille,
  uint32_t elapsed_ms,
  int16_t *command_permille);

/** One control step. PIVOT uses continuous direction-normalized PI;
 * NORMAL preserves the established rolling/straight law. Pivot bias must be zero. */
bool WheelSpeedController_UpdateWithMode(
  MotorDriverWheel wheel,
  float target_counts_per_second,
  float measured_counts_per_second,
  float feedforward_bias_permille,
  WheelSpeedControlMode mode,
  uint32_t elapsed_ms,
  int16_t *command_permille);

/** Copy the latest diagnostic state for one wheel. */
bool WheelSpeedController_GetState(MotorDriverWheel wheel,
                                   WheelSpeedControllerState *state);

#ifdef __cplusplus
}
#endif

#endif /* APPLICATION_CHASSIS_WHEEL_SPEED_CONTROLLER_H_ */
