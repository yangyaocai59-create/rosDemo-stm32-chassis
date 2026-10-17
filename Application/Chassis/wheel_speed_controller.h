/**
 * @file wheel_speed_controller.h
 * @brief Reusable per-wheel speed feedforward and PI controller.
 */

#ifndef APPLICATION_CHASSIS_WHEEL_SPEED_CONTROLLER_H_
#define APPLICATION_CHASSIS_WHEEL_SPEED_CONTROLLER_H_

#include <stdbool.h>
#include <stdint.h>

#include "motor_driver.h"

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
} WheelSpeedControllerState;

/** Initialize all controller instances with zero output and integral state. */
void WheelSpeedController_Init(void);

/** Clear one wheel controller before a new enable, stop, or direction phase. */
bool WheelSpeedController_Reset(MotorDriverWheel wheel);

/**
 * Seed the output history before a supervised bumpless closed-loop takeover.
 * The next update still applies the configured magnitude and slew limits.
 */
bool WheelSpeedController_PrimeOutput(MotorDriverWheel wheel,
                                      int16_t command_permille);

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

/** Copy the latest diagnostic state for one wheel. */
bool WheelSpeedController_GetState(MotorDriverWheel wheel,
                                   WheelSpeedControllerState *state);

#ifdef __cplusplus
}
#endif

#endif /* APPLICATION_CHASSIS_WHEEL_SPEED_CONTROLLER_H_ */
