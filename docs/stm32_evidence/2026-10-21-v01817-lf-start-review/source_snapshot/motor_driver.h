/**
 * @file motor_driver.h
 * @brief Safe four-channel TB6612 motor output abstraction.
 *
 * Commands use chassis semantics: positive is physical wheel-forward and
 * negative is physical wheel-reverse. Per-wheel electrical polarity is
 * applied from board_config.h inside the driver.
 */

#ifndef APPLICATION_CHASSIS_MOTOR_DRIVER_H_
#define APPLICATION_CHASSIS_MOTOR_DRIVER_H_

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define MOTOR_DRIVER_COMMAND_MAX_PERMILLE 1000

typedef enum
{
  MOTOR_DRIVER_WHEEL_LF = 0,
  MOTOR_DRIVER_WHEEL_LR,
  MOTOR_DRIVER_WHEEL_RF,
  MOTOR_DRIVER_WHEEL_RR,
  MOTOR_DRIVER_WHEEL_COUNT
} MotorDriverWheel;

typedef struct
{
  bool initialized;
  bool standby_enabled;
  int16_t command_permille[MOTOR_DRIVER_WHEEL_COUNT];
} MotorDriverStatus;

/**
 * Start all four TIM8 PWM channels at zero duty while D24A STBY (PD10) is low.
 * The reserved/unconnected PD11 output is also kept low. This function never
 * enables the external motor driver.
 */
bool MotorDriver_Init(void);

/**
 * Enable the D24A common STBY on PD10 from an all-coast, zero-duty state.
 */
bool MotorDriver_Enable(void);

/** Disable D24A STBY first, then clear all directions and PWM. */
void MotorDriver_Disable(void);

/**
 * Set one signed chassis-direction command in permille (positive = forward).
 *
 * Values are saturated to [-1000, 1000].  A non-zero command is rejected
 * while standby is disabled. Sign changes coast first and wait >=2 HAL ticks
 * for zero CCR to latch; callers retry each control tick (no blocking wait).
 * Same-direction updates write only CCR. Status reports APPLIED logical
 * command (zero during guard), not a queued request or the
 * polarity-adjusted electrical command.
 */
bool MotorDriver_SetCommand(MotorDriverWheel wheel, int16_t command_permille);

/** Coast one wheel with both direction inputs low and PWM at zero. */
bool MotorDriver_Coast(MotorDriverWheel wheel);

/** Coast all four wheels without changing the standby state. */
void MotorDriver_CoastAll(void);

/**
 * Immediately restore the board-level safe state and invalidate the driver.
 * MotorDriver_Init() is required before outputs can be enabled again.
 */
void MotorDriver_ForceSafeState(void);

/** Copy the current software-visible motor output state. */
bool MotorDriver_GetStatus(MotorDriverStatus *status);

#ifdef __cplusplus
}
#endif

#endif /* APPLICATION_CHASSIS_MOTOR_DRIVER_H_ */
