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

typedef enum
{
  WHEEL_SPEED_CONTROL_MODE_NORMAL = 0,
  WHEEL_SPEED_CONTROL_MODE_PIVOT
} WheelSpeedControlMode;

/** Front-pivot maneuver-local diagnostics; magnitudes in permille.
 * No wire/trace layout is changed. Rear wheels do not use this policy. */
typedef struct
{
  float load_permille;
  float recovery_permille;
  uint32_t still_ms;
  uint32_t moving_ms;
  uint32_t load_overspeed_ms;
  uint32_t recovery_ms;
  uint32_t no_motion_ms;
  int8_t sign;
  bool active;
  bool recovering;
  bool exhausted;
  bool recovery_failed;
} WheelSpeedFrontPivotState;

/** Initialize all controller instances with zero output and integral state. */
void WheelSpeedController_Init(void);

/* Internal chassis/PI handshake. No USB or public-state layout change.
 * Selection requires old startup bias zero and prior confirmed motion. */
bool WheelSpeedController_SelectLrRunningSupport(bool may_start);
void WheelSpeedController_ClearLrRunningSupport(void);

/** Read-only safety query; call within the fault-manager critical section.
 * Checks the observed target against the latest PI target, not a timestamped
 * atomic snapshot of every chassis/PI field. Ownership persists across the
 * entire running maneuver (not just a temporary pulse); a different target,
 * reset, direction or mode exit invalidates it. No public/wire struct change. */
bool WheelSpeedController_IsLrRunningSupportOwned(float observed_target_cps);

/* Offline RR-forward analogue. Same zero-legacy-bias/motion owner contract,
 * and same read-only safety-query/critical-section requirement as LR. */
bool WheelSpeedController_SelectRrRunningSupport(bool may_start);
void WheelSpeedController_ClearRrRunningSupport(void);
bool WheelSpeedController_IsRrRunningSupportOwned(float observed_target_cps);

/** Clear one wheel controller before a new enable, stop, or direction phase. */
bool WheelSpeedController_Reset(MotorDriverWheel wheel);

/**
 * Seed the output history before a supervised bumpless closed-loop takeover.
 * The next update still applies the configured magnitude and slew limits.
 */
bool WheelSpeedController_PrimeOutput(MotorDriverWheel wheel,
                                      int16_t command_permille);

/**
 * Release a maneuver bias by transferring it into the bounded PI integral.
 * Returns the bias that could not fit in the integral (or NAN on bad input).
 * The caller must immediately use the returned bias for the next update.
 * Integral + bias is conserved; output history and GPIO are untouched.
 */
float WheelSpeedController_ReleaseFeedforwardBias(
  MotorDriverWheel wheel, float feedforward_bias_permille);

/** Signed, speed-ramped pivot running-load CEILING (NAN if invalid).
 * Pure lookup only; actual front support may be withdrawn below this value.
 */
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

/**
 * Pivot mode uses front-only adaptive load/recovery, plus proportional
 * damping only for same-direction overspeed. LR reverse running support is
 * PI-owned after explicit chassis handoff; other rear support is caller-owned.
 * Normal-mode control retains the baseline gains and feedforward.
 */
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

/** Copy maneuver-local front support state; false for rear/invalid/NULL. */
bool WheelSpeedController_GetFrontPivotState(
  MotorDriverWheel wheel, WheelSpeedFrontPivotState *state);

#ifdef __cplusplus
}
#endif

#endif /* APPLICATION_CHASSIS_WHEEL_SPEED_CONTROLLER_H_ */
