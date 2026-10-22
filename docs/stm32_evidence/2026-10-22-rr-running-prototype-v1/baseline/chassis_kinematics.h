/**
 * @file chassis_kinematics.h
 * @brief Four-wheel differential-drive forward and inverse kinematics.
 */

#ifndef APPLICATION_CHASSIS_CHASSIS_KINEMATICS_H_
#define APPLICATION_CHASSIS_CHASSIS_KINEMATICS_H_

#include <stdbool.h>

#include "motor_driver.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
  /** ROS base_link convention: positive x is chassis forward. */
  float linear_x_m_s;
  /** ROS convention: positive z is counter-clockwise/left turn. */
  float angular_z_rad_s;
} ChassisVelocityCommand;

typedef struct
{
  float counts_per_second[MOTOR_DRIVER_WHEEL_COUNT];
  float left_linear_m_s;
  float right_linear_m_s;
  /** 1.0 unless both sides were proportionally reduced to respect a limit. */
  float scale;
} ChassisWheelTargets;

typedef struct
{
  float linear_x_m_s;
  float angular_z_rad_s;
  float left_linear_m_s;
  float right_linear_m_s;
} ChassisVelocityEstimate;

/**
 * Convert a chassis command to four wheel encoder rates.
 *
 * If either side exceeds max_abs_counts_per_second, all wheel targets are
 * scaled by the same factor so the requested curvature is preserved.
 */
bool ChassisKinematics_ToWheelTargets(
  const ChassisVelocityCommand *command,
  float max_abs_counts_per_second,
  ChassisWheelTargets *targets);

/** Convert four measured encoder rates back to chassis linear/angular speed. */
bool ChassisKinematics_FromWheelSpeeds(
  const float counts_per_second[MOTOR_DRIVER_WHEEL_COUNT],
  ChassisVelocityEstimate *estimate);

#ifdef __cplusplus
}
#endif

#endif /* APPLICATION_CHASSIS_CHASSIS_KINEMATICS_H_ */
