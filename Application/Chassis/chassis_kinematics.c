/**
 * @file chassis_kinematics.c
 * @brief Four-wheel differential-drive kinematic conversions.
 */

#include "chassis_kinematics.h"

#include <stddef.h>

#include "chassis_config.h"

#define CHASSIS_KINEMATICS_COMMAND_SANITY_LIMIT 100.0f
#define CHASSIS_KINEMATICS_NUMERIC_SANITY_LIMIT 1000000.0f

static float ChassisKinematics_Abs(float value)
{
  return (value < 0.0f) ? -value : value;
}

static bool ChassisKinematics_IsFiniteRange(float value, float limit)
{
  return (value == value) &&
         (value <= limit) &&
         (value >= -limit);
}

bool ChassisKinematics_ToWheelTargets(
  const ChassisVelocityCommand *command,
  float max_abs_counts_per_second,
  ChassisWheelTargets *targets)
{
  const float counts_per_metre =
    (float)CHASSIS_ENCODER_COUNTS_PER_WHEEL_REV /
    CHASSIS_WHEEL_CIRCUMFERENCE_M;
  float left_linear;
  float right_linear;
  float left_cps;
  float right_cps;
  float maximum;
  float scale = 1.0f;

  if ((command == NULL) || (targets == NULL) ||
      (!ChassisKinematics_IsFiniteRange(
        command->linear_x_m_s,
        CHASSIS_KINEMATICS_COMMAND_SANITY_LIMIT)) ||
      (!ChassisKinematics_IsFiniteRange(
        command->angular_z_rad_s,
        CHASSIS_KINEMATICS_COMMAND_SANITY_LIMIT)) ||
      (!ChassisKinematics_IsFiniteRange(
        max_abs_counts_per_second,
        CHASSIS_KINEMATICS_NUMERIC_SANITY_LIMIT)) ||
      (max_abs_counts_per_second <= 0.0f))
  {
    return false;
  }

  left_linear = command->linear_x_m_s -
    command->angular_z_rad_s * (CHASSIS_EFFECTIVE_TRACK_M * 0.5f);
  right_linear = command->linear_x_m_s +
    command->angular_z_rad_s * (CHASSIS_EFFECTIVE_TRACK_M * 0.5f);
  left_cps = left_linear * counts_per_metre;
  right_cps = right_linear * counts_per_metre;

  maximum = ChassisKinematics_Abs(left_cps);
  if (ChassisKinematics_Abs(right_cps) > maximum)
  {
    maximum = ChassisKinematics_Abs(right_cps);
  }
  if (maximum > max_abs_counts_per_second)
  {
    scale = max_abs_counts_per_second / maximum;
    left_linear *= scale;
    right_linear *= scale;
    left_cps *= scale;
    right_cps *= scale;
  }

  targets->counts_per_second[MOTOR_DRIVER_WHEEL_LF] = left_cps;
  targets->counts_per_second[MOTOR_DRIVER_WHEEL_LR] = left_cps;
  targets->counts_per_second[MOTOR_DRIVER_WHEEL_RF] = right_cps;
  targets->counts_per_second[MOTOR_DRIVER_WHEEL_RR] = right_cps;
  targets->left_linear_m_s = left_linear;
  targets->right_linear_m_s = right_linear;
  targets->scale = scale;
  return true;
}

bool ChassisKinematics_FromWheelSpeeds(
  const float counts_per_second[MOTOR_DRIVER_WHEEL_COUNT],
  ChassisVelocityEstimate *estimate)
{
  const float metres_per_count = CHASSIS_WHEEL_CIRCUMFERENCE_M /
    (float)CHASSIS_ENCODER_COUNTS_PER_WHEEL_REV;
  float left_cps;
  float right_cps;

  if ((counts_per_second == NULL) || (estimate == NULL))
  {
    return false;
  }

  for (uint32_t wheel = 0U; wheel < MOTOR_DRIVER_WHEEL_COUNT; ++wheel)
  {
    if (!ChassisKinematics_IsFiniteRange(
          counts_per_second[wheel],
          CHASSIS_KINEMATICS_NUMERIC_SANITY_LIMIT))
    {
      return false;
    }
  }

  left_cps = 0.5f *
    (counts_per_second[MOTOR_DRIVER_WHEEL_LF] +
     counts_per_second[MOTOR_DRIVER_WHEEL_LR]);
  right_cps = 0.5f *
    (counts_per_second[MOTOR_DRIVER_WHEEL_RF] +
     counts_per_second[MOTOR_DRIVER_WHEEL_RR]);
  estimate->left_linear_m_s = left_cps * metres_per_count;
  estimate->right_linear_m_s = right_cps * metres_per_count;
  estimate->linear_x_m_s = 0.5f *
    (estimate->left_linear_m_s + estimate->right_linear_m_s);
  estimate->angular_z_rad_s =
    (estimate->right_linear_m_s - estimate->left_linear_m_s) /
    CHASSIS_EFFECTIVE_TRACK_M;
  return true;
}
