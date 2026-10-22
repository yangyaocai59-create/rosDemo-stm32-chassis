#include "chassis_motion_profile.h"
#include <math.h>
#include <stddef.h>

static float StepAxis(float current, float target, float accel,
                      float decel, float dt)
{
  const bool reversing = ((current > 0.0f) && (target < 0.0f)) ||
                         ((current < 0.0f) && (target > 0.0f));
  if (reversing) { target = 0.0f; }
  const float rate = (reversing || (fabsf(target) < fabsf(current))) ?
    decel : accel;
  const float change = rate * dt;
  if (target > current + change) { return current + change; }
  if (target < current - change) { return current - change; }
  return target;
}

bool ChassisMotionProfile_Step(const ChassisVelocityCommand *current,
  const ChassisVelocityCommand *target, uint32_t elapsed_ms,
  ChassisVelocityCommand *result)
{
  if ((current == NULL) || (target == NULL) || (result == NULL) ||
      (elapsed_ms == 0U) || (elapsed_ms > CHASSIS_PROFILE_MAX_STEP_MS) ||
      !isfinite(current->linear_x_m_s) || !isfinite(current->angular_z_rad_s) ||
      !isfinite(target->linear_x_m_s) || !isfinite(target->angular_z_rad_s))
  { return false; }
  const float dt = (float)elapsed_ms / 1000.0f;
  const ChassisVelocityCommand next = {
    StepAxis(current->linear_x_m_s, target->linear_x_m_s,
      CHASSIS_PROFILE_LINEAR_ACCEL_M_S2, CHASSIS_PROFILE_LINEAR_DECEL_M_S2, dt),
    StepAxis(current->angular_z_rad_s, target->angular_z_rad_s,
      CHASSIS_PROFILE_ANGULAR_ACCEL_RAD_S2, CHASSIS_PROFILE_ANGULAR_DECEL_RAD_S2, dt)
  };
  *result = next;
  return true;
}
