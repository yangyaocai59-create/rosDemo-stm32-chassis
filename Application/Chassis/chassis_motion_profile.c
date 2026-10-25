#include "chassis_motion_profile.h"
#include <math.h>
#include <stddef.h>
#include "chassis_control_config.h"
#include "chassis_config.h"

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
  ChassisVelocityCommand next = {
    StepAxis(current->linear_x_m_s, target->linear_x_m_s,
      CHASSIS_PROFILE_LINEAR_ACCEL_M_S2, CHASSIS_PROFILE_LINEAR_DECEL_M_S2, dt),
    StepAxis(current->angular_z_rad_s, target->angular_z_rad_s,
      CHASSIS_PROFILE_ANGULAR_ACCEL_RAD_S2, CHASSIS_PROFILE_ANGULAR_DECEL_RAD_S2, dt)
  };
#if CHASSIS_CONTROL_V2
  /* Preserve each axis's existing limits for pure straight/pivot motion.
   * Mixed commands share the larger single-axis wheel acceleration budget,
   * rather than silently adding both independent budgets on one wheel. */
  const float half_track=CHASSIS_EFFECTIVE_TRACK_M*.5f;
  const float acceleration=fmaxf(CHASSIS_PROFILE_LINEAR_ACCEL_M_S2,
    half_track*CHASSIS_PROFILE_ANGULAR_ACCEL_RAD_S2);
  const float deceleration=fmaxf(CHASSIS_PROFILE_LINEAR_DECEL_M_S2,
    half_track*CHASSIS_PROFILE_ANGULAR_DECEL_RAD_S2);
  const float dv=next.linear_x_m_s-current->linear_x_m_s;
  const float dw=half_track*(next.angular_z_rad_s-current->angular_z_rad_s);
  float scale=1.0f;
  for (int side=-1;side<=1;side+=2) {
    const float before=current->linear_x_m_s+(float)side*half_track*current->angular_z_rad_s;
    const float change=dv+(float)side*dw;
    const float after=before+change;
    const float allowed=(fabsf(after)<fabsf(before)?deceleration:acceleration)*dt;
    if (fabsf(change)>allowed) scale=fminf(scale,allowed/fabsf(change));
  }
  next.linear_x_m_s=current->linear_x_m_s+scale*dv;
  next.angular_z_rad_s=current->angular_z_rad_s+scale*(next.angular_z_rad_s-current->angular_z_rad_s);
#endif
  *result = next;
  return true;
}
