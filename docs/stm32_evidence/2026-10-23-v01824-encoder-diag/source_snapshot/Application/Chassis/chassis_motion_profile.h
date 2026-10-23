#ifndef APPLICATION_CHASSIS_MOTION_PROFILE_H_
#define APPLICATION_CHASSIS_MOTION_PROFILE_H_

#include <stdint.h>
#include "chassis_kinematics.h"

/* Initial conservative engineering limits, not a loaded-chassis tune. */
#define CHASSIS_PROFILE_LINEAR_ACCEL_M_S2    0.20f
#define CHASSIS_PROFILE_LINEAR_DECEL_M_S2    0.30f
#define CHASSIS_PROFILE_ANGULAR_ACCEL_RAD_S2 1.50f
#define CHASSIS_PROFILE_ANGULAR_DECEL_RAD_S2 2.00f
#define CHASSIS_PROFILE_MAX_STEP_MS          100U

/* Pure normal-motion limiter. No GPIO, RTOS, or stored state.
 * A reversal reaches zero before accelerating in the opposite direction.
 * Safety stops bypass this function entirely. */
bool ChassisMotionProfile_Step(const ChassisVelocityCommand *current,
  const ChassisVelocityCommand *target, uint32_t elapsed_ms,
  ChassisVelocityCommand *result);

#endif
