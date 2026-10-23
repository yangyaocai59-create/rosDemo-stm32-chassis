#include <cmath>
#include <cstdio>

#include "diff_drive_lib/diff_drive_controller.hpp"

namespace diff_drive_lib {

static constexpr float PI = 3.141592653F;

void DiffDriveController::setSpeed(const float linear_x, const float linear_y,
                                   const float angular) {
  if (params_.robot_input_timeout > 0)
    last_command_time_remaining_ = params_.robot_input_timeout;
  if (!enabled_) enable();

  const float angular_multiplied =
      angular * params_.robot_angular_velocity_multiplier;
  const float wheel_L_lin_vel =
      linear_x - (angular_multiplied * params_.robot_wheel_separation / 2.0F);
  const float wheel_R_lin_vel =
      linear_x + (angular_multiplied * params_.robot_wheel_separation / 2.0F);
  const float wheel_L_ang_vel = wheel_L_lin_vel / params_.robot_wheel_radius;
  const float wheel_R_ang_vel = wheel_R_lin_vel / params_.robot_wheel_radius;

  wheel_FL.setTargetVelocity(wheel_L_ang_vel);
  wheel_RL.setTargetVelocity(wheel_L_ang_vel);
  wheel_FR.setTargetVelocity(wheel_R_ang_vel);
  wheel_RR.setTargetVelocity(wheel_R_ang_vel);
}

void DiffDriveController::update(uint32_t dt_ms) {
  if (enabled_ && params_.robot_input_timeout > 0) {
    last_command_time_remaining_ -= dt_ms;
    if (last_command_time_remaining_ < 0) disable();
  }

  wheel_FL.update(dt_ms);
  wheel_RL.update(dt_ms);
  wheel_FR.update(dt_ms);
  wheel_RR.update(dt_ms);

  // velocity in radians per second
  const float FL_ang_vel = wheel_FL.getVelocity();
  const float RL_ang_vel = wheel_RL.getVelocity();
  const float FR_ang_vel = wheel_FR.getVelocity();
  const float RR_ang_vel = wheel_RR.getVelocity();

  const float L_ang_vel = (FL_ang_vel + RL_ang_vel) / 2.0F;
  const float R_ang_vel = (FR_ang_vel + RR_ang_vel) / 2.0F;

  // velocity in meters per second
  const float L_lin_vel = L_ang_vel * params_.robot_wheel_radius;
  const float R_lin_vel = R_ang_vel * params_.robot_wheel_radius;

  const float dt_s = static_cast<float>(dt_ms) * 0.001F;

  // linear (m/s) and angular (r/s) velocities of the robot
  odom_.velocity_lin_x = (L_lin_vel + R_lin_vel) / 2.0F;
  odom_.velocity_ang = (R_lin_vel - L_lin_vel) / params_.robot_wheel_separation;

  odom_.velocity_ang /= params_.robot_angular_velocity_multiplier;

  // Integrate the velocity using the rectangle rule
  odom_.pose_yaw += odom_.velocity_ang * dt_s;
  if (odom_.pose_yaw > 2.0F * PI)
    odom_.pose_yaw -= 2.0F * PI;
  else if (odom_.pose_yaw < 0.0F)
    odom_.pose_yaw += 2.0F * PI;

  odom_.pose_x += odom_.velocity_lin_x * std::cos(odom_.pose_yaw) * dt_s;
  odom_.pose_y += odom_.velocity_lin_x * std::sin(odom_.pose_yaw) * dt_s;
}

}  // namespace diff_drive_lib