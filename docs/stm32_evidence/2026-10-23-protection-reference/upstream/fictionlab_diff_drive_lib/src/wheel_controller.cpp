#include <algorithm>

#include "diff_drive_lib/motor_controller_interface.hpp"
#include "diff_drive_lib/wheel_controller.hpp"

namespace diff_drive_lib {

static constexpr float PI = 3.141592653F;

WheelController::WheelController(const WheelConfiguration& wheel_conf)
    : motor(wheel_conf.motor),
      op_mode_(wheel_conf.op_mode),
      encoder_buffer_(wheel_conf.velocity_rolling_window_size) {}

void WheelController::init(const WheelParams& params) {
  updateParams(params);
  motor.init();
  motor.resetEncoderCnt();
}

void WheelController::updateParams(const WheelParams& params) {
  pid_reg_.setCoeffs(params.wheel_pid_p, params.wheel_pid_i,
                     params.wheel_pid_d);
  pid_reg_.setRange(std::min(1000.0F, params.wheel_pwm_duty_limit * 10.0F));
  params_ = params;
}

void WheelController::update(const uint32_t dt_ms) {
  int32_t ticks_prev = ticks_now_;
  ticks_now_ = motor.getEncoderCnt() - ticks_offset_;

  int32_t new_ticks = ticks_now_ - ticks_prev;

  if (params_.wheel_encoder_jump_detection_enabled) {
    float ins_vel = static_cast<float>(std::abs(new_ticks)) / (dt_ms * 0.001F);
    if (ins_vel > params_.wheel_encoder_jump_threshold) {
      ticks_offset_ += new_ticks;
      ticks_now_ -= new_ticks;
      new_ticks = 0;
    }
  }

  std::pair<int32_t, uint32_t> encoder_old =
      encoder_buffer_.push_back(std::pair<int32_t, uint32_t>(new_ticks, dt_ms));

  ticks_sum_ += new_ticks;
  dt_sum_ += dt_ms;

  ticks_sum_ -= encoder_old.first;
  dt_sum_ -= encoder_old.second;

  v_now_ = static_cast<float>(ticks_sum_) / (dt_sum_ * 0.001F);

  float target_pwm_duty = 0.0F;
  if (enabled_) {
    if (op_mode_ == WheelOperationMode::VELOCITY) {
      if (v_now_ == 0.0F && v_target_ == 0.0F) {
        pid_reg_.reset();
        target_pwm_duty = 0.0F;
      } else {
        float v_err = v_target_ - v_now_;
        target_pwm_duty = pid_reg_.update(v_err, dt_ms) / 10.0F;
      }
    } else if (op_mode_ == WheelOperationMode::POSITION) {
      float ticks_err = ticks_target_ - ticks_now_;
      target_pwm_duty = pid_reg_.update(ticks_err, dt_ms) / 10.0F;
    }

    float pwm_duty;
    float current_pwm_duty = motor.getPWMDutyCycle();
    float max_pwm_change =
        params_.wheel_pwm_duty_ramp * static_cast<float>(dt_ms);
    if (target_pwm_duty > current_pwm_duty) {
      pwm_duty = std::min(target_pwm_duty, current_pwm_duty + max_pwm_change);
    } else {
      pwm_duty = std::max(target_pwm_duty, current_pwm_duty - max_pwm_change);
    }

    motor.setPWMDutyCycle(pwm_duty);
  }
}

void WheelController::setTargetVelocity(const float speed) {
  v_target_ = (speed / (2.0F * PI)) * params_.wheel_encoder_resolution;
}

void WheelController::setTargetPosition(const float position) {
  ticks_target_ = (position / (2.0F * PI)) * params_.wheel_encoder_resolution;
}

float WheelController::getVelocity() {
  return (v_now_ / params_.wheel_encoder_resolution) * (2.0F * PI);
}

float WheelController::getTorque() {
  return motor.getWindingCurrent() * params_.wheel_torque_constant;
}

float WheelController::getDistance() {
  return (ticks_now_ / params_.wheel_encoder_resolution) * (2.0F * PI);
}

void WheelController::resetDistance(float position = 0.0F) {
  motor.resetEncoderCnt();
  ticks_now_ = 0;
  ticks_offset_ = -(position / (2.0F * PI)) * params_.wheel_encoder_resolution;
}

void WheelController::enable() {
  if (!enabled_) {
    pid_reg_.reset();
    enabled_ = true;
  }
}

void WheelController::disable() {
  if (enabled_) {
    enabled_ = false;
    motor.setPWMDutyCycle(0.0F);
  }
}

}  // namespace diff_drive_lib
