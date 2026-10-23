/** Read-only calibration interlock, checked at start and throughout collection. */
#include "imu_calibration_guard.h"
#include <stddef.h>
#include "FreeRTOS.h"
#include "task.h"
#include "stm32f4xx_hal.h"
#include "chassis_runtime.h"
#include "encoder_service.h"
#include "wheel_speed_estimator.h"
#include "motor_driver.h"
#include "emergency_stop.h"
#include "fault_manager.h"
#include "motor_open_loop_test.h"
#include "chassis_runtime_test.h"

bool ImuCalibrationGuard_Read(int64_t wheel_counts[4], bool collecting)
{
  ChassisRuntimeStatus runtime;
  MotorDriverStatus motor;
  EncoderState encoders[ENCODER_WHEEL_COUNT];
  WheelSpeedState speeds[ENCODER_WHEEL_COUNT];
  bool allowed;
  if (wheel_counts == NULL) { return false; }
  taskENTER_CRITICAL();
  allowed = ChassisRuntime_GetStatus(&runtime) &&
    MotorDriver_GetStatus(&motor) && EncoderService_GetSnapshot(encoders) &&
    WheelSpeedEstimator_GetSnapshot(speeds) &&
    (runtime.state == CHASSIS_RUNTIME_STATE_IDLE) &&
    (runtime.fault == CHASSIS_RUNTIME_FAULT_NONE) &&
    !runtime.command_valid && (!runtime.diagnostics_active || collecting) &&
    !ChassisController_IsActive() && EmergencyStop_CanEnableMotor() &&
    !FaultManager_HasCriticalFault() && !MotorOpenLoopTest_IsActive() &&
    !ChassisRuntimeTest_IsActive() && motor.initialized && !motor.standby_enabled;
  const uint32_t now_ms = HAL_GetTick();
  for (uint32_t i = 0U; allowed && i < ENCODER_WHEEL_COUNT; ++i)
  {
    allowed = (motor.command_permille[i] == 0) && speeds[i].valid &&
      speeds[i].stopped && (encoders[i].delta == 0) &&
      ((uint32_t)(now_ms - speeds[i].sample_time_ms) <= 30U);
    wheel_counts[i] = encoders[i].accumulated_count;
  }
  taskEXIT_CRITICAL();
  return allowed;
}
