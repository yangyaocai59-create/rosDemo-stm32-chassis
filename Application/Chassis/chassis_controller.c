/**
 * @file chassis_controller.c
 * @brief Supervised four-wheel differential chassis velocity controller.
 */

#include "chassis_controller.h"

#include <stddef.h>

#include "FreeRTOS.h"
#include "chassis_config.h"
#include "emergency_stop.h"
#include "task.h"
#include "wheel_speed_control_config.h"
#include "wheel_speed_estimator.h"

#define CHASSIS_CONTROLLER_COMMAND_TIMEOUT_MS 250U
#define CHASSIS_CONTROLLER_MAX_STEP_MS        100U
#define CHASSIS_CONTROLLER_OVERSPEED_CPS      2200.0f
#define CHASSIS_CONTROLLER_PIVOT_LINEAR_MAX_M_S 0.001f
#define CHASSIS_CONTROLLER_PIVOT_ANGULAR_MIN_RAD_S 0.010f

static const EncoderWheel s_encoder_wheels[MOTOR_DRIVER_WHEEL_COUNT] = {
  [MOTOR_DRIVER_WHEEL_LF] = ENCODER_WHEEL_LF,
  [MOTOR_DRIVER_WHEEL_LR] = ENCODER_WHEEL_LR,
  [MOTOR_DRIVER_WHEEL_RF] = ENCODER_WHEEL_RF,
  [MOTOR_DRIVER_WHEEL_RR] = ENCODER_WHEEL_RR
};

static ChassisControllerStatus s_status;

static bool ChassisController_TimeReached(uint32_t now_ms,
                                          uint32_t deadline_ms)
{
  return ((int32_t)(now_ms - deadline_ms) >= 0);
}

static float ChassisController_Abs(float value)
{
  return (value < 0.0f) ? -value : value;
}

static float ChassisController_GetFeedforwardBias(
  const ChassisVelocityCommand *command,
  MotorDriverWheel wheel,
  float target_counts_per_second,
  float measured_counts_per_second)
{
  const bool is_pivot =
    (command != NULL) &&
    (ChassisController_Abs(command->linear_x_m_s) <=
     CHASSIS_CONTROLLER_PIVOT_LINEAR_MAX_M_S) &&
    (ChassisController_Abs(command->angular_z_rad_s) >=
     CHASSIS_CONTROLLER_PIVOT_ANGULAR_MIN_RAD_S);

  if (!is_pivot)
  {
    return 0.0f;
  }

  (void)wheel;
  if ((ChassisController_Abs(target_counts_per_second) >=
       WHEEL_SPEED_CONTROL_PIVOT_STATIONARY_CPS) &&
      (ChassisController_Abs(measured_counts_per_second) <=
       WHEEL_SPEED_CONTROL_PIVOT_STATIONARY_CPS))
  {
    return (target_counts_per_second > 0.0f) ?
      WHEEL_SPEED_CONTROL_PIVOT_BREAKAWAY_PERMILLE :
      -WHEEL_SPEED_CONTROL_PIVOT_BREAKAWAY_PERMILLE;
  }
  return 0.0f;
}

static void ChassisController_ResetWheelControllers(void)
{
  for (uint32_t wheel = 0U; wheel < MOTOR_DRIVER_WHEEL_COUNT; ++wheel)
  {
    (void)WheelSpeedController_Reset((MotorDriverWheel)wheel);
  }
}

static bool ChassisController_GetSpeeds(
  float measured_counts_per_second[MOTOR_DRIVER_WHEEL_COUNT])
{
  WheelSpeedState speeds[ENCODER_WHEEL_COUNT];

  if ((measured_counts_per_second == NULL) ||
      (!WheelSpeedEstimator_GetSnapshot(speeds)))
  {
    return false;
  }

  for (uint32_t wheel = 0U; wheel < MOTOR_DRIVER_WHEEL_COUNT; ++wheel)
  {
    const WheelSpeedState *speed = &speeds[s_encoder_wheels[wheel]];

    if (!speed->valid)
    {
      return false;
    }
    measured_counts_per_second[wheel] =
      speed->filtered_counts_per_second;
  }
  return true;
}

static bool ChassisController_HasNonzeroTarget(
  const ChassisWheelTargets *targets)
{
  for (uint32_t wheel = 0U; wheel < MOTOR_DRIVER_WHEEL_COUNT; ++wheel)
  {
    if (ChassisController_Abs(targets->counts_per_second[wheel]) >=
        WHEEL_SPEED_CONTROL_ZERO_TARGET_CPS)
    {
      return true;
    }
  }
  return false;
}

static void ChassisController_ClearRuntimeState(void)
{
  s_status.active = false;
  s_status.command.linear_x_m_s = 0.0f;
  s_status.command.angular_z_rad_s = 0.0f;
  s_status.targets.left_linear_m_s = 0.0f;
  s_status.targets.right_linear_m_s = 0.0f;
  s_status.targets.scale = 1.0f;
  s_status.limited_mask = 0U;
  s_status.last_command_ms = 0U;
  s_status.last_control_ms = 0U;
  for (uint32_t wheel = 0U; wheel < MOTOR_DRIVER_WHEEL_COUNT; ++wheel)
  {
    s_status.targets.counts_per_second[wheel] = 0.0f;
    s_status.measured_counts_per_second[wheel] = 0.0f;
    s_status.output_permille[wheel] = 0;
  }
}

static void ChassisController_FaultLocked(ChassisControllerFault fault)
{
  MotorDriver_Disable();
  ChassisController_ResetWheelControllers();
  s_status.active = false;
  s_status.fault = fault;
  s_status.limited_mask = 0U;
  for (uint32_t wheel = 0U; wheel < MOTOR_DRIVER_WHEEL_COUNT; ++wheel)
  {
    s_status.output_permille[wheel] = 0;
  }
}

void ChassisController_Init(void)
{
  taskENTER_CRITICAL();
  MotorDriver_Disable();
  ChassisController_ResetWheelControllers();
  ChassisController_ClearRuntimeState();
  s_status.fault = CHASSIS_CONTROLLER_FAULT_NONE;
  taskEXIT_CRITICAL();
}

static bool ChassisController_StartInternal(
  const ChassisVelocityCommand *command,
  uint32_t now_ms,
  bool adopt_active_outputs)
{
  ChassisWheelTargets targets;
  float measured[MOTOR_DRIVER_WHEEL_COUNT];
  WheelSpeedControllerState control;
  MotorDriverStatus motor_status = { 0 };
  int16_t outputs[MOTOR_DRIVER_WHEEL_COUNT];
  uint32_t limited_mask = 0U;
  bool started = false;
  bool already_active;

  taskENTER_CRITICAL();
  already_active = s_status.active;
  taskEXIT_CRITICAL();
  if (already_active)
  {
    return false;
  }

  if (!EmergencyStop_CanEnableMotor())
  {
    taskENTER_CRITICAL();
    s_status.fault = CHASSIS_CONTROLLER_FAULT_ESTOP;
    taskEXIT_CRITICAL();
    return false;
  }

  if (adopt_active_outputs &&
      ((!MotorDriver_GetStatus(&motor_status)) ||
       (!motor_status.initialized) ||
       (!motor_status.standby_enabled)))
  {
    taskENTER_CRITICAL();
    s_status.fault = CHASSIS_CONTROLLER_FAULT_OUTPUT;
    taskEXIT_CRITICAL();
    return false;
  }

  if ((command == NULL) ||
      (!ChassisKinematics_ToWheelTargets(
        command,
        WHEEL_SPEED_CONTROL_TARGET_LIMIT_CPS,
        &targets)) ||
      (!ChassisController_HasNonzeroTarget(&targets)))
  {
    taskENTER_CRITICAL();
    s_status.fault = CHASSIS_CONTROLLER_FAULT_INVALID_COMMAND;
    taskEXIT_CRITICAL();
    return false;
  }

  if (!ChassisController_GetSpeeds(measured))
  {
    taskENTER_CRITICAL();
    s_status.fault = CHASSIS_CONTROLLER_FAULT_SPEED_SNAPSHOT;
    taskEXIT_CRITICAL();
    return false;
  }

  ChassisController_ResetWheelControllers();
  for (uint32_t wheel = 0U; wheel < MOTOR_DRIVER_WHEEL_COUNT; ++wheel)
  {
    if (adopt_active_outputs &&
        ((((targets.counts_per_second[wheel] > 0.0f) &&
           (motor_status.command_permille[wheel] <= 0))) ||
         ((targets.counts_per_second[wheel] < 0.0f) &&
          (motor_status.command_permille[wheel] >= 0)) ||
         (!WheelSpeedController_PrimeOutput(
           (MotorDriverWheel)wheel,
           motor_status.command_permille[wheel]))))
    {
      taskENTER_CRITICAL();
      ChassisController_FaultLocked(CHASSIS_CONTROLLER_FAULT_OUTPUT);
      taskEXIT_CRITICAL();
      return false;
    }
    if ((!WheelSpeedController_UpdateWithFeedforwardBias(
          (MotorDriverWheel)wheel,
          targets.counts_per_second[wheel],
          measured[wheel],
          ChassisController_GetFeedforwardBias(
            command,
            (MotorDriverWheel)wheel,
            targets.counts_per_second[wheel],
            measured[wheel]),
          CHASSIS_SPEED_SAMPLE_PERIOD_MS,
          &outputs[wheel])) ||
        (!WheelSpeedController_GetState((MotorDriverWheel)wheel, &control)))
    {
      taskENTER_CRITICAL();
      ChassisController_FaultLocked(CHASSIS_CONTROLLER_FAULT_CONTROL_UPDATE);
      taskEXIT_CRITICAL();
      return false;
    }
    if (control.limited)
    {
      limited_mask |= (1UL << wheel);
    }
  }

  taskENTER_CRITICAL();
  if (!s_status.active && EmergencyStop_CanEnableMotor())
  {
    started = adopt_active_outputs ? true : MotorDriver_Enable();
    for (uint32_t wheel = 0U;
         started && (wheel < MOTOR_DRIVER_WHEEL_COUNT);
         ++wheel)
    {
      started = MotorDriver_SetCommand((MotorDriverWheel)wheel,
                                       outputs[wheel]);
    }

    if (started)
    {
      (void)MotorDriver_GetStatus(&motor_status);
      for (uint32_t wheel = 0U; wheel < MOTOR_DRIVER_WHEEL_COUNT; ++wheel)
      {
        if (outputs[wheel] != motor_status.command_permille[wheel])
        {
          outputs[wheel] = motor_status.command_permille[wheel];
          (void)WheelSpeedController_PrimeOutput((MotorDriverWheel)wheel,
                                                outputs[wheel]);
          limited_mask |= 1UL << wheel;
        }
      }
      s_status.active = true;
      s_status.fault = CHASSIS_CONTROLLER_FAULT_NONE;
      s_status.command = *command;
      s_status.targets = targets;
      s_status.limited_mask = limited_mask;
      s_status.last_command_ms = now_ms;
      s_status.last_control_ms = now_ms;
      for (uint32_t wheel = 0U; wheel < MOTOR_DRIVER_WHEEL_COUNT; ++wheel)
      {
        s_status.measured_counts_per_second[wheel] = measured[wheel];
        s_status.output_permille[wheel] = outputs[wheel];
      }
    }
    else
    {
      ChassisController_FaultLocked(
        EmergencyStop_IsLatched() ? CHASSIS_CONTROLLER_FAULT_ESTOP :
                                    CHASSIS_CONTROLLER_FAULT_OUTPUT);
    }
  }
  taskEXIT_CRITICAL();
  return started;
}

bool ChassisController_Start(const ChassisVelocityCommand *command,
                             uint32_t now_ms)
{
  return ChassisController_StartInternal(command, now_ms, false);
}

bool ChassisController_StartFromActiveOutputs(
  const ChassisVelocityCommand *command,
  uint32_t now_ms)
{
  return ChassisController_StartInternal(command, now_ms, true);
}

bool ChassisController_SetCommand(const ChassisVelocityCommand *command,
                                  uint32_t now_ms)
{
  ChassisWheelTargets targets;
  bool accepted = false;

  if ((command == NULL) ||
      (!ChassisKinematics_ToWheelTargets(
        command,
        WHEEL_SPEED_CONTROL_TARGET_LIMIT_CPS,
        &targets)))
  {
    return false;
  }

  taskENTER_CRITICAL();
  if (s_status.active &&
      (s_status.fault == CHASSIS_CONTROLLER_FAULT_NONE))
  {
    s_status.command = *command;
    s_status.targets = targets;
    s_status.last_command_ms = now_ms;
    accepted = true;
  }
  taskEXIT_CRITICAL();
  return accepted;
}

void ChassisController_Process(uint32_t now_ms)
{
  ChassisVelocityCommand command;
  ChassisWheelTargets targets;
  float measured[MOTOR_DRIVER_WHEEL_COUNT];
  WheelSpeedControllerState controls[MOTOR_DRIVER_WHEEL_COUNT];
  int16_t outputs[MOTOR_DRIVER_WHEEL_COUNT];
  uint32_t limited_mask = 0U;
  uint32_t elapsed_ms;

  taskENTER_CRITICAL();
  if (!s_status.active)
  {
    taskEXIT_CRITICAL();
    return;
  }
  command = s_status.command;
  targets = s_status.targets;
  elapsed_ms = now_ms - s_status.last_control_ms;
  taskEXIT_CRITICAL();

  if ((elapsed_ms == 0U) || (elapsed_ms > CHASSIS_CONTROLLER_MAX_STEP_MS))
  {
    if (elapsed_ms > CHASSIS_CONTROLLER_MAX_STEP_MS)
    {
      taskENTER_CRITICAL();
      if (s_status.active)
      {
        ChassisController_FaultLocked(
          CHASSIS_CONTROLLER_FAULT_CONTROL_TIMING);
      }
      taskEXIT_CRITICAL();
    }
    return;
  }

  if (!ChassisController_GetSpeeds(measured))
  {
    taskENTER_CRITICAL();
    if (s_status.active)
    {
      ChassisController_FaultLocked(
        CHASSIS_CONTROLLER_FAULT_SPEED_SNAPSHOT);
    }
    taskEXIT_CRITICAL();
    return;
  }

  for (uint32_t wheel = 0U; wheel < MOTOR_DRIVER_WHEEL_COUNT; ++wheel)
  {
    if (ChassisController_Abs(measured[wheel]) >
        CHASSIS_CONTROLLER_OVERSPEED_CPS)
    {
      taskENTER_CRITICAL();
      if (s_status.active)
      {
        ChassisController_FaultLocked(CHASSIS_CONTROLLER_FAULT_OVERSPEED);
      }
      taskEXIT_CRITICAL();
      return;
    }

    if ((!WheelSpeedController_UpdateWithFeedforwardBias(
          (MotorDriverWheel)wheel,
          targets.counts_per_second[wheel],
          measured[wheel],
          ChassisController_GetFeedforwardBias(
            &command,
            (MotorDriverWheel)wheel,
            targets.counts_per_second[wheel],
            measured[wheel]),
          elapsed_ms,
          &outputs[wheel])) ||
        (!WheelSpeedController_GetState((MotorDriverWheel)wheel,
                                        &controls[wheel])))
    {
      taskENTER_CRITICAL();
      if (s_status.active)
      {
        ChassisController_FaultLocked(
          CHASSIS_CONTROLLER_FAULT_CONTROL_UPDATE);
      }
      taskEXIT_CRITICAL();
      return;
    }
    if (controls[wheel].limited)
    {
      limited_mask |= (1UL << wheel);
    }
  }

  taskENTER_CRITICAL();
  if (s_status.active)
  {
    bool outputs_ok = true;

    for (uint32_t wheel = 0U;
         outputs_ok && (wheel < MOTOR_DRIVER_WHEEL_COUNT);
         ++wheel)
    {
      outputs_ok = MotorDriver_SetCommand((MotorDriverWheel)wheel,
                                          outputs[wheel]);
    }

    if (!outputs_ok)
    {
      ChassisController_FaultLocked(
        EmergencyStop_IsLatched() ? CHASSIS_CONTROLLER_FAULT_ESTOP :
                                    CHASSIS_CONTROLLER_FAULT_OUTPUT);
    }
    else
    {
      MotorDriverStatus applied;
      (void)MotorDriver_GetStatus(&applied);
      for (uint32_t wheel = 0U; wheel < MOTOR_DRIVER_WHEEL_COUNT; ++wheel)
      {
        if (outputs[wheel] != applied.command_permille[wheel])
        {
          outputs[wheel] = applied.command_permille[wheel];
          (void)WheelSpeedController_PrimeOutput((MotorDriverWheel)wheel,
                                                outputs[wheel]);
          limited_mask |= 1UL << wheel;
        }
      }
      s_status.limited_mask = limited_mask;
      s_status.last_control_ms = now_ms;
      for (uint32_t wheel = 0U; wheel < MOTOR_DRIVER_WHEEL_COUNT; ++wheel)
      {
        s_status.measured_counts_per_second[wheel] = measured[wheel];
        s_status.output_permille[wheel] = outputs[wheel];
      }
    }
  }
  taskEXIT_CRITICAL();
}

void ChassisController_SafetyProcess(uint32_t now_ms)
{
  taskENTER_CRITICAL();
  if (s_status.active && EmergencyStop_IsLatched())
  {
    ChassisController_FaultLocked(CHASSIS_CONTROLLER_FAULT_ESTOP);
  }
  else if (s_status.active &&
           ChassisController_TimeReached(
             now_ms,
             s_status.last_command_ms +
               CHASSIS_CONTROLLER_COMMAND_TIMEOUT_MS))
  {
    ChassisController_FaultLocked(
      CHASSIS_CONTROLLER_FAULT_COMMAND_TIMEOUT);
  }
  taskEXIT_CRITICAL();
}

void ChassisController_Stop(void)
{
  taskENTER_CRITICAL();
  MotorDriver_Disable();
  ChassisController_ResetWheelControllers();
  s_status.active = false;
  s_status.limited_mask = 0U;
  for (uint32_t wheel = 0U; wheel < MOTOR_DRIVER_WHEEL_COUNT; ++wheel)
  {
    s_status.output_permille[wheel] = 0;
  }
  taskEXIT_CRITICAL();
}

bool ChassisController_IsActive(void)
{
  return s_status.active;
}

bool ChassisController_GetStatus(ChassisControllerStatus *status)
{
  if (status == NULL)
  {
    return false;
  }

  taskENTER_CRITICAL();
  *status = s_status;
  taskEXIT_CRITICAL();
  return true;
}

bool ChassisController_GetSafetyState(ChassisControllerSafetyState *state)
{
  if (state == NULL) { return false; }
  taskENTER_CRITICAL();
  state->active = s_status.active;
  state->fault = s_status.fault;
  taskEXIT_CRITICAL();
  return true;
}

const char *ChassisController_GetFaultName(ChassisControllerFault fault)
{
  switch (fault)
  {
    case CHASSIS_CONTROLLER_FAULT_NONE:
      return "none";
    case CHASSIS_CONTROLLER_FAULT_INVALID_COMMAND:
      return "invalid command";
    case CHASSIS_CONTROLLER_FAULT_ESTOP:
      return "ESTOP latched";
    case CHASSIS_CONTROLLER_FAULT_SPEED_SNAPSHOT:
      return "speed snapshot failure";
    case CHASSIS_CONTROLLER_FAULT_CONTROL_UPDATE:
      return "wheel controller update failure";
    case CHASSIS_CONTROLLER_FAULT_OUTPUT:
      return "motor output failure";
    case CHASSIS_CONTROLLER_FAULT_OVERSPEED:
      return "wheel overspeed";
    case CHASSIS_CONTROLLER_FAULT_COMMAND_TIMEOUT:
      return "velocity command timeout";
    case CHASSIS_CONTROLLER_FAULT_CONTROL_TIMING:
      return "control task timing violation";
    default:
      return "unknown";
  }
}
