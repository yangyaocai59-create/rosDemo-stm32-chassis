/**
 * @file chassis_controller.c
 * @brief Supervised four-wheel differential chassis velocity controller.
 */

#include "chassis_controller.h"

#include <stddef.h>
#include <stdint.h>
#include <math.h>

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
/* Stop/reset/start invalidates every in-flight wheel update. */
static uint32_t s_control_generation;
#if CHASSIS_CONTROL_V2
static uint32_t s_command_revision;
static ChassisRunMetrics s_run_metrics;
bool ChassisController_GetRunMetrics(ChassisRunMetrics *metrics)
{
  if (metrics==NULL) return false;
  taskENTER_CRITICAL(); *metrics=s_run_metrics; taskEXIT_CRITICAL();
  return true;
}
#endif
static bool ChassisController_TimeReached(uint32_t now_ms,
                                          uint32_t deadline_ms)
{
  return ((int32_t)(now_ms - deadline_ms) >= 0);
}

static float ChassisController_Abs(float value)
{
  return (value < 0.0f) ? -value : value;
}

static bool ChassisController_IsPivot(
  const ChassisVelocityCommand *command)
{
  return
    (command != NULL) &&
    (ChassisController_Abs(command->linear_x_m_s) <=
     CHASSIS_CONTROLLER_PIVOT_LINEAR_MAX_M_S) &&
    (ChassisController_Abs(command->angular_z_rad_s) >=
     CHASSIS_CONTROLLER_PIVOT_ANGULAR_MIN_RAD_S);
}

static WheelSpeedControlMode ChassisController_GetWheelControlMode(
  const ChassisVelocityCommand *command)
{
#if CHASSIS_CONTROL_V2
  if (command != NULL && s_status.pivot_mask != 0U &&
      fabsf(command->linear_x_m_s) < CHASSIS_PIVOT_EXIT_LINEAR_M_S &&
      fabsf(command->angular_z_rad_s) > CHASSIS_PIVOT_EXIT_ANGULAR_RAD_S)
    return WHEEL_SPEED_CONTROL_MODE_PIVOT;
#endif
  return ChassisController_IsPivot(command) ?
    WHEEL_SPEED_CONTROL_MODE_PIVOT : WHEEL_SPEED_CONTROL_MODE_NORMAL;
}

static void ChassisController_ResetWheelControllers(void)
{
  ++s_control_generation;
  s_status.motion_generation = s_control_generation;
  s_status.pivot_mask = 0U;
  for (uint32_t wheel = 0U; wheel < MOTOR_DRIVER_WHEEL_COUNT; ++wheel)
  {
    (void)WheelSpeedController_Reset((MotorDriverWheel)wheel);
  }
}

static bool ChassisController_GetSpeeds(
  uint32_t now_ms, float measured_counts_per_second[MOTOR_DRIVER_WHEEL_COUNT],
  uint32_t *sample_time_ms, uint32_t *sample_sequence,
  uint32_t *zero_delta_mask,uint32_t *sample_dt)
{
  WheelSpeedState speeds[ENCODER_WHEEL_COUNT];

  if ((measured_counts_per_second == NULL) ||
      (!WheelSpeedEstimator_GetSnapshot(speeds)))
  {
    return false;
  }
  uint32_t zero_mask=0U;

  for (uint32_t wheel = 0U; wheel < MOTOR_DRIVER_WHEEL_COUNT; ++wheel)
  {
    const WheelSpeedState *speed = &speeds[s_encoder_wheels[wheel]];

    if (!speed->valid)
    {
      return false;
    }
#if CHASSIS_CONTROL_V2
    if (!isfinite(speed->filtered_counts_per_second) ||
        !isfinite(speed->raw_counts_per_second) ||
        (uint32_t)(now_ms-speed->sample_time_ms) > CHASSIS_FEEDBACK_MAX_AGE_MS ||
        speed->sample_period_ms == 0U ||
        speed->sample_period_ms > CHASSIS_FEEDBACK_MAX_INTERVAL_MS ||
        speed->quality_flags != 0U ||
        speed->sample_time_ms != speeds[0].sample_time_ms ||
        speed->sample_period_ms != speeds[0].sample_period_ms ||
        speed->sample_sequence != speeds[0].sample_sequence)
      return false;
#else
    (void)now_ms;
#endif
    measured_counts_per_second[wheel] =
      speed->filtered_counts_per_second;
    if (speed->delta_counts==0) zero_mask|=1UL<<wheel;
  }
  if (sample_time_ms != NULL) *sample_time_ms = speeds[0].sample_time_ms;
  if (sample_sequence != NULL) *sample_sequence = speeds[0].sample_sequence;
  if (zero_delta_mask != NULL) *zero_delta_mask=zero_mask;
  if (sample_dt != NULL) *sample_dt=speeds[0].sample_period_ms;
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

#if CHASSIS_CONTROL_V2
static bool ChassisController_ConstrainFeedforward(const ChassisVelocityCommand *command,
                                                 ChassisWheelTargets *targets)
{
  const WheelSpeedControlMode mode=ChassisController_GetWheelControlMode(command);
  float scale=1.0f;
  for (uint32_t w=0U;w<MOTOR_DRIVER_WHEEL_COUNT;++w) {
    const float target=targets->counts_per_second[w];
    if (fabsf(target)<WHEEL_SPEED_CONTROL_ZERO_TARGET_CPS) continue;
    const float limit=WheelSpeedController_GetFeedforwardTargetLimit((MotorDriverWheel)w,target>0,mode);
    if (!isfinite(limit) || limit<=0) return false;
    scale=fminf(scale,limit/fabsf(target));
  }
  if (scale<1.0f) {
    for (uint32_t w=0U;w<MOTOR_DRIVER_WHEEL_COUNT;++w) targets->counts_per_second[w]*=scale;
    targets->left_linear_m_s*=scale;
    targets->right_linear_m_s*=scale;
    targets->scale*=scale;
  }
  return true;
}
#endif

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
  s_status.feedback_time_ms = 0U;
  s_status.feedback_sequence = 0U;
  for (uint32_t wheel = 0U; wheel < MOTOR_DRIVER_WHEEL_COUNT; ++wheel)
  {
    s_status.targets.counts_per_second[wheel] = 0.0f;
    s_status.measured_counts_per_second[wheel] = 0.0f;
    s_status.output_permille[wheel] = 0;
    s_status.output_limit_reasons[wheel] = 0U;
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

#if CHASSIS_CONTROL_REVISION >= 201
static bool ChassisController_RejectStart(uint32_t generation,ChassisControllerFault fault)
{
  taskENTER_CRITICAL();
  if (!s_status.active && s_control_generation==generation)
    ChassisController_FaultLocked(fault);
  taskEXIT_CRITICAL();
  return false;
}

static bool ChassisController_StartTransactional(const ChassisVelocityCommand *command,
                                                uint32_t now_ms,bool adopt)
{
  WheelSpeedTransaction transaction;
  uint32_t generation;
  taskENTER_CRITICAL();
  if (s_status.active) { taskEXIT_CRITICAL(); return false; }
  generation=s_control_generation;
  (void)WheelSpeedController_Begin(&transaction);
  taskEXIT_CRITICAL();
  if (!EmergencyStop_CanEnableMotor())
    return ChassisController_RejectStart(generation,CHASSIS_CONTROLLER_FAULT_ESTOP);
  ChassisWheelTargets targets;
  if (command==NULL || !ChassisKinematics_ToWheelTargets(command,
        WHEEL_SPEED_CONTROL_TARGET_LIMIT_CPS,&targets) ||
      !ChassisController_HasNonzeroTarget(&targets) ||
      !ChassisController_ConstrainFeedforward(command,&targets))
    return ChassisController_RejectStart(generation,CHASSIS_CONTROLLER_FAULT_INVALID_COMMAND);
  float measured[MOTOR_DRIVER_WHEEL_COUNT];
  uint32_t feedback_time,feedback_sequence,zero_mask,sample_dt;
  if (!ChassisController_GetSpeeds(now_ms,measured,&feedback_time,&feedback_sequence,&zero_mask,&sample_dt))
    return ChassisController_RejectStart(generation,CHASSIS_CONTROLLER_FAULT_SPEED_SNAPSHOT);
  MotorDriverStatus motor;
  if (adopt && (!MotorDriver_GetStatus(&motor) || !motor.initialized || !motor.standby_enabled))
    return ChassisController_RejectStart(generation,CHASSIS_CONTROLLER_FAULT_OUTPUT);
  const WheelSpeedControlMode mode=ChassisController_GetWheelControlMode(command);
  ChassisRunMetrics metrics={.started_ms=now_ms,.last_sample_ms=now_ms,.samples=1};
  int16_t outputs[MOTOR_DRIVER_WHEEL_COUNT];
  uint32_t limited_mask=targets.scale<1.0f ? (1UL<<MOTOR_DRIVER_WHEEL_COUNT)-1U : 0U;
  for (uint32_t w=0;w<MOTOR_DRIVER_WHEEL_COUNT;++w) {
    transaction.memory[w]=(WheelControlMemory){0};
    transaction.state[w]=(WheelSpeedControllerState){0};
    if (fabsf(measured[w])>CHASSIS_CONTROLLER_OVERSPEED_CPS)
      return ChassisController_RejectStart(generation,CHASSIS_CONTROLLER_FAULT_OVERSPEED);
    if (adopt) {
      const int16_t applied=motor.command_permille[w];
      const float target=targets.counts_per_second[w];
      if ((target>0 && applied<=0) || (target<0 && applied>=0) ||
          applied>WHEEL_SPEED_CONTROL_OUTPUT_LIMIT_PERMILLE ||
          applied<-WHEEL_SPEED_CONTROL_OUTPUT_LIMIT_PERMILLE)
        return ChassisController_RejectStart(generation,CHASSIS_CONTROLLER_FAULT_OUTPUT);
      transaction.memory[w].output=(float)applied;
    }
    if (!WheelSpeedController_Step(&transaction,(MotorDriverWheel)w,targets.counts_per_second[w],
                                  measured[w],0,mode,CHASSIS_SPEED_SAMPLE_PERIOD_MS,&outputs[w]))
      return ChassisController_RejectStart(generation,CHASSIS_CONTROLLER_FAULT_CONTROL_UPDATE);
    const WheelSpeedControllerState *control=&transaction.state[w];
    if (control->limited) limited_mask|=1UL<<w;
    ControlRunMetrics_Add(&metrics.wheel[w],control->error_counts_per_second,
      control->integrator_permille,control->limit_reasons,
      (zero_mask&(1UL<<w))!=0U && fabsf(control->target_counts_per_second)>=1,sample_dt);
  }
  taskENTER_CRITICAL();
  /* Stop during validation or PI calculation must not be followed by a new
   * enable, a stale controller-state commit, or an obsolete fault write. */
  if (s_status.active || s_control_generation!=generation) { taskEXIT_CRITICAL(); return false; }
  if (!EmergencyStop_CanEnableMotor()) {
    ChassisController_FaultLocked(CHASSIS_CONTROLLER_FAULT_ESTOP);
    taskEXIT_CRITICAL();return false;
  }
  if (!WheelSpeedController_Commit(&transaction)) { taskEXIT_CRITICAL();return false; }
  if ((adopt && (!MotorDriver_GetStatus(&motor) || !motor.standby_enabled)) ||
      (!adopt && !MotorDriver_Enable())) {
    ChassisController_FaultLocked(CHASSIS_CONTROLLER_FAULT_OUTPUT);
    taskEXIT_CRITICAL();return false;
  }
  for (uint32_t w=0;w<MOTOR_DRIVER_WHEEL_COUNT;++w) {
    if (!MotorDriver_SetCommand((MotorDriverWheel)w,outputs[w])) {
      ChassisController_FaultLocked(EmergencyStop_IsLatched() ?
        CHASSIS_CONTROLLER_FAULT_ESTOP:CHASSIS_CONTROLLER_FAULT_OUTPUT);
      taskEXIT_CRITICAL();return false;
    }
  }
  if (!MotorDriver_GetStatus(&motor)) {
    ChassisController_FaultLocked(CHASSIS_CONTROLLER_FAULT_OUTPUT);
    taskEXIT_CRITICAL();return false;
  }
  ++s_control_generation;
  s_status.motion_generation=s_control_generation;
  s_status.active=true;
  s_status.fault=CHASSIS_CONTROLLER_FAULT_NONE;
  s_status.command=*command;
  s_status.targets=targets;
  s_status.last_command_ms=s_status.last_control_ms=now_ms;
  s_status.feedback_time_ms=feedback_time;
  s_status.feedback_sequence=feedback_sequence;
  s_status.pivot_mask=0;
  for (uint32_t w=0;w<MOTOR_DRIVER_WHEEL_COUNT;++w) {
    uint32_t reasons=transaction.state[w].limit_reasons|(targets.scale<1.0f?WHEEL_LIMIT_TARGET:0U);
    if (outputs[w]!=motor.command_permille[w]) {
      (void)WheelSpeedController_PrimeOutput((MotorDriverWheel)w,motor.command_permille[w]);
      reasons|=WHEEL_LIMIT_ACTUATOR;limited_mask|=1UL<<w;
      ++metrics.wheel[w].actuator_limited_samples;
    }
    if (WheelSpeedController_IsPivotMode((MotorDriverWheel)w)) s_status.pivot_mask|=1UL<<w;
    s_status.measured_counts_per_second[w]=measured[w];
    s_status.output_permille[w]=motor.command_permille[w];
    s_status.output_limit_reasons[w]=reasons;
  }
  s_status.limited_mask=limited_mask;
  metrics.generation=s_control_generation;
  s_run_metrics=metrics;
  taskEXIT_CRITICAL();return true;
}
#endif

static bool ChassisController_StartInternal(
  const ChassisVelocityCommand *command,
  uint32_t now_ms,
  bool adopt_active_outputs)
{
#if CHASSIS_CONTROL_REVISION >= 201
  return ChassisController_StartTransactional(command,now_ms,adopt_active_outputs);
#else
  ChassisWheelTargets targets;
  float measured[MOTOR_DRIVER_WHEEL_COUNT];
  WheelSpeedControllerState control;
  MotorDriverStatus motor_status = { 0 };
  int16_t outputs[MOTOR_DRIVER_WHEEL_COUNT];
  uint32_t limited_mask = 0U;
  bool started = false;
  bool already_active;
  uint32_t feedback_time_ms, feedback_sequence;
  uint32_t zero_delta_mask,sample_dt;

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

  if (!ChassisController_GetSpeeds(now_ms, measured, &feedback_time_ms, &feedback_sequence,
                                 &zero_delta_mask,&sample_dt))
  {
    taskENTER_CRITICAL();
    s_status.fault = CHASSIS_CONTROLLER_FAULT_SPEED_SNAPSHOT;
    taskEXIT_CRITICAL();
    return false;
  }
#if CHASSIS_CONTROL_V2
  if (!ChassisController_ConstrainFeedforward(command,&targets)) {
    taskENTER_CRITICAL();
    ChassisController_FaultLocked(CHASSIS_CONTROLLER_FAULT_INVALID_COMMAND);
    taskEXIT_CRITICAL(); return false;
  }
  for (uint32_t w=0U;w<MOTOR_DRIVER_WHEEL_COUNT;++w) {
    if (fabsf(measured[w])>CHASSIS_CONTROLLER_OVERSPEED_CPS) {
      taskENTER_CRITICAL();
      ChassisController_FaultLocked(CHASSIS_CONTROLLER_FAULT_OVERSPEED);
      taskEXIT_CRITICAL();
      return false;
    }
  }
  if (targets.scale<1.0f) limited_mask=(1UL<<MOTOR_DRIVER_WHEEL_COUNT)-1U;
#endif

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
    if ((!WheelSpeedController_UpdateWithMode(
          (MotorDriverWheel)wheel,
          targets.counts_per_second[wheel],
          measured[wheel],
          0.0f,
          ChassisController_GetWheelControlMode(command),
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
      s_status.feedback_time_ms = feedback_time_ms;
      s_status.feedback_sequence = feedback_sequence;
#if CHASSIS_CONTROL_V2
      s_run_metrics=(ChassisRunMetrics){.generation=s_control_generation,
        .started_ms=now_ms,.last_sample_ms=now_ms};
#endif
      for (uint32_t wheel = 0U; wheel < MOTOR_DRIVER_WHEEL_COUNT; ++wheel)
      {
        s_status.measured_counts_per_second[wheel] = measured[wheel];
        s_status.output_permille[wheel] = outputs[wheel];
#if CHASSIS_CONTROL_V2
        (void)WheelSpeedController_GetState((MotorDriverWheel)wheel,&control);
        s_status.output_limit_reasons[wheel]=control.limit_reasons |
          (targets.scale<1.0f?WHEEL_LIMIT_TARGET:0U);
        ControlRunMetrics_Add(&s_run_metrics.wheel[wheel],control.error_counts_per_second,
          control.integrator_permille,control.limit_reasons,
          (zero_delta_mask&(1UL<<wheel))!=0U && fabsf(control.target_counts_per_second)>=1,sample_dt);
#endif
        if (WheelSpeedController_IsPivotMode((MotorDriverWheel)wheel))
          s_status.pivot_mask |= 1UL << wheel;
      }
#if CHASSIS_CONTROL_V2
      s_run_metrics.samples=1U;
#endif
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
#endif
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
#if CHASSIS_CONTROL_V2
    if (!ChassisController_ConstrainFeedforward(command,&targets)) {
      taskEXIT_CRITICAL(); return false;
    }
#endif
    s_status.command = *command;
    s_status.targets = targets;
    s_status.last_command_ms = now_ms;
#if CHASSIS_CONTROL_V2
    ++s_command_revision;
#endif
    accepted = true;
  }
  taskEXIT_CRITICAL();
  return accepted;
}

void ChassisController_Process(uint32_t now_ms)
{
#if CHASSIS_CONTROL_V2
  /* One motor-task producer. Private candidates stay off its 2 KiB stack and
   * are never published unless both the motion and command revisions match. */
  static WheelSpeedTransaction transaction;
  static ChassisRunMetrics metrics;
  ChassisWheelTargets targets;
  uint32_t generation, command_revision, elapsed_ms, previous_feedback, previous_feedback_time;
  WheelSpeedControlMode mode;
  taskENTER_CRITICAL();
  if (!s_status.active) { taskEXIT_CRITICAL(); return; }
  generation=s_control_generation;
  command_revision=s_command_revision;
  targets=s_status.targets;
  elapsed_ms=now_ms-s_status.last_control_ms;
  previous_feedback=s_status.feedback_sequence;
  previous_feedback_time=s_status.feedback_time_ms;
  mode=ChassisController_GetWheelControlMode(&s_status.command);
  (void)WheelSpeedController_Begin(&transaction);
  metrics=s_run_metrics;
  taskEXIT_CRITICAL();
  if (elapsed_ms==0U) return;
  ChassisControllerFault fault=CHASSIS_CONTROLLER_FAULT_NONE;
  float measured[MOTOR_DRIVER_WHEEL_COUNT];
  int16_t outputs[MOTOR_DRIVER_WHEEL_COUNT];
  uint32_t feedback_time=0U, feedback_sequence=0U;
  uint32_t zero_delta_mask=0U,sample_dt=0U;
  if (elapsed_ms>CHASSIS_CONTROLLER_MAX_STEP_MS)
    fault=CHASSIS_CONTROLLER_FAULT_CONTROL_TIMING;
  else if (!ChassisController_GetSpeeds(now_ms,measured,&feedback_time,&feedback_sequence,
                                      &zero_delta_mask,&sample_dt) ||
           (int32_t)(feedback_sequence-previous_feedback)<=0 ||
           feedback_time==previous_feedback_time)
    fault=CHASSIS_CONTROLLER_FAULT_SPEED_SNAPSHOT;
  uint32_t limited_mask=targets.scale<1.0f ? (1UL<<MOTOR_DRIVER_WHEEL_COUNT)-1U : 0U;
  for (uint32_t w=0U; fault==CHASSIS_CONTROLLER_FAULT_NONE && w<MOTOR_DRIVER_WHEEL_COUNT; ++w) {
    if (fabsf(measured[w])>CHASSIS_CONTROLLER_OVERSPEED_CPS)
      fault=CHASSIS_CONTROLLER_FAULT_OVERSPEED;
    else if (!WheelSpeedController_Step(&transaction,(MotorDriverWheel)w,
              targets.counts_per_second[w],measured[w],0,mode,elapsed_ms,&outputs[w]))
      fault=CHASSIS_CONTROLLER_FAULT_CONTROL_UPDATE;
    else if (transaction.state[w].limited) limited_mask|=1UL<<w;
    if (fault==CHASSIS_CONTROLLER_FAULT_NONE)
      ControlRunMetrics_Add(&metrics.wheel[w],transaction.state[w].error_counts_per_second,
        transaction.state[w].integrator_permille,transaction.state[w].limit_reasons,
        (zero_delta_mask&(1UL<<w))!=0U && fabsf(targets.counts_per_second[w])>=1,sample_dt);
  }
  taskENTER_CRITICAL();
  if (!s_status.active || s_control_generation!=generation ||
      s_command_revision!=command_revision) { taskEXIT_CRITICAL(); return; }
  if (EmergencyStop_IsLatched()) fault=CHASSIS_CONTROLLER_FAULT_ESTOP;
  if (fault==CHASSIS_CONTROLLER_FAULT_NONE && !WheelSpeedController_Commit(&transaction))
    fault=CHASSIS_CONTROLLER_FAULT_CONTROL_UPDATE;
  if (fault!=CHASSIS_CONTROLLER_FAULT_NONE) {
    ChassisController_FaultLocked(fault);
    taskEXIT_CRITICAL();
    return;
  }
  for (uint32_t w=0U; w<MOTOR_DRIVER_WHEEL_COUNT; ++w) {
    if (!MotorDriver_SetCommand((MotorDriverWheel)w,outputs[w])) {
      ChassisController_FaultLocked(EmergencyStop_IsLatched() ?
        CHASSIS_CONTROLLER_FAULT_ESTOP : CHASSIS_CONTROLLER_FAULT_OUTPUT);
      taskEXIT_CRITICAL(); return;
    }
  }
  MotorDriverStatus applied;
  if (!MotorDriver_GetStatus(&applied)) {
    ChassisController_FaultLocked(CHASSIS_CONTROLLER_FAULT_OUTPUT);
    taskEXIT_CRITICAL(); return;
  }
  s_status.pivot_mask=0U;
  for (uint32_t w=0U; w<MOTOR_DRIVER_WHEEL_COUNT; ++w) {
    uint32_t reasons=transaction.state[w].limit_reasons |
      (targets.scale<1.0f?WHEEL_LIMIT_TARGET:0U);
    if (outputs[w]!=applied.command_permille[w]) {
      (void)WheelSpeedController_PrimeOutput((MotorDriverWheel)w,applied.command_permille[w]);
      reasons|=WHEEL_LIMIT_ACTUATOR;
      ++metrics.wheel[w].actuator_limited_samples;
      limited_mask|=1UL<<w;
    }
    if (WheelSpeedController_IsPivotMode((MotorDriverWheel)w)) s_status.pivot_mask|=1UL<<w;
    s_status.measured_counts_per_second[w]=measured[w];
    s_status.output_permille[w]=applied.command_permille[w];
    s_status.output_limit_reasons[w]=reasons;
  }
  s_status.limited_mask=limited_mask;
  s_status.last_control_ms=now_ms;
  s_status.feedback_time_ms=feedback_time;
  s_status.feedback_sequence=feedback_sequence;
  if (metrics.samples!=UINT32_MAX) ++metrics.samples;
  metrics.last_sample_ms=now_ms;
  s_run_metrics=metrics;
  taskEXIT_CRITICAL();
#else
  ChassisVelocityCommand command;
  ChassisWheelTargets targets;
  float measured[MOTOR_DRIVER_WHEEL_COUNT];
  WheelSpeedControllerState controls[MOTOR_DRIVER_WHEEL_COUNT];
  int16_t outputs[MOTOR_DRIVER_WHEEL_COUNT];
  uint32_t limited_mask = 0U;
  uint32_t elapsed_ms;
  uint32_t generation;

  taskENTER_CRITICAL();
  if (!s_status.active)
  {
    taskEXIT_CRITICAL();
    return;
  }
  command = s_status.command;
  targets = s_status.targets;
  elapsed_ms = now_ms - s_status.last_control_ms;
  generation = s_control_generation;
  taskEXIT_CRITICAL();

  if ((elapsed_ms == 0U) || (elapsed_ms > CHASSIS_CONTROLLER_MAX_STEP_MS))
  {
    if (elapsed_ms > CHASSIS_CONTROLLER_MAX_STEP_MS)
    {
      taskENTER_CRITICAL();
      if (s_status.active && (s_control_generation == generation))
      {
        ChassisController_FaultLocked(
          CHASSIS_CONTROLLER_FAULT_CONTROL_TIMING);
      }
      taskEXIT_CRITICAL();
    }
    return;
  }

  if (!ChassisController_GetSpeeds(now_ms, measured, NULL, NULL, NULL, NULL))
  {
    taskENTER_CRITICAL();
    if (s_status.active && (s_control_generation == generation))
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
      if (s_status.active && (s_control_generation == generation))
      {
        ChassisController_FaultLocked(CHASSIS_CONTROLLER_FAULT_OVERSPEED);
      }
      taskEXIT_CRITICAL();
      return;
    }

    /* Serialize each wheel update with task-level Stop/Reset. Priority-4
     * ESTOP still cuts the power stage directly. No I/O occurs under this lock. */
    taskENTER_CRITICAL();
    if ((!s_status.active) || (s_control_generation != generation))
    {
      taskEXIT_CRITICAL();
      return;
    }
    const bool control_ok = WheelSpeedController_UpdateWithMode(
          (MotorDriverWheel)wheel,
          targets.counts_per_second[wheel],
          measured[wheel],
          0.0f,
          ChassisController_GetWheelControlMode(&command),
          elapsed_ms,
          &outputs[wheel]) &&
        WheelSpeedController_GetState((MotorDriverWheel)wheel, &controls[wheel]);
    taskEXIT_CRITICAL();
    if (!control_ok)
    {
      taskENTER_CRITICAL();
      if (s_status.active && (s_control_generation == generation))
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
  if (s_status.active && (s_control_generation == generation))
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
#endif
}

void ChassisController_SafetyProcess(uint32_t now_ms)
{
  taskENTER_CRITICAL();
  if (s_status.active && EmergencyStop_IsLatched())
  {
    ChassisController_FaultLocked(CHASSIS_CONTROLLER_FAULT_ESTOP);
  }
#if CHASSIS_CONTROL_V2
  else if (s_status.active &&
#if CHASSIS_CONTROL_REVISION >= 202
           /* The caller's timestamp may precede a newer motor-task commit
            * after preemption. Compare modular deadlines, like command expiry,
            * rather than treating that small negative age as UINT32_MAX. */
           ChassisController_TimeReached(now_ms,s_status.feedback_time_ms+
             CHASSIS_FEEDBACK_MAX_AGE_MS+1U))
#else
           (uint32_t)(now_ms-s_status.feedback_time_ms)>CHASSIS_FEEDBACK_MAX_AGE_MS)
#endif
  {
    ChassisController_FaultLocked(CHASSIS_CONTROLLER_FAULT_SPEED_SNAPSHOT);
  }
#endif
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
