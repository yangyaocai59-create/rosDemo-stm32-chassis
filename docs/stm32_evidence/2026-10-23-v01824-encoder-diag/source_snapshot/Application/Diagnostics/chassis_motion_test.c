/**
 * @file chassis_motion_test.c
 * @brief Automatic lifted-wheel straight/rotation chassis validation.
 */

#include "chassis_motion_test.h"

#if MOTOR_OPEN_LOOP_TEST_ENABLED

#include <stddef.h>

#include "FreeRTOS.h"
#include "chassis_controller.h"
#include "chassis_kinematics.h"
#include "debug_log.h"
#include "emergency_stop.h"
#include "encoder_service.h"
#include "task.h"
#include "wheel_speed_control_config.h"
#include "wheel_speed_estimator.h"

#define CHASSIS_TEST_ARM_WINDOW_MS          5000U
#define CHASSIS_TEST_COUNTDOWN_MS           3000U
#define CHASSIS_TEST_PHASE_DURATION_MS      3000U
#define CHASSIS_TEST_ANALYSIS_START_MS      1500U
#define CHASSIS_TEST_NO_MOTION_TIMEOUT_MS   1000U
#define CHASSIS_TEST_DRIVE_MARGIN_MS         500U
#define CHASSIS_TEST_STOP_GUARD_MS           300U
#define CHASSIS_TEST_STOP_TIMEOUT_MS        2500U
#define CHASSIS_TEST_OVERALL_TIMEOUT_MS    30000U
#define CHASSIS_TEST_LOG_PERIOD_MS           200U
#define CHASSIS_TEST_MOVED_THRESHOLD_COUNTS    4
#define CHASSIS_TEST_MIN_ACTIVE_TARGET_CPS    50.0f
#define CHASSIS_TEST_PHASE_COUNT                4U

typedef enum
{
  CHASSIS_TEST_STATE_DISARMED = 0,
  CHASSIS_TEST_STATE_ARMED,
  CHASSIS_TEST_STATE_COUNTDOWN,
  CHASSIS_TEST_STATE_PREPARE,
  CHASSIS_TEST_STATE_RUNNING,
  CHASSIS_TEST_STATE_WAIT_STOP,
  CHASSIS_TEST_STATE_COMPLETED,
  CHASSIS_TEST_STATE_ABORTED
} ChassisMotionTestState;

typedef enum
{
  CHASSIS_TEST_ABORT_NONE = 0,
  CHASSIS_TEST_ABORT_SNAPSHOT_FAILURE,
  CHASSIS_TEST_ABORT_KINEMATICS_FAILURE,
  CHASSIS_TEST_ABORT_CONTROLLER_FAILURE,
  CHASSIS_TEST_ABORT_WRONG_DIRECTION,
  CHASSIS_TEST_ABORT_NO_MOTION,
  CHASSIS_TEST_ABORT_STOP_TIMEOUT,
  CHASSIS_TEST_ABORT_DRIVE_TIMEOUT,
  CHASSIS_TEST_ABORT_OVERALL_TIMEOUT,
  CHASSIS_TEST_ABORT_ESTOP
} ChassisMotionTestAbortReason;

typedef struct
{
  const char *name;
  ChassisVelocityCommand command;
} ChassisMotionTestPhase;

typedef struct
{
  float sum_cps;
  float min_cps;
  float max_cps;
  uint32_t count;
  uint16_t max_command_permille;
} ChassisMotionTestStatistics;

static const ChassisMotionTestPhase s_phases[CHASSIS_TEST_PHASE_COUNT] = {
  { "FORWARD",  {  0.065f,  0.0f } },
  { "REVERSE",  { -0.065f,  0.0f } },
  { "TURN_LEFT",  { 0.0f,  0.600f } },
  { "TURN_RIGHT", { 0.0f, -0.600f } }
};

static const EncoderWheel s_encoder_wheels[MOTOR_DRIVER_WHEEL_COUNT] = {
  [MOTOR_DRIVER_WHEEL_LF] = ENCODER_WHEEL_LF,
  [MOTOR_DRIVER_WHEEL_LR] = ENCODER_WHEEL_LR,
  [MOTOR_DRIVER_WHEEL_RF] = ENCODER_WHEEL_RF,
  [MOTOR_DRIVER_WHEEL_RR] = ENCODER_WHEEL_RR
};

static const char *const s_wheel_names[MOTOR_DRIVER_WHEEL_COUNT] = {
  [MOTOR_DRIVER_WHEEL_LF] = "LF",
  [MOTOR_DRIVER_WHEEL_LR] = "LR",
  [MOTOR_DRIVER_WHEEL_RF] = "RF",
  [MOTOR_DRIVER_WHEEL_RR] = "RR"
};

static volatile ChassisMotionTestState s_state;
static volatile ChassisMotionTestAbortReason s_abort_reason;
static volatile ChassisControllerFault s_controller_fault;
static volatile MotorDriverWheel s_fault_wheel;
static volatile uint32_t s_arm_deadline_ms;
static volatile uint32_t s_drive_deadline_ms;
static volatile uint32_t s_overall_deadline_ms;
static volatile bool s_arm_expired_notice;
static uint32_t s_phase_index;
static uint32_t s_phase_start_ms;
static uint32_t s_phase_deadline_ms;
static uint32_t s_no_motion_deadline_ms;
static uint32_t s_stop_guard_deadline_ms;
static uint32_t s_stop_timeout_ms;
static uint32_t s_next_log_ms;
static int64_t s_phase_start_totals[MOTOR_DRIVER_WHEEL_COUNT];
static ChassisWheelTargets s_phase_targets;
static ChassisMotionTestStatistics s_statistics[MOTOR_DRIVER_WHEEL_COUNT];

static bool ChassisMotionTest_TimeReached(uint32_t now_ms,
                                          uint32_t deadline_ms)
{
  return ((int32_t)(now_ms - deadline_ms) >= 0);
}

static float ChassisMotionTest_AbsFloat(float value)
{
  return (value < 0.0f) ? -value : value;
}

static int64_t ChassisMotionTest_AbsCount(int64_t value)
{
  return (value < 0) ? -value : value;
}

static int32_t ChassisMotionTest_RoundFloat(float value)
{
  return (int32_t)((value >= 0.0f) ? (value + 0.5f) : (value - 0.5f));
}

static const char *ChassisMotionTest_GetAbortReasonName(
  ChassisMotionTestAbortReason reason)
{
  switch (reason)
  {
    case CHASSIS_TEST_ABORT_SNAPSHOT_FAILURE:
      return "encoder/speed snapshot failure";
    case CHASSIS_TEST_ABORT_KINEMATICS_FAILURE:
      return "kinematics conversion failure";
    case CHASSIS_TEST_ABORT_CONTROLLER_FAILURE:
      return "chassis controller failure";
    case CHASSIS_TEST_ABORT_WRONG_DIRECTION:
      return "encoder direction disagrees with wheel target";
    case CHASSIS_TEST_ABORT_NO_MOTION:
      return "no encoder motion";
    case CHASSIS_TEST_ABORT_STOP_TIMEOUT:
      return "wheel did not stop between phases";
    case CHASSIS_TEST_ABORT_DRIVE_TIMEOUT:
      return "drive phase safety timeout";
    case CHASSIS_TEST_ABORT_OVERALL_TIMEOUT:
      return "overall safety timeout";
    case CHASSIS_TEST_ABORT_ESTOP:
      return "ESTOP latched";
    default:
      return "unspecified failure";
  }
}

static bool ChassisMotionTest_GetSnapshots(
  int64_t totals[MOTOR_DRIVER_WHEEL_COUNT],
  WheelSpeedState speeds[MOTOR_DRIVER_WHEEL_COUNT])
{
  EncoderState encoder_states[ENCODER_WHEEL_COUNT];
  WheelSpeedState speed_states[ENCODER_WHEEL_COUNT];

  if ((totals == NULL) || (speeds == NULL) ||
      (!EncoderService_GetSnapshot(encoder_states)) ||
      (!WheelSpeedEstimator_GetSnapshot(speed_states)))
  {
    return false;
  }

  for (uint32_t wheel = 0U; wheel < MOTOR_DRIVER_WHEEL_COUNT; ++wheel)
  {
    const EncoderWheel encoder_wheel = s_encoder_wheels[wheel];

    totals[wheel] = encoder_states[encoder_wheel].accumulated_count;
    speeds[wheel] = speed_states[encoder_wheel];
    if (!speeds[wheel].valid)
    {
      return false;
    }
  }
  return true;
}

static bool ChassisMotionTest_GetAllStopped(bool *all_stopped)
{
  int64_t totals[MOTOR_DRIVER_WHEEL_COUNT];
  WheelSpeedState speeds[MOTOR_DRIVER_WHEEL_COUNT];

  if ((all_stopped == NULL) ||
      (!ChassisMotionTest_GetSnapshots(totals, speeds)))
  {
    return false;
  }

  *all_stopped = true;
  for (uint32_t wheel = 0U; wheel < MOTOR_DRIVER_WHEEL_COUNT; ++wheel)
  {
    if (!speeds[wheel].stopped)
    {
      *all_stopped = false;
      break;
    }
  }
  return true;
}

static void ChassisMotionTest_ResetStatistics(void)
{
  for (uint32_t wheel = 0U; wheel < MOTOR_DRIVER_WHEEL_COUNT; ++wheel)
  {
    s_statistics[wheel].sum_cps = 0.0f;
    s_statistics[wheel].min_cps = 0.0f;
    s_statistics[wheel].max_cps = 0.0f;
    s_statistics[wheel].count = 0U;
    s_statistics[wheel].max_command_permille = 0U;
  }
}

static void ChassisMotionTest_RecordStatistics(
  uint32_t phase_elapsed_ms,
  const ChassisControllerStatus *status)
{
  for (uint32_t wheel = 0U; wheel < MOTOR_DRIVER_WHEEL_COUNT; ++wheel)
  {
    ChassisMotionTestStatistics *statistics = &s_statistics[wheel];
    const float measured = status->measured_counts_per_second[wheel];
    const int16_t output = status->output_permille[wheel];
    const uint16_t magnitude = (uint16_t)((output < 0) ? -output : output);

    if (magnitude > statistics->max_command_permille)
    {
      statistics->max_command_permille = magnitude;
    }
    if (phase_elapsed_ms < CHASSIS_TEST_ANALYSIS_START_MS)
    {
      continue;
    }

    if (statistics->count == 0U)
    {
      statistics->min_cps = measured;
      statistics->max_cps = measured;
    }
    else
    {
      if (measured < statistics->min_cps)
      {
        statistics->min_cps = measured;
      }
      if (measured > statistics->max_cps)
      {
        statistics->max_cps = measured;
      }
    }
    statistics->sum_cps += measured;
    ++statistics->count;
  }
}

static void ChassisMotionTest_SetAborted(ChassisMotionTestAbortReason reason,
                                         MotorDriverWheel fault_wheel,
                                         ChassisControllerFault controller_fault)
{
  taskENTER_CRITICAL();
  ChassisController_Stop();
  s_abort_reason = reason;
  s_fault_wheel = fault_wheel;
  s_controller_fault = controller_fault;
  s_state = CHASSIS_TEST_STATE_ABORTED;
  taskEXIT_CRITICAL();
}

static void ChassisMotionTest_LogSample(
  uint32_t phase_elapsed_ms,
  const ChassisControllerStatus *status)
{
  DEBUG_LOG_INFO(
    "KIN sample: phase=%s t=%lu target_left=%ld right=%ld measured=%ld,%ld,%ld,%ld pwm=%d,%d,%d,%d lim=0x%02lX",
    s_phases[s_phase_index].name,
    (unsigned long)phase_elapsed_ms,
    (long)ChassisMotionTest_RoundFloat(
      status->targets.counts_per_second[MOTOR_DRIVER_WHEEL_LF]),
    (long)ChassisMotionTest_RoundFloat(
      status->targets.counts_per_second[MOTOR_DRIVER_WHEEL_RF]),
    (long)ChassisMotionTest_RoundFloat(
      status->measured_counts_per_second[MOTOR_DRIVER_WHEEL_LF]),
    (long)ChassisMotionTest_RoundFloat(
      status->measured_counts_per_second[MOTOR_DRIVER_WHEEL_LR]),
    (long)ChassisMotionTest_RoundFloat(
      status->measured_counts_per_second[MOTOR_DRIVER_WHEEL_RF]),
    (long)ChassisMotionTest_RoundFloat(
      status->measured_counts_per_second[MOTOR_DRIVER_WHEEL_RR]),
    (int)status->output_permille[MOTOR_DRIVER_WHEEL_LF],
    (int)status->output_permille[MOTOR_DRIVER_WHEEL_LR],
    (int)status->output_permille[MOTOR_DRIVER_WHEEL_RF],
    (int)status->output_permille[MOTOR_DRIVER_WHEEL_RR],
    (unsigned long)status->limited_mask);
}

static void ChassisMotionTest_LogPhaseResult(void)
{
  float averages[MOTOR_DRIVER_WHEEL_COUNT];
  ChassisVelocityEstimate estimate;
  const ChassisMotionTestPhase *phase = &s_phases[s_phase_index];

  for (uint32_t wheel = 0U; wheel < MOTOR_DRIVER_WHEEL_COUNT; ++wheel)
  {
    averages[wheel] = (s_statistics[wheel].count > 0U) ?
      (s_statistics[wheel].sum_cps /
       (float)s_statistics[wheel].count) : 0.0f;
  }

  if (!ChassisKinematics_FromWheelSpeeds(averages, &estimate))
  {
    estimate.linear_x_m_s = 0.0f;
    estimate.angular_z_rad_s = 0.0f;
  }

  DEBUG_LOG_INFO(
    "KIN result: phase=%s cmd_vx=%ldmm/s cmd_wz=%ldmrad/s target_left=%ld right=%ld avg=%ld,%ld,%ld,%ld",
    phase->name,
    (long)ChassisMotionTest_RoundFloat(phase->command.linear_x_m_s * 1000.0f),
    (long)ChassisMotionTest_RoundFloat(phase->command.angular_z_rad_s *
                                      1000.0f),
    (long)ChassisMotionTest_RoundFloat(
      s_phase_targets.counts_per_second[MOTOR_DRIVER_WHEEL_LF]),
    (long)ChassisMotionTest_RoundFloat(
      s_phase_targets.counts_per_second[MOTOR_DRIVER_WHEEL_RF]),
    (long)ChassisMotionTest_RoundFloat(averages[MOTOR_DRIVER_WHEEL_LF]),
    (long)ChassisMotionTest_RoundFloat(averages[MOTOR_DRIVER_WHEEL_LR]),
    (long)ChassisMotionTest_RoundFloat(averages[MOTOR_DRIVER_WHEEL_RF]),
    (long)ChassisMotionTest_RoundFloat(averages[MOTOR_DRIVER_WHEEL_RR]));
  DEBUG_LOG_INFO(
    "KIN chassis: phase=%s estimated_vx=%ldmm/s estimated_wz=%ldmrad/s pair_diff_left/right=%ld/%ld max_pwm=%u,%u,%u,%u samples=%lu",
    phase->name,
    (long)ChassisMotionTest_RoundFloat(estimate.linear_x_m_s * 1000.0f),
    (long)ChassisMotionTest_RoundFloat(estimate.angular_z_rad_s * 1000.0f),
    (long)ChassisMotionTest_RoundFloat(ChassisMotionTest_AbsFloat(
      averages[MOTOR_DRIVER_WHEEL_LF] -
      averages[MOTOR_DRIVER_WHEEL_LR])),
    (long)ChassisMotionTest_RoundFloat(ChassisMotionTest_AbsFloat(
      averages[MOTOR_DRIVER_WHEEL_RF] -
      averages[MOTOR_DRIVER_WHEEL_RR])),
    (unsigned int)s_statistics[MOTOR_DRIVER_WHEEL_LF].max_command_permille,
    (unsigned int)s_statistics[MOTOR_DRIVER_WHEEL_LR].max_command_permille,
    (unsigned int)s_statistics[MOTOR_DRIVER_WHEEL_RF].max_command_permille,
    (unsigned int)s_statistics[MOTOR_DRIVER_WHEEL_RR].max_command_permille,
    (unsigned long)s_statistics[MOTOR_DRIVER_WHEEL_LF].count);
}

static void ChassisMotionTest_StartPhase(uint32_t now_ms)
{
  int64_t totals[MOTOR_DRIVER_WHEEL_COUNT];
  WheelSpeedState speeds[MOTOR_DRIVER_WHEEL_COUNT];
  const ChassisMotionTestPhase *phase = &s_phases[s_phase_index];

  if (!ChassisMotionTest_GetSnapshots(totals, speeds))
  {
    ChassisMotionTest_SetAborted(CHASSIS_TEST_ABORT_SNAPSHOT_FAILURE,
                                 MOTOR_DRIVER_WHEEL_COUNT,
                                 CHASSIS_CONTROLLER_FAULT_NONE);
    return;
  }
  if (!ChassisKinematics_ToWheelTargets(
        &phase->command,
        WHEEL_SPEED_CONTROL_TARGET_LIMIT_CPS,
        &s_phase_targets))
  {
    ChassisMotionTest_SetAborted(CHASSIS_TEST_ABORT_KINEMATICS_FAILURE,
                                 MOTOR_DRIVER_WHEEL_COUNT,
                                 CHASSIS_CONTROLLER_FAULT_NONE);
    return;
  }

  ChassisMotionTest_ResetStatistics();
  for (uint32_t wheel = 0U; wheel < MOTOR_DRIVER_WHEEL_COUNT; ++wheel)
  {
    s_phase_start_totals[wheel] = totals[wheel];
  }

  if (!ChassisController_Start(&phase->command, now_ms))
  {
    ChassisControllerStatus status = { 0 };
    ChassisControllerFault fault = CHASSIS_CONTROLLER_FAULT_CONTROL_UPDATE;

    if (ChassisController_GetStatus(&status))
    {
      fault = status.fault;
    }
    ChassisMotionTest_SetAborted(CHASSIS_TEST_ABORT_CONTROLLER_FAILURE,
                                 MOTOR_DRIVER_WHEEL_COUNT,
                                 fault);
    return;
  }

  taskENTER_CRITICAL();
  if (s_state == CHASSIS_TEST_STATE_PREPARE)
  {
    s_phase_start_ms = now_ms;
    s_phase_deadline_ms = now_ms + CHASSIS_TEST_PHASE_DURATION_MS;
    s_drive_deadline_ms = now_ms + CHASSIS_TEST_PHASE_DURATION_MS +
      CHASSIS_TEST_DRIVE_MARGIN_MS;
    s_no_motion_deadline_ms = now_ms + CHASSIS_TEST_NO_MOTION_TIMEOUT_MS;
    s_next_log_ms = now_ms + CHASSIS_TEST_LOG_PERIOD_MS;
    s_state = CHASSIS_TEST_STATE_RUNNING;
  }
  else
  {
    ChassisController_Stop();
  }
  taskEXIT_CRITICAL();

  if (s_state == CHASSIS_TEST_STATE_RUNNING)
  {
    DEBUG_LOG_WARN(
      "KIN phase started: %s vx=%ld mm/s wz=%ld mrad/s target_left=%ld right=%ld cps duration=3 s",
      phase->name,
      (long)ChassisMotionTest_RoundFloat(phase->command.linear_x_m_s *
                                        1000.0f),
      (long)ChassisMotionTest_RoundFloat(phase->command.angular_z_rad_s *
                                        1000.0f),
      (long)ChassisMotionTest_RoundFloat(
        s_phase_targets.counts_per_second[MOTOR_DRIVER_WHEEL_LF]),
      (long)ChassisMotionTest_RoundFloat(
        s_phase_targets.counts_per_second[MOTOR_DRIVER_WHEEL_RF]));
  }
}

static void ChassisMotionTest_ProcessRunning(uint32_t now_ms)
{
  int64_t totals[MOTOR_DRIVER_WHEEL_COUNT];
  WheelSpeedState speeds[MOTOR_DRIVER_WHEEL_COUNT];
  ChassisControllerStatus status = { 0 };
  const ChassisMotionTestPhase *phase = &s_phases[s_phase_index];
  const uint32_t phase_elapsed_ms = now_ms - s_phase_start_ms;
  bool log_sample = false;

  if (ChassisMotionTest_TimeReached(now_ms, s_phase_deadline_ms))
  {
    ChassisController_Stop();
    ChassisMotionTest_LogPhaseResult();
    taskENTER_CRITICAL();
    if (s_state == CHASSIS_TEST_STATE_RUNNING)
    {
      ++s_phase_index;
      s_stop_guard_deadline_ms = now_ms + CHASSIS_TEST_STOP_GUARD_MS;
      s_stop_timeout_ms = now_ms + CHASSIS_TEST_STOP_TIMEOUT_MS;
      s_state = CHASSIS_TEST_STATE_WAIT_STOP;
    }
    taskEXIT_CRITICAL();
    return;
  }

  if ((!ChassisController_SetCommand(&phase->command, now_ms)))
  {
    (void)ChassisController_GetStatus(&status);
    ChassisMotionTest_SetAborted(CHASSIS_TEST_ABORT_CONTROLLER_FAILURE,
                                 MOTOR_DRIVER_WHEEL_COUNT,
                                 status.fault);
    return;
  }
  ChassisController_Process(now_ms);

  if ((!ChassisController_GetStatus(&status)) || (!status.active) ||
      (status.fault != CHASSIS_CONTROLLER_FAULT_NONE))
  {
    ChassisMotionTest_SetAborted(CHASSIS_TEST_ABORT_CONTROLLER_FAILURE,
                                 MOTOR_DRIVER_WHEEL_COUNT,
                                 status.fault);
    return;
  }
  if (!ChassisMotionTest_GetSnapshots(totals, speeds))
  {
    ChassisMotionTest_SetAborted(CHASSIS_TEST_ABORT_SNAPSHOT_FAILURE,
                                 MOTOR_DRIVER_WHEEL_COUNT,
                                 CHASSIS_CONTROLLER_FAULT_NONE);
    return;
  }

  for (uint32_t wheel = 0U; wheel < MOTOR_DRIVER_WHEEL_COUNT; ++wheel)
  {
    const float target = s_phase_targets.counts_per_second[wheel];
    const int64_t delta = totals[wheel] - s_phase_start_totals[wheel];

    if ((ChassisMotionTest_AbsFloat(target) >=
         CHASSIS_TEST_MIN_ACTIVE_TARGET_CPS) &&
        (ChassisMotionTest_AbsCount(delta) >=
         CHASSIS_TEST_MOVED_THRESHOLD_COUNTS) &&
        (((target > 0.0f) && (delta < 0)) ||
         ((target < 0.0f) && (delta > 0))))
    {
      ChassisMotionTest_SetAborted(CHASSIS_TEST_ABORT_WRONG_DIRECTION,
                                   (MotorDriverWheel)wheel,
                                   CHASSIS_CONTROLLER_FAULT_NONE);
      return;
    }
    if (ChassisMotionTest_TimeReached(now_ms, s_no_motion_deadline_ms) &&
        (ChassisMotionTest_AbsFloat(target) >=
         CHASSIS_TEST_MIN_ACTIVE_TARGET_CPS) &&
        (ChassisMotionTest_AbsCount(delta) <
         CHASSIS_TEST_MOVED_THRESHOLD_COUNTS))
    {
      ChassisMotionTest_SetAborted(CHASSIS_TEST_ABORT_NO_MOTION,
                                   (MotorDriverWheel)wheel,
                                   CHASSIS_CONTROLLER_FAULT_NONE);
      return;
    }
  }

  ChassisMotionTest_RecordStatistics(phase_elapsed_ms, &status);
  taskENTER_CRITICAL();
  if ((s_state == CHASSIS_TEST_STATE_RUNNING) &&
      ChassisMotionTest_TimeReached(now_ms, s_next_log_ms))
  {
    s_next_log_ms = now_ms + CHASSIS_TEST_LOG_PERIOD_MS;
    log_sample = true;
  }
  taskEXIT_CRITICAL();

  if (log_sample)
  {
    ChassisMotionTest_LogSample(phase_elapsed_ms, &status);
  }
}

static void ChassisMotionTest_WaitForStop(uint32_t now_ms)
{
  bool all_stopped;

  if (!ChassisMotionTest_GetAllStopped(&all_stopped))
  {
    ChassisMotionTest_SetAborted(CHASSIS_TEST_ABORT_SNAPSHOT_FAILURE,
                                 MOTOR_DRIVER_WHEEL_COUNT,
                                 CHASSIS_CONTROLLER_FAULT_NONE);
  }
  else if (ChassisMotionTest_TimeReached(now_ms, s_stop_timeout_ms))
  {
    ChassisMotionTest_SetAborted(CHASSIS_TEST_ABORT_STOP_TIMEOUT,
                                 MOTOR_DRIVER_WHEEL_COUNT,
                                 CHASSIS_CONTROLLER_FAULT_NONE);
  }
  else if (ChassisMotionTest_TimeReached(now_ms,
                                         s_stop_guard_deadline_ms) &&
           all_stopped)
  {
    if (s_phase_index >= CHASSIS_TEST_PHASE_COUNT)
    {
      taskENTER_CRITICAL();
      if (s_state == CHASSIS_TEST_STATE_WAIT_STOP)
      {
        s_state = CHASSIS_TEST_STATE_COMPLETED;
      }
      taskEXIT_CRITICAL();
    }
    else
    {
      taskENTER_CRITICAL();
      if (s_state == CHASSIS_TEST_STATE_WAIT_STOP)
      {
        s_state = CHASSIS_TEST_STATE_PREPARE;
      }
      taskEXIT_CRITICAL();
      ChassisMotionTest_StartPhase(now_ms);
    }
  }
}

void ChassisMotionTest_Init(void)
{
  taskENTER_CRITICAL();
  ChassisController_Stop();
  s_state = CHASSIS_TEST_STATE_DISARMED;
  s_abort_reason = CHASSIS_TEST_ABORT_NONE;
  s_controller_fault = CHASSIS_CONTROLLER_FAULT_NONE;
  s_fault_wheel = MOTOR_DRIVER_WHEEL_COUNT;
  s_arm_deadline_ms = 0U;
  s_drive_deadline_ms = 0U;
  s_overall_deadline_ms = 0U;
  s_arm_expired_notice = false;
  s_phase_index = 0U;
  s_phase_start_ms = 0U;
  s_phase_deadline_ms = 0U;
  s_no_motion_deadline_ms = 0U;
  s_stop_guard_deadline_ms = 0U;
  s_stop_timeout_ms = 0U;
  s_next_log_ms = 0U;
  for (uint32_t wheel = 0U; wheel < MOTOR_DRIVER_WHEEL_COUNT; ++wheel)
  {
    s_phase_start_totals[wheel] = 0;
  }
  ChassisMotionTest_ResetStatistics();
  taskEXIT_CRITICAL();
}

bool ChassisMotionTest_Arm(uint32_t now_ms)
{
  bool armed = false;

  if (!EmergencyStop_CanEnableMotor())
  {
    DEBUG_LOG_ERROR("KIN arm rejected: ESTOP input/latch active");
    return false;
  }

  taskENTER_CRITICAL();
  if (s_state == CHASSIS_TEST_STATE_DISARMED)
  {
    ChassisController_Stop();
    s_phase_index = 0U;
    s_arm_deadline_ms = now_ms + CHASSIS_TEST_ARM_WINDOW_MS;
    s_abort_reason = CHASSIS_TEST_ABORT_NONE;
    s_controller_fault = CHASSIS_CONTROLLER_FAULT_NONE;
    s_fault_wheel = MOTOR_DRIVER_WHEEL_COUNT;
    s_arm_expired_notice = false;
    s_state = CHASSIS_TEST_STATE_ARMED;
    armed = true;
  }
  taskEXIT_CRITICAL();

  if (armed)
  {
    DEBUG_LOG_WARN("KIN chassis test armed for 5 s; send 'g' to start");
    DEBUG_LOG_WARN(
      "Sequence: forward, reverse, turn-left, turn-right; 3 s each with full stops");
  }
  return armed;
}

bool ChassisMotionTest_Start(uint32_t now_ms)
{
  bool queued = false;

  if (!EmergencyStop_CanEnableMotor())
  {
    DEBUG_LOG_ERROR("KIN start rejected: ESTOP input/latch active");
    return false;
  }

  taskENTER_CRITICAL();
  if ((s_state == CHASSIS_TEST_STATE_ARMED) &&
      (!ChassisMotionTest_TimeReached(now_ms, s_arm_deadline_ms)))
  {
    ChassisController_Stop();
    s_phase_index = 0U;
    s_phase_deadline_ms = now_ms + CHASSIS_TEST_COUNTDOWN_MS;
    s_overall_deadline_ms = now_ms + CHASSIS_TEST_OVERALL_TIMEOUT_MS;
    s_state = CHASSIS_TEST_STATE_COUNTDOWN;
    queued = true;
  }
  taskEXIT_CRITICAL();

  if (queued)
  {
    DEBUG_LOG_WARN("KIN chassis test starts in 3 s; send 's' to cancel");
    DEBUG_LOG_WARN("Keep all four wheels lifted and keep 12 V physical cutoff ready");
    DEBUG_LOG_INFO(
      "KIN convention: +vx=forward +wz=CCW-left; track=210 mm counts/rev=1560");
  }
  else
  {
    DEBUG_LOG_WARN("KIN start rejected: send 'k', then 'g' within 5 s");
  }
  return queued;
}

bool ChassisMotionTest_IsActive(void)
{
  return (s_state != CHASSIS_TEST_STATE_DISARMED);
}

void ChassisMotionTest_Process(uint32_t now_ms)
{
  ChassisMotionTestState state;
  ChassisMotionTestAbortReason abort_reason = CHASSIS_TEST_ABORT_NONE;
  ChassisControllerFault controller_fault = CHASSIS_CONTROLLER_FAULT_NONE;
  MotorDriverWheel fault_wheel = MOTOR_DRIVER_WHEEL_COUNT;
  bool completed_notice = false;
  bool aborted_notice = false;
  bool arm_expired_notice = false;

  taskENTER_CRITICAL();
  if (s_state == CHASSIS_TEST_STATE_COMPLETED)
  {
    s_state = CHASSIS_TEST_STATE_DISARMED;
    completed_notice = true;
  }
  else if (s_state == CHASSIS_TEST_STATE_ABORTED)
  {
    abort_reason = s_abort_reason;
    controller_fault = s_controller_fault;
    fault_wheel = s_fault_wheel;
    s_state = CHASSIS_TEST_STATE_DISARMED;
    aborted_notice = true;
  }
  if (s_arm_expired_notice)
  {
    s_arm_expired_notice = false;
    arm_expired_notice = true;
  }
  state = s_state;
  taskEXIT_CRITICAL();

  if (completed_notice)
  {
    DEBUG_LOG_INFO("KIN chassis test completed; all motor outputs disabled");
  }
  if (aborted_notice)
  {
    DEBUG_LOG_ERROR(
      "KIN test aborted: wheel=%s reason=%s controller=%s; all motor outputs disabled",
      (fault_wheel < MOTOR_DRIVER_WHEEL_COUNT) ?
        s_wheel_names[fault_wheel] : "ALL",
      ChassisMotionTest_GetAbortReasonName(abort_reason),
      ChassisController_GetFaultName(controller_fault));
  }
  if (arm_expired_notice)
  {
    DEBUG_LOG_INFO("KIN arm window expired; motors remain disabled");
  }

  switch (state)
  {
    case CHASSIS_TEST_STATE_COUNTDOWN:
      if (ChassisMotionTest_TimeReached(now_ms, s_phase_deadline_ms))
      {
        taskENTER_CRITICAL();
        if (s_state == CHASSIS_TEST_STATE_COUNTDOWN)
        {
          s_stop_guard_deadline_ms = now_ms + CHASSIS_TEST_STOP_GUARD_MS;
          s_stop_timeout_ms = now_ms + CHASSIS_TEST_STOP_TIMEOUT_MS;
          s_state = CHASSIS_TEST_STATE_PREPARE;
        }
        taskEXIT_CRITICAL();
      }
      break;

    case CHASSIS_TEST_STATE_PREPARE:
    {
      bool all_stopped;
      if (!ChassisMotionTest_GetAllStopped(&all_stopped))
      {
        ChassisMotionTest_SetAborted(CHASSIS_TEST_ABORT_SNAPSHOT_FAILURE,
                                     MOTOR_DRIVER_WHEEL_COUNT,
                                     CHASSIS_CONTROLLER_FAULT_NONE);
      }
      else if (ChassisMotionTest_TimeReached(now_ms, s_stop_timeout_ms))
      {
        ChassisMotionTest_SetAborted(CHASSIS_TEST_ABORT_STOP_TIMEOUT,
                                     MOTOR_DRIVER_WHEEL_COUNT,
                                     CHASSIS_CONTROLLER_FAULT_NONE);
      }
      else if (ChassisMotionTest_TimeReached(now_ms,
                                             s_stop_guard_deadline_ms) &&
               all_stopped)
      {
        ChassisMotionTest_StartPhase(now_ms);
      }
      break;
    }

    case CHASSIS_TEST_STATE_RUNNING:
      ChassisMotionTest_ProcessRunning(now_ms);
      break;

    case CHASSIS_TEST_STATE_WAIT_STOP:
      ChassisMotionTest_WaitForStop(now_ms);
      break;

    default:
      break;
  }
}

void ChassisMotionTest_Stop(void)
{
  bool was_active;

  taskENTER_CRITICAL();
  was_active = (s_state != CHASSIS_TEST_STATE_DISARMED);
  ChassisController_Stop();
  s_state = CHASSIS_TEST_STATE_DISARMED;
  s_abort_reason = CHASSIS_TEST_ABORT_NONE;
  s_controller_fault = CHASSIS_CONTROLLER_FAULT_NONE;
  s_fault_wheel = MOTOR_DRIVER_WHEEL_COUNT;
  s_arm_expired_notice = false;
  taskEXIT_CRITICAL();

  if (was_active)
  {
    DEBUG_LOG_WARN("KIN chassis test cancelled; all motor outputs disabled");
  }
}

void ChassisMotionTest_SafetyProcess(uint32_t now_ms)
{
  taskENTER_CRITICAL();
  if (EmergencyStop_IsLatched() &&
      (s_state != CHASSIS_TEST_STATE_DISARMED))
  {
    ChassisController_Stop();
    s_abort_reason = CHASSIS_TEST_ABORT_ESTOP;
    s_controller_fault = CHASSIS_CONTROLLER_FAULT_ESTOP;
    s_fault_wheel = MOTOR_DRIVER_WHEEL_COUNT;
    s_state = CHASSIS_TEST_STATE_ABORTED;
  }
  else if ((s_state == CHASSIS_TEST_STATE_ARMED) &&
           ChassisMotionTest_TimeReached(now_ms, s_arm_deadline_ms))
  {
    ChassisController_Stop();
    s_state = CHASSIS_TEST_STATE_DISARMED;
    s_arm_expired_notice = true;
  }
  else if (((s_state == CHASSIS_TEST_STATE_COUNTDOWN) ||
            (s_state == CHASSIS_TEST_STATE_PREPARE) ||
            (s_state == CHASSIS_TEST_STATE_RUNNING) ||
            (s_state == CHASSIS_TEST_STATE_WAIT_STOP)) &&
           ChassisMotionTest_TimeReached(now_ms, s_overall_deadline_ms))
  {
    ChassisController_Stop();
    s_abort_reason = CHASSIS_TEST_ABORT_OVERALL_TIMEOUT;
    s_controller_fault = CHASSIS_CONTROLLER_FAULT_NONE;
    s_fault_wheel = MOTOR_DRIVER_WHEEL_COUNT;
    s_state = CHASSIS_TEST_STATE_ABORTED;
  }
  else if ((s_state == CHASSIS_TEST_STATE_RUNNING) &&
           ChassisMotionTest_TimeReached(now_ms, s_drive_deadline_ms))
  {
    ChassisController_Stop();
    s_abort_reason = CHASSIS_TEST_ABORT_DRIVE_TIMEOUT;
    s_controller_fault = CHASSIS_CONTROLLER_FAULT_NONE;
    s_fault_wheel = MOTOR_DRIVER_WHEEL_COUNT;
    s_state = CHASSIS_TEST_STATE_ABORTED;
  }
  taskEXIT_CRITICAL();
}

#else

void ChassisMotionTest_Init(void)
{
}

bool ChassisMotionTest_Arm(uint32_t now_ms)
{
  (void)now_ms;
  return false;
}

bool ChassisMotionTest_Start(uint32_t now_ms)
{
  (void)now_ms;
  return false;
}

bool ChassisMotionTest_IsActive(void)
{
  return false;
}

void ChassisMotionTest_Process(uint32_t now_ms)
{
  (void)now_ms;
}

void ChassisMotionTest_Stop(void)
{
}

void ChassisMotionTest_SafetyProcess(uint32_t now_ms)
{
  (void)now_ms;
}

#endif /* MOTOR_OPEN_LOOP_TEST_ENABLED */
