/**
 * @file chassis_ground_test.c
 * @brief Short, explicitly armed low-speed chassis ground motion test.
 */

#include "chassis_ground_test.h"

#if MOTOR_OPEN_LOOP_TEST_ENABLED

#include <stddef.h>

#include "FreeRTOS.h"
#include "chassis_controller.h"
#include "chassis_kinematics.h"
#include "debug_log.h"
#include "emergency_stop.h"
#include "encoder_service.h"
#include "motor_driver.h"
#include "task.h"
#include "wheel_speed_control_config.h"
#include "wheel_speed_estimator.h"

#define GROUND_TEST_ARM_WINDOW_MS          5000U
#define GROUND_TEST_COUNTDOWN_MS           2000U
#define GROUND_TEST_DRIVE_DURATION_MS      1500U
#define GROUND_TEST_ANALYSIS_START_MS       750U
#define GROUND_TEST_NO_MOTION_TIMEOUT_MS    700U
#define GROUND_TEST_START_ASSIST_MAX_MS      300U
#define GROUND_TEST_START_ASSIST_MIN_MS      200U
#define GROUND_TEST_START_ASSIST_PERMILLE    200
#define GROUND_TEST_START_ASSIST_TRAVEL_COUNTS 12
#define GROUND_TEST_DRIVE_MARGIN_MS          300U
#define GROUND_TEST_STOP_GUARD_MS            300U
#define GROUND_TEST_STOP_TIMEOUT_MS         2000U
#define GROUND_TEST_OVERALL_TIMEOUT_MS      7000U
#define GROUND_TEST_LOG_PERIOD_MS            150U
#define GROUND_TEST_DIRECTION_THRESHOLD_COUNTS 4
#define GROUND_TEST_PROGRESS_COUNTS             8
#define GROUND_TEST_MIN_TRAVEL_COUNTS           40
#define GROUND_TEST_MIN_TRAVEL_FRACTION       0.15f
#define GROUND_TEST_MIN_ACTIVE_TARGET_CPS    50.0f
#define GROUND_TEST_OVERSPEED_CPS          1000.0f

typedef enum
{
  GROUND_TEST_STATE_DISARMED = 0,
  GROUND_TEST_STATE_ARMED,
  GROUND_TEST_STATE_COUNTDOWN,
  GROUND_TEST_STATE_START_ASSIST,
  GROUND_TEST_STATE_RUNNING,
  GROUND_TEST_STATE_WAIT_STOP,
  GROUND_TEST_STATE_COMPLETED,
  GROUND_TEST_STATE_ABORTED
} ChassisGroundTestState;

typedef enum
{
  GROUND_TEST_ABORT_NONE = 0,
  GROUND_TEST_ABORT_NOT_STOPPED,
  GROUND_TEST_ABORT_SNAPSHOT_FAILURE,
  GROUND_TEST_ABORT_KINEMATICS_FAILURE,
  GROUND_TEST_ABORT_CONTROLLER_FAILURE,
  GROUND_TEST_ABORT_WRONG_DIRECTION,
  GROUND_TEST_ABORT_NO_MOTION,
  GROUND_TEST_ABORT_INSUFFICIENT_TRAVEL,
  GROUND_TEST_ABORT_OVERSPEED,
  GROUND_TEST_ABORT_STOP_TIMEOUT,
  GROUND_TEST_ABORT_DRIVE_TIMEOUT,
  GROUND_TEST_ABORT_OVERALL_TIMEOUT,
  GROUND_TEST_ABORT_ESTOP
} ChassisGroundTestAbortReason;

typedef struct
{
  const char *name;
  char command_key;
  ChassisVelocityCommand command;
} ChassisGroundTestDefinition;

typedef struct
{
  float sum_cps;
  uint32_t count;
  uint16_t max_command_permille;
} ChassisGroundTestStatistics;

static const ChassisGroundTestDefinition s_definitions[
  CHASSIS_GROUND_ACTION_COUNT] = {
  [CHASSIS_GROUND_ACTION_FORWARD] = {
    "FORWARD", 'w', { 0.040f, 0.0f }
  },
  [CHASSIS_GROUND_ACTION_REVERSE] = {
    "REVERSE", 'x', { -0.040f, 0.0f }
  },
  [CHASSIS_GROUND_ACTION_TURN_LEFT] = {
    "TURN_LEFT", 'a', { 0.0f, 0.350f }
  },
  [CHASSIS_GROUND_ACTION_TURN_RIGHT] = {
    "TURN_RIGHT", 'd', { 0.0f, -0.350f }
  },
  [CHASSIS_GROUND_ACTION_ARC_LEFT] = {
    "ARC_LEFT", 'j', { 0.060f, 0.300f }
  },
  [CHASSIS_GROUND_ACTION_ARC_RIGHT] = {
    "ARC_RIGHT", 'l', { 0.060f, -0.300f }
  }
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

static volatile ChassisGroundTestState s_state;
static volatile ChassisGroundTestAbortReason s_abort_reason;
static volatile ChassisControllerFault s_controller_fault;
static volatile MotorDriverWheel s_fault_wheel;
static volatile uint32_t s_arm_deadline_ms;
static volatile uint32_t s_countdown_deadline_ms;
static volatile uint32_t s_drive_deadline_ms;
static volatile uint32_t s_overall_deadline_ms;
static volatile bool s_arm_expired_notice;
static ChassisGroundTestAction s_action;
static uint32_t s_motion_start_ms;
static uint32_t s_motion_deadline_ms;
static uint32_t s_start_assist_start_ms;
static uint32_t s_start_assist_deadline_ms;
static uint32_t s_stop_guard_deadline_ms;
static uint32_t s_stop_timeout_ms;
static uint32_t s_next_log_ms;
static int64_t s_start_totals[MOTOR_DRIVER_WHEEL_COUNT];
static int64_t s_end_totals[MOTOR_DRIVER_WHEEL_COUNT];
static int64_t s_progress_totals[MOTOR_DRIVER_WHEEL_COUNT];
static uint32_t s_last_progress_ms[MOTOR_DRIVER_WHEEL_COUNT];
static ChassisWheelTargets s_targets;
static ChassisGroundTestStatistics s_statistics[MOTOR_DRIVER_WHEEL_COUNT];

static void ChassisGroundTest_SetAborted(ChassisGroundTestAbortReason reason,
                                         MotorDriverWheel fault_wheel,
                                         ChassisControllerFault controller_fault);

static bool ChassisGroundTest_TimeReached(uint32_t now_ms,
                                          uint32_t deadline_ms)
{
  return ((int32_t)(now_ms - deadline_ms) >= 0);
}

static float ChassisGroundTest_AbsFloat(float value)
{
  return (value < 0.0f) ? -value : value;
}

static int64_t ChassisGroundTest_AbsCount(int64_t value)
{
  return (value < 0) ? -value : value;
}

static int32_t ChassisGroundTest_RoundFloat(float value)
{
  return (int32_t)((value >= 0.0f) ? (value + 0.5f) : (value - 0.5f));
}

static const char *ChassisGroundTest_GetAbortReasonName(
  ChassisGroundTestAbortReason reason)
{
  switch (reason)
  {
    case GROUND_TEST_ABORT_NOT_STOPPED:
      return "chassis was moving before start";
    case GROUND_TEST_ABORT_SNAPSHOT_FAILURE:
      return "encoder/speed snapshot failure";
    case GROUND_TEST_ABORT_KINEMATICS_FAILURE:
      return "kinematics conversion failure";
    case GROUND_TEST_ABORT_CONTROLLER_FAILURE:
      return "chassis controller failure";
    case GROUND_TEST_ABORT_WRONG_DIRECTION:
      return "encoder direction disagrees with wheel target";
    case GROUND_TEST_ABORT_NO_MOTION:
      return "no continuous encoder progress";
    case GROUND_TEST_ABORT_INSUFFICIENT_TRAVEL:
      return "insufficient encoder travel";
    case GROUND_TEST_ABORT_OVERSPEED:
      return "ground-test overspeed";
    case GROUND_TEST_ABORT_STOP_TIMEOUT:
      return "wheel did not stop";
    case GROUND_TEST_ABORT_DRIVE_TIMEOUT:
      return "drive phase safety timeout";
    case GROUND_TEST_ABORT_OVERALL_TIMEOUT:
      return "overall safety timeout";
    case GROUND_TEST_ABORT_ESTOP:
      return "ESTOP latched";
    default:
      return "unspecified failure";
  }
}

static bool ChassisGroundTest_IsPivotAction(void)
{
  return (s_action == CHASSIS_GROUND_ACTION_TURN_LEFT) ||
         (s_action == CHASSIS_GROUND_ACTION_TURN_RIGHT);
}

static bool ChassisGroundTest_HasExpectedDirection(float target,
                                                    int64_t delta)
{
  return ((target > 0.0f) && (delta > 0)) ||
         ((target < 0.0f) && (delta < 0));
}

static int16_t ChassisGroundTest_GetAssistCommand(float target)
{
  if (target > 0.0f)
  {
    return GROUND_TEST_START_ASSIST_PERMILLE;
  }
  if (target < 0.0f)
  {
    return -GROUND_TEST_START_ASSIST_PERMILLE;
  }
  return 0;
}

static bool ChassisGroundTest_GetSnapshots(
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

static void ChassisGroundTest_ResetStatistics(void)
{
  for (uint32_t wheel = 0U; wheel < MOTOR_DRIVER_WHEEL_COUNT; ++wheel)
  {
    s_statistics[wheel].sum_cps = 0.0f;
    s_statistics[wheel].count = 0U;
    s_statistics[wheel].max_command_permille = 0U;
  }
}

static void ChassisGroundTest_ResetProgress(
  const int64_t totals[MOTOR_DRIVER_WHEEL_COUNT],
  uint32_t now_ms)
{
  for (uint32_t wheel = 0U; wheel < MOTOR_DRIVER_WHEEL_COUNT; ++wheel)
  {
    s_progress_totals[wheel] = totals[wheel];
    s_last_progress_ms[wheel] = now_ms;
  }
}

static bool ChassisGroundTest_CheckWheelMotion(
  uint32_t now_ms,
  const int64_t totals[MOTOR_DRIVER_WHEEL_COUNT],
  const WheelSpeedState speeds[MOTOR_DRIVER_WHEEL_COUNT])
{
  for (uint32_t wheel = 0U; wheel < MOTOR_DRIVER_WHEEL_COUNT; ++wheel)
  {
    const float target = s_targets.counts_per_second[wheel];
    const int64_t total_delta = totals[wheel] - s_start_totals[wheel];
    const int64_t progress_delta = totals[wheel] - s_progress_totals[wheel];

    s_end_totals[wheel] = totals[wheel];
    if (ChassisGroundTest_AbsFloat(
          speeds[wheel].filtered_counts_per_second) >
        GROUND_TEST_OVERSPEED_CPS)
    {
      ChassisGroundTest_SetAborted(GROUND_TEST_ABORT_OVERSPEED,
                                   (MotorDriverWheel)wheel,
                                   CHASSIS_CONTROLLER_FAULT_NONE);
      return false;
    }
    if ((ChassisGroundTest_AbsFloat(target) >=
         GROUND_TEST_MIN_ACTIVE_TARGET_CPS) &&
        (ChassisGroundTest_AbsCount(total_delta) >=
         GROUND_TEST_DIRECTION_THRESHOLD_COUNTS) &&
        (!ChassisGroundTest_HasExpectedDirection(target, total_delta)))
    {
      ChassisGroundTest_SetAborted(GROUND_TEST_ABORT_WRONG_DIRECTION,
                                   (MotorDriverWheel)wheel,
                                   CHASSIS_CONTROLLER_FAULT_NONE);
      return false;
    }
    if ((ChassisGroundTest_AbsFloat(target) >=
         GROUND_TEST_MIN_ACTIVE_TARGET_CPS) &&
        (ChassisGroundTest_AbsCount(progress_delta) >=
         GROUND_TEST_PROGRESS_COUNTS) &&
        ChassisGroundTest_HasExpectedDirection(target, progress_delta))
    {
      s_progress_totals[wheel] = totals[wheel];
      s_last_progress_ms[wheel] = now_ms;
    }
    if ((ChassisGroundTest_AbsFloat(target) >=
         GROUND_TEST_MIN_ACTIVE_TARGET_CPS) &&
        ChassisGroundTest_TimeReached(
          now_ms,
          s_last_progress_ms[wheel] + GROUND_TEST_NO_MOTION_TIMEOUT_MS))
    {
      ChassisGroundTest_SetAborted(GROUND_TEST_ABORT_NO_MOTION,
                                   (MotorDriverWheel)wheel,
                                   CHASSIS_CONTROLLER_FAULT_NONE);
      return false;
    }
  }
  return true;
}

static bool ChassisGroundTest_ValidateTravel(void)
{
  for (uint32_t wheel = 0U; wheel < MOTOR_DRIVER_WHEEL_COUNT; ++wheel)
  {
    const float target = s_targets.counts_per_second[wheel];
    const int64_t delta = s_end_totals[wheel] - s_start_totals[wheel];
    float required = ChassisGroundTest_AbsFloat(target) *
      ((float)GROUND_TEST_DRIVE_DURATION_MS / 1000.0f) *
      GROUND_TEST_MIN_TRAVEL_FRACTION;

    if (required < (float)GROUND_TEST_MIN_TRAVEL_COUNTS)
    {
      required = (float)GROUND_TEST_MIN_TRAVEL_COUNTS;
    }
    if ((ChassisGroundTest_AbsFloat(target) >=
         GROUND_TEST_MIN_ACTIVE_TARGET_CPS) &&
        ((!ChassisGroundTest_HasExpectedDirection(target, delta)) ||
         ((float)ChassisGroundTest_AbsCount(delta) < required)))
    {
      ChassisGroundTest_SetAborted(GROUND_TEST_ABORT_INSUFFICIENT_TRAVEL,
                                   (MotorDriverWheel)wheel,
                                   CHASSIS_CONTROLLER_FAULT_NONE);
      return false;
    }
  }
  return true;
}

static bool ChassisGroundTest_AllWheelsStarted(
  const int64_t totals[MOTOR_DRIVER_WHEEL_COUNT])
{
  for (uint32_t wheel = 0U; wheel < MOTOR_DRIVER_WHEEL_COUNT; ++wheel)
  {
    const float target = s_targets.counts_per_second[wheel];
    const int64_t delta = totals[wheel] - s_start_totals[wheel];

    if ((ChassisGroundTest_AbsCount(delta) <
         GROUND_TEST_START_ASSIST_TRAVEL_COUNTS) ||
        (!ChassisGroundTest_HasExpectedDirection(target, delta)))
    {
      return false;
    }
  }
  return true;
}

static void ChassisGroundTest_RecordStatistics(
  uint32_t elapsed_ms,
  const ChassisControllerStatus *status)
{
  for (uint32_t wheel = 0U; wheel < MOTOR_DRIVER_WHEEL_COUNT; ++wheel)
  {
    ChassisGroundTestStatistics *statistics = &s_statistics[wheel];
    const int16_t output = status->output_permille[wheel];
    const uint16_t magnitude = (uint16_t)((output < 0) ? -output : output);

    if (magnitude > statistics->max_command_permille)
    {
      statistics->max_command_permille = magnitude;
    }
    if (elapsed_ms >= GROUND_TEST_ANALYSIS_START_MS)
    {
      statistics->sum_cps += status->measured_counts_per_second[wheel];
      ++statistics->count;
    }
  }
}

static void ChassisGroundTest_SetAborted(ChassisGroundTestAbortReason reason,
                                         MotorDriverWheel fault_wheel,
                                         ChassisControllerFault controller_fault)
{
  taskENTER_CRITICAL();
  ChassisController_Stop();
  s_abort_reason = reason;
  s_fault_wheel = fault_wheel;
  s_controller_fault = controller_fault;
  s_state = GROUND_TEST_STATE_ABORTED;
  taskEXIT_CRITICAL();
}

static void ChassisGroundTest_LogSample(
  uint32_t elapsed_ms,
  const ChassisControllerStatus *status)
{
  DEBUG_LOG_INFO(
    "GROUND sample: action=%s t=%lu target_left=%ld right=%ld measured=%ld,%ld,%ld,%ld pwm=%d,%d,%d,%d lim=0x%02lX",
    s_definitions[s_action].name,
    (unsigned long)elapsed_ms,
    (long)ChassisGroundTest_RoundFloat(
      status->targets.counts_per_second[MOTOR_DRIVER_WHEEL_LF]),
    (long)ChassisGroundTest_RoundFloat(
      status->targets.counts_per_second[MOTOR_DRIVER_WHEEL_RF]),
    (long)ChassisGroundTest_RoundFloat(
      status->measured_counts_per_second[MOTOR_DRIVER_WHEEL_LF]),
    (long)ChassisGroundTest_RoundFloat(
      status->measured_counts_per_second[MOTOR_DRIVER_WHEEL_LR]),
    (long)ChassisGroundTest_RoundFloat(
      status->measured_counts_per_second[MOTOR_DRIVER_WHEEL_RF]),
    (long)ChassisGroundTest_RoundFloat(
      status->measured_counts_per_second[MOTOR_DRIVER_WHEEL_RR]),
    (int)status->output_permille[MOTOR_DRIVER_WHEEL_LF],
    (int)status->output_permille[MOTOR_DRIVER_WHEEL_LR],
    (int)status->output_permille[MOTOR_DRIVER_WHEEL_RF],
    (int)status->output_permille[MOTOR_DRIVER_WHEEL_RR],
    (unsigned long)status->limited_mask);
}

static void ChassisGroundTest_LogResult(void)
{
  float averages[MOTOR_DRIVER_WHEEL_COUNT];
  ChassisVelocityEstimate estimate;
  char delta_text[MOTOR_DRIVER_WHEEL_COUNT][DEBUG_LOG_INT64_TEXT_SIZE];

  for (uint32_t wheel = 0U; wheel < MOTOR_DRIVER_WHEEL_COUNT; ++wheel)
  {
    averages[wheel] = (s_statistics[wheel].count > 0U) ?
      (s_statistics[wheel].sum_cps /
       (float)s_statistics[wheel].count) : 0.0f;
    DebugLog_FormatInt64(s_end_totals[wheel] - s_start_totals[wheel],
                         delta_text[wheel]);
  }
  if (!ChassisKinematics_FromWheelSpeeds(averages, &estimate))
  {
    estimate.linear_x_m_s = 0.0f;
    estimate.angular_z_rad_s = 0.0f;
  }

  DEBUG_LOG_INFO(
    "GROUND result: action=%s avg=%ld,%ld,%ld,%ld estimated_vx=%ldmm/s estimated_wz=%ldmrad/s",
    s_definitions[s_action].name,
    (long)ChassisGroundTest_RoundFloat(averages[MOTOR_DRIVER_WHEEL_LF]),
    (long)ChassisGroundTest_RoundFloat(averages[MOTOR_DRIVER_WHEEL_LR]),
    (long)ChassisGroundTest_RoundFloat(averages[MOTOR_DRIVER_WHEEL_RF]),
    (long)ChassisGroundTest_RoundFloat(averages[MOTOR_DRIVER_WHEEL_RR]),
    (long)ChassisGroundTest_RoundFloat(estimate.linear_x_m_s * 1000.0f),
    (long)ChassisGroundTest_RoundFloat(estimate.angular_z_rad_s * 1000.0f));
  DEBUG_LOG_INFO(
    "GROUND travel: action=%s delta=%s,%s,%s,%s max_pwm=%u,%u,%u,%u samples=%lu",
    s_definitions[s_action].name,
    delta_text[MOTOR_DRIVER_WHEEL_LF],
    delta_text[MOTOR_DRIVER_WHEEL_LR],
    delta_text[MOTOR_DRIVER_WHEEL_RF],
    delta_text[MOTOR_DRIVER_WHEEL_RR],
    (unsigned int)s_statistics[MOTOR_DRIVER_WHEEL_LF].max_command_permille,
    (unsigned int)s_statistics[MOTOR_DRIVER_WHEEL_LR].max_command_permille,
    (unsigned int)s_statistics[MOTOR_DRIVER_WHEEL_RF].max_command_permille,
    (unsigned int)s_statistics[MOTOR_DRIVER_WHEEL_RR].max_command_permille,
    (unsigned long)s_statistics[MOTOR_DRIVER_WHEEL_LF].count);
}

static void ChassisGroundTest_StartController(uint32_t now_ms)
{
  int64_t totals[MOTOR_DRIVER_WHEEL_COUNT];
  WheelSpeedState speeds[MOTOR_DRIVER_WHEEL_COUNT];
  const ChassisGroundTestDefinition *definition = &s_definitions[s_action];
  const bool adopt_assist_outputs =
    (s_state == GROUND_TEST_STATE_START_ASSIST);

  if (!ChassisGroundTest_GetSnapshots(totals, speeds))
  {
    ChassisGroundTest_SetAborted(GROUND_TEST_ABORT_SNAPSHOT_FAILURE,
                                 MOTOR_DRIVER_WHEEL_COUNT,
                                 CHASSIS_CONTROLLER_FAULT_NONE);
    return;
  }
  if (!(adopt_assist_outputs ?
        ChassisController_StartFromActiveOutputs(&definition->command,
                                                 now_ms) :
        ChassisController_Start(&definition->command, now_ms)))
  {
    ChassisControllerStatus status = { 0 };
    (void)ChassisController_GetStatus(&status);
    ChassisGroundTest_SetAborted(GROUND_TEST_ABORT_CONTROLLER_FAILURE,
                                 MOTOR_DRIVER_WHEEL_COUNT,
                                 status.fault);
    return;
  }

  ChassisGroundTest_ResetProgress(totals, now_ms);
  taskENTER_CRITICAL();
  if ((s_state == GROUND_TEST_STATE_COUNTDOWN) ||
      (s_state == GROUND_TEST_STATE_START_ASSIST))
  {
    s_motion_start_ms = now_ms;
    s_motion_deadline_ms = now_ms + GROUND_TEST_DRIVE_DURATION_MS;
    s_drive_deadline_ms = now_ms + GROUND_TEST_DRIVE_DURATION_MS +
      GROUND_TEST_DRIVE_MARGIN_MS;
    s_next_log_ms = now_ms + GROUND_TEST_LOG_PERIOD_MS;
    s_state = GROUND_TEST_STATE_RUNNING;
  }
  else
  {
    ChassisController_Stop();
  }
  taskEXIT_CRITICAL();

  if (s_state == GROUND_TEST_STATE_RUNNING)
  {
    DEBUG_LOG_WARN(
      "GROUND motion started: %s vx=%ldmm/s wz=%ldmrad/s target_left=%ld right=%ld cps duration=%lu ms",
      definition->name,
      (long)ChassisGroundTest_RoundFloat(definition->command.linear_x_m_s *
                                        1000.0f),
      (long)ChassisGroundTest_RoundFloat(definition->command.angular_z_rad_s *
                                        1000.0f),
      (long)ChassisGroundTest_RoundFloat(
        s_targets.counts_per_second[MOTOR_DRIVER_WHEEL_LF]),
      (long)ChassisGroundTest_RoundFloat(
        s_targets.counts_per_second[MOTOR_DRIVER_WHEEL_RF]),
      (unsigned long)GROUND_TEST_DRIVE_DURATION_MS);
  }
}

static bool ChassisGroundTest_StartAssist(uint32_t now_ms)
{
  bool started = MotorDriver_Enable();

  for (uint32_t wheel = 0U;
       started && (wheel < MOTOR_DRIVER_WHEEL_COUNT);
       ++wheel)
  {
    started = MotorDriver_SetCommand(
      (MotorDriverWheel)wheel,
      ChassisGroundTest_GetAssistCommand(
        s_targets.counts_per_second[wheel]));
  }
  if (!started)
  {
    MotorDriver_Disable();
    return false;
  }

  ChassisGroundTest_ResetProgress(s_start_totals, now_ms);
  for (uint32_t wheel = 0U; wheel < MOTOR_DRIVER_WHEEL_COUNT; ++wheel)
  {
    s_statistics[wheel].max_command_permille =
      GROUND_TEST_START_ASSIST_PERMILLE;
  }
  taskENTER_CRITICAL();
  if (s_state == GROUND_TEST_STATE_COUNTDOWN)
  {
    s_start_assist_start_ms = now_ms;
    s_start_assist_deadline_ms = now_ms + GROUND_TEST_START_ASSIST_MAX_MS;
    s_state = GROUND_TEST_STATE_START_ASSIST;
  }
  else
  {
    MotorDriver_Disable();
    started = false;
  }
  taskEXIT_CRITICAL();

  if (started)
  {
    DEBUG_LOG_WARN(
      "GROUND pivot startup assist: duty=%u%% max=%lu ms, early-exit enabled",
      (unsigned int)(GROUND_TEST_START_ASSIST_PERMILLE / 10),
      (unsigned long)GROUND_TEST_START_ASSIST_MAX_MS);
  }
  return started;
}

static void ChassisGroundTest_StartDrive(uint32_t now_ms)
{
  int64_t totals[MOTOR_DRIVER_WHEEL_COUNT];
  WheelSpeedState speeds[MOTOR_DRIVER_WHEEL_COUNT];
  const ChassisGroundTestDefinition *definition = &s_definitions[s_action];

  if (!ChassisGroundTest_GetSnapshots(totals, speeds))
  {
    ChassisGroundTest_SetAborted(GROUND_TEST_ABORT_SNAPSHOT_FAILURE,
                                 MOTOR_DRIVER_WHEEL_COUNT,
                                 CHASSIS_CONTROLLER_FAULT_NONE);
    return;
  }
  for (uint32_t wheel = 0U; wheel < MOTOR_DRIVER_WHEEL_COUNT; ++wheel)
  {
    if (!speeds[wheel].stopped)
    {
      ChassisGroundTest_SetAborted(GROUND_TEST_ABORT_NOT_STOPPED,
                                   (MotorDriverWheel)wheel,
                                   CHASSIS_CONTROLLER_FAULT_NONE);
      return;
    }
  }
  if (!ChassisKinematics_ToWheelTargets(
        &definition->command,
        WHEEL_SPEED_CONTROL_TARGET_LIMIT_CPS,
        &s_targets))
  {
    ChassisGroundTest_SetAborted(GROUND_TEST_ABORT_KINEMATICS_FAILURE,
                                 MOTOR_DRIVER_WHEEL_COUNT,
                                 CHASSIS_CONTROLLER_FAULT_NONE);
    return;
  }

  ChassisGroundTest_ResetStatistics();
  for (uint32_t wheel = 0U; wheel < MOTOR_DRIVER_WHEEL_COUNT; ++wheel)
  {
    s_start_totals[wheel] = totals[wheel];
    s_end_totals[wheel] = totals[wheel];
  }

  if (ChassisGroundTest_IsPivotAction())
  {
    if (!ChassisGroundTest_StartAssist(now_ms))
    {
      ChassisGroundTest_SetAborted(GROUND_TEST_ABORT_CONTROLLER_FAILURE,
                                   MOTOR_DRIVER_WHEEL_COUNT,
                                   CHASSIS_CONTROLLER_FAULT_OUTPUT);
    }
    return;
  }
  ChassisGroundTest_StartController(now_ms);
}

static void ChassisGroundTest_ProcessAssist(uint32_t now_ms)
{
  int64_t totals[MOTOR_DRIVER_WHEEL_COUNT];
  WheelSpeedState speeds[MOTOR_DRIVER_WHEEL_COUNT];
  uint32_t elapsed_ms;
  bool all_wheels_started;
  bool assist_complete;

  if (!ChassisGroundTest_GetSnapshots(totals, speeds))
  {
    ChassisGroundTest_SetAborted(GROUND_TEST_ABORT_SNAPSHOT_FAILURE,
                                 MOTOR_DRIVER_WHEEL_COUNT,
                                 CHASSIS_CONTROLLER_FAULT_NONE);
    return;
  }
  if (!ChassisGroundTest_CheckWheelMotion(now_ms, totals, speeds))
  {
    return;
  }
  elapsed_ms = now_ms - s_start_assist_start_ms;
  all_wheels_started = ChassisGroundTest_AllWheelsStarted(totals);
  assist_complete =
    ((elapsed_ms >= GROUND_TEST_START_ASSIST_MIN_MS) &&
     all_wheels_started) ||
    ChassisGroundTest_TimeReached(now_ms, s_start_assist_deadline_ms);

  if (assist_complete)
  {
    DEBUG_LOG_INFO(
      "GROUND pivot assist ended: elapsed=%lums reason=%s delta=%ld,%ld,%ld,%ld",
      (unsigned long)elapsed_ms,
      all_wheels_started ? "all wheels moving" : "time limit",
      (long)(totals[MOTOR_DRIVER_WHEEL_LF] -
             s_start_totals[MOTOR_DRIVER_WHEEL_LF]),
      (long)(totals[MOTOR_DRIVER_WHEEL_LR] -
             s_start_totals[MOTOR_DRIVER_WHEEL_LR]),
      (long)(totals[MOTOR_DRIVER_WHEEL_RF] -
             s_start_totals[MOTOR_DRIVER_WHEEL_RF]),
      (long)(totals[MOTOR_DRIVER_WHEEL_RR] -
             s_start_totals[MOTOR_DRIVER_WHEEL_RR]));
    ChassisGroundTest_StartController(now_ms);
  }
}

static void ChassisGroundTest_ProcessRunning(uint32_t now_ms)
{
  int64_t totals[MOTOR_DRIVER_WHEEL_COUNT];
  WheelSpeedState speeds[MOTOR_DRIVER_WHEEL_COUNT];
  ChassisControllerStatus status = { 0 };
  const ChassisGroundTestDefinition *definition = &s_definitions[s_action];
  const uint32_t elapsed_ms = now_ms - s_motion_start_ms;
  bool log_sample = false;

  if (ChassisGroundTest_TimeReached(now_ms, s_motion_deadline_ms))
  {
    if (!ChassisGroundTest_GetSnapshots(totals, speeds))
    {
      ChassisGroundTest_SetAborted(GROUND_TEST_ABORT_SNAPSHOT_FAILURE,
                                   MOTOR_DRIVER_WHEEL_COUNT,
                                   CHASSIS_CONTROLLER_FAULT_NONE);
      return;
    }
    for (uint32_t wheel = 0U; wheel < MOTOR_DRIVER_WHEEL_COUNT; ++wheel)
    {
      s_end_totals[wheel] = totals[wheel];
    }
    if (!ChassisGroundTest_ValidateTravel())
    {
      return;
    }
    ChassisController_Stop();
    ChassisGroundTest_LogResult();
    taskENTER_CRITICAL();
    if (s_state == GROUND_TEST_STATE_RUNNING)
    {
      s_stop_guard_deadline_ms = now_ms + GROUND_TEST_STOP_GUARD_MS;
      s_stop_timeout_ms = now_ms + GROUND_TEST_STOP_TIMEOUT_MS;
      s_state = GROUND_TEST_STATE_WAIT_STOP;
    }
    taskEXIT_CRITICAL();
    return;
  }

  if (!ChassisController_SetCommand(&definition->command, now_ms))
  {
    (void)ChassisController_GetStatus(&status);
    ChassisGroundTest_SetAborted(GROUND_TEST_ABORT_CONTROLLER_FAILURE,
                                 MOTOR_DRIVER_WHEEL_COUNT,
                                 status.fault);
    return;
  }
  ChassisController_Process(now_ms);
  if ((!ChassisController_GetStatus(&status)) || (!status.active) ||
      (status.fault != CHASSIS_CONTROLLER_FAULT_NONE))
  {
    ChassisGroundTest_SetAborted(GROUND_TEST_ABORT_CONTROLLER_FAILURE,
                                 MOTOR_DRIVER_WHEEL_COUNT,
                                 status.fault);
    return;
  }
  if (!ChassisGroundTest_GetSnapshots(totals, speeds))
  {
    ChassisGroundTest_SetAborted(GROUND_TEST_ABORT_SNAPSHOT_FAILURE,
                                 MOTOR_DRIVER_WHEEL_COUNT,
                                 CHASSIS_CONTROLLER_FAULT_NONE);
    return;
  }

  if (!ChassisGroundTest_CheckWheelMotion(now_ms, totals, speeds))
  {
    return;
  }

  ChassisGroundTest_RecordStatistics(elapsed_ms, &status);
  taskENTER_CRITICAL();
  if ((s_state == GROUND_TEST_STATE_RUNNING) &&
      ChassisGroundTest_TimeReached(now_ms, s_next_log_ms))
  {
    s_next_log_ms = now_ms + GROUND_TEST_LOG_PERIOD_MS;
    log_sample = true;
  }
  taskEXIT_CRITICAL();
  if (log_sample)
  {
    ChassisGroundTest_LogSample(elapsed_ms, &status);
  }
}

static void ChassisGroundTest_WaitForStop(uint32_t now_ms)
{
  int64_t totals[MOTOR_DRIVER_WHEEL_COUNT];
  WheelSpeedState speeds[MOTOR_DRIVER_WHEEL_COUNT];
  bool all_stopped = true;

  if (!ChassisGroundTest_GetSnapshots(totals, speeds))
  {
    ChassisGroundTest_SetAborted(GROUND_TEST_ABORT_SNAPSHOT_FAILURE,
                                 MOTOR_DRIVER_WHEEL_COUNT,
                                 CHASSIS_CONTROLLER_FAULT_NONE);
    return;
  }
  for (uint32_t wheel = 0U; wheel < MOTOR_DRIVER_WHEEL_COUNT; ++wheel)
  {
    if (!speeds[wheel].stopped)
    {
      all_stopped = false;
      break;
    }
  }

  if (ChassisGroundTest_TimeReached(now_ms, s_stop_timeout_ms))
  {
    ChassisGroundTest_SetAborted(GROUND_TEST_ABORT_STOP_TIMEOUT,
                                 MOTOR_DRIVER_WHEEL_COUNT,
                                 CHASSIS_CONTROLLER_FAULT_NONE);
  }
  else if (ChassisGroundTest_TimeReached(now_ms,
                                         s_stop_guard_deadline_ms) &&
           all_stopped)
  {
    taskENTER_CRITICAL();
    if (s_state == GROUND_TEST_STATE_WAIT_STOP)
    {
      s_state = GROUND_TEST_STATE_COMPLETED;
    }
    taskEXIT_CRITICAL();
  }
}

void ChassisGroundTest_Init(void)
{
  taskENTER_CRITICAL();
  ChassisController_Stop();
  s_state = GROUND_TEST_STATE_DISARMED;
  s_abort_reason = GROUND_TEST_ABORT_NONE;
  s_controller_fault = CHASSIS_CONTROLLER_FAULT_NONE;
  s_fault_wheel = MOTOR_DRIVER_WHEEL_COUNT;
  s_arm_deadline_ms = 0U;
  s_countdown_deadline_ms = 0U;
  s_drive_deadline_ms = 0U;
  s_overall_deadline_ms = 0U;
  s_arm_expired_notice = false;
  s_action = CHASSIS_GROUND_ACTION_COUNT;
  s_motion_start_ms = 0U;
  s_motion_deadline_ms = 0U;
  s_start_assist_start_ms = 0U;
  s_start_assist_deadline_ms = 0U;
  s_stop_guard_deadline_ms = 0U;
  s_stop_timeout_ms = 0U;
  s_next_log_ms = 0U;
  for (uint32_t wheel = 0U; wheel < MOTOR_DRIVER_WHEEL_COUNT; ++wheel)
  {
    s_start_totals[wheel] = 0;
    s_end_totals[wheel] = 0;
    s_progress_totals[wheel] = 0;
    s_last_progress_ms[wheel] = 0U;
  }
  ChassisGroundTest_ResetStatistics();
  taskEXIT_CRITICAL();
}

bool ChassisGroundTest_Arm(uint32_t now_ms)
{
  bool armed = false;

  if (!EmergencyStop_CanEnableMotor())
  {
    DEBUG_LOG_ERROR("GROUND arm rejected: ESTOP input/latch active");
    return false;
  }

  taskENTER_CRITICAL();
  if (s_state == GROUND_TEST_STATE_DISARMED)
  {
    ChassisController_Stop();
    s_action = CHASSIS_GROUND_ACTION_COUNT;
    s_arm_deadline_ms = now_ms + GROUND_TEST_ARM_WINDOW_MS;
    s_abort_reason = GROUND_TEST_ABORT_NONE;
    s_controller_fault = CHASSIS_CONTROLLER_FAULT_NONE;
    s_fault_wheel = MOTOR_DRIVER_WHEEL_COUNT;
    s_arm_expired_notice = false;
    s_state = GROUND_TEST_STATE_ARMED;
    armed = true;
  }
  taskEXIT_CRITICAL();

  if (armed)
  {
    DEBUG_LOG_WARN("GROUND armed 5 s: w/x=straight a/d=pivot j/l=rolling turn");
    DEBUG_LOG_WARN("Selected motion has a 2 s countdown and runs for only 1.5 s");
  }
  return armed;
}

bool ChassisGroundTest_Start(ChassisGroundTestAction action,
                             uint32_t now_ms)
{
  bool queued = false;

  if (((uint32_t)action >= (uint32_t)CHASSIS_GROUND_ACTION_COUNT) ||
      (!EmergencyStop_CanEnableMotor()))
  {
    DEBUG_LOG_ERROR("GROUND start rejected: invalid action or ESTOP active");
    return false;
  }

  taskENTER_CRITICAL();
  if ((s_state == GROUND_TEST_STATE_ARMED) &&
      (!ChassisGroundTest_TimeReached(now_ms, s_arm_deadline_ms)))
  {
    ChassisController_Stop();
    s_action = action;
    s_countdown_deadline_ms = now_ms + GROUND_TEST_COUNTDOWN_MS;
    s_overall_deadline_ms = now_ms + GROUND_TEST_OVERALL_TIMEOUT_MS;
    s_state = GROUND_TEST_STATE_COUNTDOWN;
    queued = true;
  }
  taskEXIT_CRITICAL();

  if (queued)
  {
    DEBUG_LOG_WARN("GROUND %s starts in 2 s; send 's' to cancel",
                   s_definitions[action].name);
  }
  else
  {
    DEBUG_LOG_WARN("GROUND start rejected: send 'b', then w/x/a/d/j/l within 5 s");
  }
  return queued;
}

bool ChassisGroundTest_IsArmed(void)
{
  return (s_state == GROUND_TEST_STATE_ARMED);
}

bool ChassisGroundTest_IsActive(void)
{
  return (s_state != GROUND_TEST_STATE_DISARMED);
}

void ChassisGroundTest_Process(uint32_t now_ms)
{
  ChassisGroundTestState state;
  ChassisGroundTestAbortReason abort_reason = GROUND_TEST_ABORT_NONE;
  ChassisControllerFault controller_fault = CHASSIS_CONTROLLER_FAULT_NONE;
  MotorDriverWheel fault_wheel = MOTOR_DRIVER_WHEEL_COUNT;
  bool completed_notice = false;
  bool aborted_notice = false;
  bool arm_expired_notice = false;

  taskENTER_CRITICAL();
  if (s_state == GROUND_TEST_STATE_COMPLETED)
  {
    s_state = GROUND_TEST_STATE_DISARMED;
    completed_notice = true;
  }
  else if (s_state == GROUND_TEST_STATE_ABORTED)
  {
    abort_reason = s_abort_reason;
    controller_fault = s_controller_fault;
    fault_wheel = s_fault_wheel;
    s_state = GROUND_TEST_STATE_DISARMED;
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
    DEBUG_LOG_INFO("GROUND test completed: %s; all motor outputs disabled",
                   s_definitions[s_action].name);
  }
  if (aborted_notice)
  {
    DEBUG_LOG_ERROR(
      "GROUND test aborted: wheel=%s reason=%s controller=%s; all motor outputs disabled",
      (fault_wheel < MOTOR_DRIVER_WHEEL_COUNT) ?
        s_wheel_names[fault_wheel] : "ALL",
      ChassisGroundTest_GetAbortReasonName(abort_reason),
      ChassisController_GetFaultName(controller_fault));
  }
  if (arm_expired_notice)
  {
    DEBUG_LOG_INFO("GROUND arm window expired; motors remain disabled");
  }

  switch (state)
  {
    case GROUND_TEST_STATE_COUNTDOWN:
      if (ChassisGroundTest_TimeReached(now_ms, s_countdown_deadline_ms))
      {
        ChassisGroundTest_StartDrive(now_ms);
      }
      break;

    case GROUND_TEST_STATE_RUNNING:
      ChassisGroundTest_ProcessRunning(now_ms);
      break;

    case GROUND_TEST_STATE_START_ASSIST:
      ChassisGroundTest_ProcessAssist(now_ms);
      break;

    case GROUND_TEST_STATE_WAIT_STOP:
      ChassisGroundTest_WaitForStop(now_ms);
      break;

    default:
      break;
  }
}

void ChassisGroundTest_Stop(void)
{
  bool was_active;

  taskENTER_CRITICAL();
  was_active = (s_state != GROUND_TEST_STATE_DISARMED);
  ChassisController_Stop();
  s_state = GROUND_TEST_STATE_DISARMED;
  s_abort_reason = GROUND_TEST_ABORT_NONE;
  s_controller_fault = CHASSIS_CONTROLLER_FAULT_NONE;
  s_fault_wheel = MOTOR_DRIVER_WHEEL_COUNT;
  s_arm_expired_notice = false;
  taskEXIT_CRITICAL();

  if (was_active)
  {
    DEBUG_LOG_WARN("GROUND test cancelled; all motor outputs disabled");
  }
}

void ChassisGroundTest_SafetyProcess(uint32_t now_ms)
{
  taskENTER_CRITICAL();
  if (EmergencyStop_IsLatched() &&
      (s_state != GROUND_TEST_STATE_DISARMED))
  {
    ChassisController_Stop();
    s_abort_reason = GROUND_TEST_ABORT_ESTOP;
    s_controller_fault = CHASSIS_CONTROLLER_FAULT_ESTOP;
    s_fault_wheel = MOTOR_DRIVER_WHEEL_COUNT;
    s_state = GROUND_TEST_STATE_ABORTED;
  }
  else if ((s_state == GROUND_TEST_STATE_ARMED) &&
           ChassisGroundTest_TimeReached(now_ms, s_arm_deadline_ms))
  {
    ChassisController_Stop();
    s_state = GROUND_TEST_STATE_DISARMED;
    s_arm_expired_notice = true;
  }
  else if (((s_state == GROUND_TEST_STATE_COUNTDOWN) ||
            (s_state == GROUND_TEST_STATE_START_ASSIST) ||
            (s_state == GROUND_TEST_STATE_RUNNING) ||
            (s_state == GROUND_TEST_STATE_WAIT_STOP)) &&
           ChassisGroundTest_TimeReached(now_ms, s_overall_deadline_ms))
  {
    ChassisController_Stop();
    s_abort_reason = GROUND_TEST_ABORT_OVERALL_TIMEOUT;
    s_controller_fault = CHASSIS_CONTROLLER_FAULT_NONE;
    s_fault_wheel = MOTOR_DRIVER_WHEEL_COUNT;
    s_state = GROUND_TEST_STATE_ABORTED;
  }
  else if ((s_state == GROUND_TEST_STATE_RUNNING) &&
           ChassisGroundTest_TimeReached(now_ms, s_drive_deadline_ms))
  {
    ChassisController_Stop();
    s_abort_reason = GROUND_TEST_ABORT_DRIVE_TIMEOUT;
    s_controller_fault = CHASSIS_CONTROLLER_FAULT_NONE;
    s_fault_wheel = MOTOR_DRIVER_WHEEL_COUNT;
    s_state = GROUND_TEST_STATE_ABORTED;
  }
  taskEXIT_CRITICAL();
}

#else

void ChassisGroundTest_Init(void)
{
}

bool ChassisGroundTest_Arm(uint32_t now_ms)
{
  (void)now_ms;
  return false;
}

bool ChassisGroundTest_Start(ChassisGroundTestAction action,
                             uint32_t now_ms)
{
  (void)action;
  (void)now_ms;
  return false;
}

bool ChassisGroundTest_IsArmed(void)
{
  return false;
}

bool ChassisGroundTest_IsActive(void)
{
  return false;
}

void ChassisGroundTest_Process(uint32_t now_ms)
{
  (void)now_ms;
}

void ChassisGroundTest_Stop(void)
{
}

void ChassisGroundTest_SafetyProcess(uint32_t now_ms)
{
  (void)now_ms;
}

#endif /* MOTOR_OPEN_LOOP_TEST_ENABLED */
