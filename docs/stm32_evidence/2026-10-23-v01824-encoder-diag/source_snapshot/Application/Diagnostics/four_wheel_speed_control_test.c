/**
 * @file four_wheel_speed_control_test.c
 * @brief Safe automatic four-wheel +500/stop/-500 counts/s PI validation.
 */

#include "four_wheel_speed_control_test.h"

#if MOTOR_OPEN_LOOP_TEST_ENABLED

#include <stddef.h>

#include "FreeRTOS.h"
#include "debug_log.h"
#include "emergency_stop.h"
#include "encoder_service.h"
#include "motor_driver.h"
#include "task.h"
#include "wheel_speed_control_config.h"
#include "wheel_speed_controller.h"
#include "wheel_speed_estimator.h"

#define FOUR_PI_ARM_WINDOW_MS          5000U
#define FOUR_PI_COUNTDOWN_MS           3000U
#define FOUR_PI_TARGET_CPS              500.0f
#define FOUR_PI_DRIVE_DURATION_MS       4000U
#define FOUR_PI_ANALYSIS_START_MS       2000U
#define FOUR_PI_NO_MOTION_TIMEOUT_MS    1000U
#define FOUR_PI_DRIVE_MARGIN_MS          500U
#define FOUR_PI_STOP_GUARD_MS            300U
#define FOUR_PI_STOP_TIMEOUT_MS         2500U
#define FOUR_PI_OVERALL_TIMEOUT_MS     20000U
#define FOUR_PI_LOG_PERIOD_MS            200U
#define FOUR_PI_MOVED_THRESHOLD_COUNTS     4
#define FOUR_PI_OVERSPEED_CPS           2200.0f

typedef enum
{
  FOUR_PI_STATE_DISARMED = 0,
  FOUR_PI_STATE_ARMED,
  FOUR_PI_STATE_COUNTDOWN,
  FOUR_PI_STATE_PREPARE_FWD,
  FOUR_PI_STATE_FWD_RUNNING,
  FOUR_PI_STATE_WAIT_BETWEEN,
  FOUR_PI_STATE_REV_RUNNING,
  FOUR_PI_STATE_WAIT_FINAL,
  FOUR_PI_STATE_COMPLETED,
  FOUR_PI_STATE_ABORTED
} FourWheelSpeedControlTestState;

typedef enum
{
  FOUR_PI_ABORT_NONE = 0,
  FOUR_PI_ABORT_OUTPUT_FAILURE,
  FOUR_PI_ABORT_SNAPSHOT_FAILURE,
  FOUR_PI_ABORT_CONTROLLER_FAILURE,
  FOUR_PI_ABORT_WRONG_DIRECTION,
  FOUR_PI_ABORT_NO_MOTION,
  FOUR_PI_ABORT_OVERSPEED,
  FOUR_PI_ABORT_STOP_TIMEOUT,
  FOUR_PI_ABORT_DRIVE_TIMEOUT,
  FOUR_PI_ABORT_OVERALL_TIMEOUT,
  FOUR_PI_ABORT_ESTOP
} FourWheelSpeedControlAbortReason;

typedef struct
{
  float sum_cps;
  float min_cps;
  float max_cps;
  float last_cps;
  uint32_t count;
  uint16_t max_command_permille;
} FourWheelSpeedStatistics;

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

static volatile FourWheelSpeedControlTestState s_state;
static volatile FourWheelSpeedControlAbortReason s_abort_reason;
static volatile MotorDriverWheel s_fault_wheel;
static volatile uint32_t s_phase_deadline_ms;
static volatile uint32_t s_drive_deadline_ms;
static volatile uint32_t s_overall_deadline_ms;
static volatile bool s_arm_expired_notice;
static int64_t s_phase_start_totals[MOTOR_DRIVER_WHEEL_COUNT];
static uint32_t s_phase_start_ms;
static uint32_t s_no_motion_deadline_ms;
static uint32_t s_stop_guard_deadline_ms;
static uint32_t s_stop_timeout_ms;
static uint32_t s_last_control_ms;
static uint32_t s_next_log_ms;
static FourWheelSpeedStatistics s_statistics[MOTOR_DRIVER_WHEEL_COUNT];
static float s_spread_sum_cps;
static float s_spread_max_cps;
static uint32_t s_spread_count;

static bool FourWheelSpeedControlTest_TimeReached(uint32_t now_ms,
                                                  uint32_t deadline_ms)
{
  return ((int32_t)(now_ms - deadline_ms) >= 0);
}

static float FourWheelSpeedControlTest_AbsFloat(float value)
{
  return (value < 0.0f) ? -value : value;
}

static int64_t FourWheelSpeedControlTest_AbsCount(int64_t value)
{
  return (value < 0) ? -value : value;
}

static int32_t FourWheelSpeedControlTest_RoundFloat(float value)
{
  return (int32_t)((value >= 0.0f) ? (value + 0.5f) : (value - 0.5f));
}

static const char *FourWheelSpeedControlTest_GetAbortReasonName(
  FourWheelSpeedControlAbortReason reason)
{
  switch (reason)
  {
    case FOUR_PI_ABORT_OUTPUT_FAILURE:
      return "motor output failure";
    case FOUR_PI_ABORT_SNAPSHOT_FAILURE:
      return "encoder/speed snapshot failure";
    case FOUR_PI_ABORT_CONTROLLER_FAILURE:
      return "controller update failure";
    case FOUR_PI_ABORT_WRONG_DIRECTION:
      return "encoder direction disagrees with target";
    case FOUR_PI_ABORT_NO_MOTION:
      return "no encoder motion";
    case FOUR_PI_ABORT_OVERSPEED:
      return "overspeed";
    case FOUR_PI_ABORT_STOP_TIMEOUT:
      return "wheel did not stop between phases";
    case FOUR_PI_ABORT_DRIVE_TIMEOUT:
      return "drive phase safety timeout";
    case FOUR_PI_ABORT_OVERALL_TIMEOUT:
      return "overall safety timeout";
    case FOUR_PI_ABORT_ESTOP:
      return "ESTOP latched";
    default:
      return "unspecified failure";
  }
}

static void FourWheelSpeedControlTest_ResetControllers(void)
{
  for (uint32_t wheel = 0U; wheel < MOTOR_DRIVER_WHEEL_COUNT; ++wheel)
  {
    (void)WheelSpeedController_Reset((MotorDriverWheel)wheel);
  }
}

static bool FourWheelSpeedControlTest_GetSnapshots(
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

static bool FourWheelSpeedControlTest_GetAllStopped(bool *all_stopped)
{
  int64_t totals[MOTOR_DRIVER_WHEEL_COUNT];
  WheelSpeedState speeds[MOTOR_DRIVER_WHEEL_COUNT];

  if ((all_stopped == NULL) ||
      (!FourWheelSpeedControlTest_GetSnapshots(totals, speeds)))
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

static void FourWheelSpeedControlTest_ResetStatistics(void)
{
  for (uint32_t wheel = 0U; wheel < MOTOR_DRIVER_WHEEL_COUNT; ++wheel)
  {
    s_statistics[wheel].sum_cps = 0.0f;
    s_statistics[wheel].min_cps = 0.0f;
    s_statistics[wheel].max_cps = 0.0f;
    s_statistics[wheel].last_cps = 0.0f;
    s_statistics[wheel].count = 0U;
    s_statistics[wheel].max_command_permille = 0U;
  }
  s_spread_sum_cps = 0.0f;
  s_spread_max_cps = 0.0f;
  s_spread_count = 0U;
}

static void FourWheelSpeedControlTest_RecordStatistics(
  uint32_t phase_elapsed_ms,
  int8_t direction,
  const WheelSpeedState speeds[MOTOR_DRIVER_WHEEL_COUNT],
  const int16_t commands[MOTOR_DRIVER_WHEEL_COUNT])
{
  float normalized_min = 0.0f;
  float normalized_max = 0.0f;

  for (uint32_t wheel = 0U; wheel < MOTOR_DRIVER_WHEEL_COUNT; ++wheel)
  {
    FourWheelSpeedStatistics *statistics = &s_statistics[wheel];
    const float measured = speeds[wheel].filtered_counts_per_second;
    const uint16_t command_magnitude = (uint16_t)
      ((commands[wheel] < 0) ? -commands[wheel] : commands[wheel]);

    statistics->last_cps = measured;
    if (command_magnitude > statistics->max_command_permille)
    {
      statistics->max_command_permille = command_magnitude;
    }

    if (phase_elapsed_ms < FOUR_PI_ANALYSIS_START_MS)
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

    {
      const float normalized = measured * (float)direction;
      if (wheel == 0U)
      {
        normalized_min = normalized;
        normalized_max = normalized;
      }
      else
      {
        if (normalized < normalized_min)
        {
          normalized_min = normalized;
        }
        if (normalized > normalized_max)
        {
          normalized_max = normalized;
        }
      }
    }
  }

  if (phase_elapsed_ms >= FOUR_PI_ANALYSIS_START_MS)
  {
    const float spread = normalized_max - normalized_min;
    s_spread_sum_cps += spread;
    if (spread > s_spread_max_cps)
    {
      s_spread_max_cps = spread;
    }
    ++s_spread_count;
  }
}

static void FourWheelSpeedControlTest_SetAborted(
  FourWheelSpeedControlAbortReason reason,
  MotorDriverWheel fault_wheel)
{
  taskENTER_CRITICAL();
  MotorDriver_Disable();
  FourWheelSpeedControlTest_ResetControllers();
  s_abort_reason = reason;
  s_fault_wheel = fault_wheel;
  s_state = FOUR_PI_STATE_ABORTED;
  taskEXIT_CRITICAL();
}

static void FourWheelSpeedControlTest_EnterStopWait(
  FourWheelSpeedControlTestState next_state,
  uint32_t now_ms)
{
  MotorDriver_Disable();
  FourWheelSpeedControlTest_ResetControllers();
  s_stop_guard_deadline_ms = now_ms + FOUR_PI_STOP_GUARD_MS;
  s_stop_timeout_ms = now_ms + FOUR_PI_STOP_TIMEOUT_MS;
  s_state = next_state;
}

static void FourWheelSpeedControlTest_LogSample(
  const char *phase,
  uint32_t phase_elapsed_ms,
  const WheelSpeedControllerState controls[MOTOR_DRIVER_WHEEL_COUNT])
{
  uint32_t limited_mask = 0U;

  for (uint32_t wheel = 0U; wheel < MOTOR_DRIVER_WHEEL_COUNT; ++wheel)
  {
    if (controls[wheel].limited)
    {
      limited_mask |= (1UL << wheel);
    }
  }

  DEBUG_LOG_INFO(
    "4PI sample: phase=%s t=%lu LF=%ld/%d LR=%ld/%d RF=%ld/%d RR=%ld/%d limited=0x%02lX",
    phase,
    (unsigned long)phase_elapsed_ms,
    (long)FourWheelSpeedControlTest_RoundFloat(
      controls[MOTOR_DRIVER_WHEEL_LF].measured_counts_per_second),
    (int)controls[MOTOR_DRIVER_WHEEL_LF].command_permille,
    (long)FourWheelSpeedControlTest_RoundFloat(
      controls[MOTOR_DRIVER_WHEEL_LR].measured_counts_per_second),
    (int)controls[MOTOR_DRIVER_WHEEL_LR].command_permille,
    (long)FourWheelSpeedControlTest_RoundFloat(
      controls[MOTOR_DRIVER_WHEEL_RF].measured_counts_per_second),
    (int)controls[MOTOR_DRIVER_WHEEL_RF].command_permille,
    (long)FourWheelSpeedControlTest_RoundFloat(
      controls[MOTOR_DRIVER_WHEEL_RR].measured_counts_per_second),
    (int)controls[MOTOR_DRIVER_WHEEL_RR].command_permille,
    (unsigned long)limited_mask);
}

static void FourWheelSpeedControlTest_LogPhaseResult(int8_t direction)
{
  const float target = (direction > 0) ?
    FOUR_PI_TARGET_CPS : -FOUR_PI_TARGET_CPS;

  for (uint32_t wheel = 0U; wheel < MOTOR_DRIVER_WHEEL_COUNT; ++wheel)
  {
    const FourWheelSpeedStatistics *statistics = &s_statistics[wheel];
    const float average = (statistics->count > 0U) ?
      (statistics->sum_cps / (float)statistics->count) : 0.0f;

    DEBUG_LOG_INFO(
      "4PI result: wheel=%s dir=%s target=%ld avg=%ld min=%ld max=%ld final=%ld avg_error=%ld max_pwm=%u samples=%lu",
      s_wheel_names[wheel],
      (direction > 0) ? "FWD" : "REV",
      (long)FourWheelSpeedControlTest_RoundFloat(target),
      (long)FourWheelSpeedControlTest_RoundFloat(average),
      (long)FourWheelSpeedControlTest_RoundFloat(statistics->min_cps),
      (long)FourWheelSpeedControlTest_RoundFloat(statistics->max_cps),
      (long)FourWheelSpeedControlTest_RoundFloat(statistics->last_cps),
      (long)FourWheelSpeedControlTest_RoundFloat(target - average),
      (unsigned int)statistics->max_command_permille,
      (unsigned long)statistics->count);
  }

  DEBUG_LOG_INFO(
    "4PI sync: dir=%s mean_spread=%ld cps max_spread=%ld cps samples=%lu",
    (direction > 0) ? "FWD" : "REV",
    (long)FourWheelSpeedControlTest_RoundFloat(
      (s_spread_count > 0U) ?
        (s_spread_sum_cps / (float)s_spread_count) : 0.0f),
    (long)FourWheelSpeedControlTest_RoundFloat(s_spread_max_cps),
    (unsigned long)s_spread_count);
}

static void FourWheelSpeedControlTest_StartDrive(int8_t direction,
                                                 uint32_t now_ms)
{
  int64_t totals[MOTOR_DRIVER_WHEEL_COUNT];
  WheelSpeedState speeds[MOTOR_DRIVER_WHEEL_COUNT];
  int16_t commands[MOTOR_DRIVER_WHEEL_COUNT];
  const float target = (direction > 0) ?
    FOUR_PI_TARGET_CPS : -FOUR_PI_TARGET_CPS;
  const FourWheelSpeedControlTestState required_state = (direction > 0) ?
    FOUR_PI_STATE_PREPARE_FWD : FOUR_PI_STATE_WAIT_BETWEEN;
  const FourWheelSpeedControlTestState running_state = (direction > 0) ?
    FOUR_PI_STATE_FWD_RUNNING : FOUR_PI_STATE_REV_RUNNING;
  bool started = false;

  if (!FourWheelSpeedControlTest_GetSnapshots(totals, speeds))
  {
    FourWheelSpeedControlTest_SetAborted(FOUR_PI_ABORT_SNAPSHOT_FAILURE,
                                         MOTOR_DRIVER_WHEEL_COUNT);
    return;
  }

  FourWheelSpeedControlTest_ResetControllers();
  for (uint32_t wheel = 0U; wheel < MOTOR_DRIVER_WHEEL_COUNT; ++wheel)
  {
    if (!WheelSpeedController_Update((MotorDriverWheel)wheel,
                                     target,
                                     speeds[wheel].filtered_counts_per_second,
                                     10U,
                                     &commands[wheel]))
    {
      FourWheelSpeedControlTest_SetAborted(FOUR_PI_ABORT_CONTROLLER_FAILURE,
                                           (MotorDriverWheel)wheel);
      return;
    }
  }

  taskENTER_CRITICAL();
  if (s_state == required_state)
  {
    FourWheelSpeedControlTest_ResetStatistics();
    for (uint32_t wheel = 0U; wheel < MOTOR_DRIVER_WHEEL_COUNT; ++wheel)
    {
      s_phase_start_totals[wheel] = totals[wheel];
    }
    s_phase_start_ms = now_ms;
    s_no_motion_deadline_ms = now_ms + FOUR_PI_NO_MOTION_TIMEOUT_MS;
    s_phase_deadline_ms = now_ms + FOUR_PI_DRIVE_DURATION_MS;
    s_drive_deadline_ms = now_ms + FOUR_PI_DRIVE_DURATION_MS +
      FOUR_PI_DRIVE_MARGIN_MS;
    s_last_control_ms = now_ms;
    s_next_log_ms = now_ms + FOUR_PI_LOG_PERIOD_MS;
    s_state = running_state;

    started = MotorDriver_Enable();
    for (uint32_t wheel = 0U;
         started && (wheel < MOTOR_DRIVER_WHEEL_COUNT);
         ++wheel)
    {
      started = MotorDriver_SetCommand((MotorDriverWheel)wheel,
                                       commands[wheel]);
    }
    if (!started)
    {
      MotorDriver_Disable();
      FourWheelSpeedControlTest_ResetControllers();
      s_abort_reason = EmergencyStop_IsLatched() ?
        FOUR_PI_ABORT_ESTOP : FOUR_PI_ABORT_OUTPUT_FAILURE;
      s_fault_wheel = MOTOR_DRIVER_WHEEL_COUNT;
      s_state = FOUR_PI_STATE_ABORTED;
    }
  }
  taskEXIT_CRITICAL();

  if (started)
  {
    DEBUG_LOG_WARN(
      "4PI phase started: all wheels dir=%s target=%ld cps duration=4 s output_limit=300/1000",
      (direction > 0) ? "FWD" : "REV",
      (long)FourWheelSpeedControlTest_RoundFloat(target));
  }
}

static void FourWheelSpeedControlTest_ProcessDrive(uint32_t now_ms,
                                                   int8_t direction)
{
  int64_t totals[MOTOR_DRIVER_WHEEL_COUNT];
  WheelSpeedState speeds[MOTOR_DRIVER_WHEEL_COUNT];
  WheelSpeedControllerState controls[MOTOR_DRIVER_WHEEL_COUNT];
  int16_t commands[MOTOR_DRIVER_WHEEL_COUNT];
  uint32_t elapsed_ms;
  uint32_t phase_elapsed_ms;
  bool phase_ended = false;
  bool log_sample = false;
  FourWheelSpeedControlAbortReason fault = FOUR_PI_ABORT_NONE;
  MotorDriverWheel fault_wheel = MOTOR_DRIVER_WHEEL_COUNT;
  const float target = (direction > 0) ?
    FOUR_PI_TARGET_CPS : -FOUR_PI_TARGET_CPS;
  const FourWheelSpeedControlTestState expected_state = (direction > 0) ?
    FOUR_PI_STATE_FWD_RUNNING : FOUR_PI_STATE_REV_RUNNING;

  if (!FourWheelSpeedControlTest_GetSnapshots(totals, speeds))
  {
    FourWheelSpeedControlTest_SetAborted(FOUR_PI_ABORT_SNAPSHOT_FAILURE,
                                         MOTOR_DRIVER_WHEEL_COUNT);
    return;
  }

  elapsed_ms = now_ms - s_last_control_ms;
  if (elapsed_ms == 0U)
  {
    return;
  }

  for (uint32_t wheel = 0U; wheel < MOTOR_DRIVER_WHEEL_COUNT; ++wheel)
  {
    if ((!WheelSpeedController_Update((MotorDriverWheel)wheel,
                                      target,
                                      speeds[wheel].filtered_counts_per_second,
                                      elapsed_ms,
                                      &commands[wheel])) ||
        (!WheelSpeedController_GetState((MotorDriverWheel)wheel,
                                        &controls[wheel])))
    {
      FourWheelSpeedControlTest_SetAborted(FOUR_PI_ABORT_CONTROLLER_FAILURE,
                                           (MotorDriverWheel)wheel);
      return;
    }
  }

  phase_elapsed_ms = now_ms - s_phase_start_ms;
  for (uint32_t wheel = 0U; wheel < MOTOR_DRIVER_WHEEL_COUNT; ++wheel)
  {
    const int64_t delta = totals[wheel] - s_phase_start_totals[wheel];

    if ((FourWheelSpeedControlTest_AbsCount(delta) >=
         FOUR_PI_MOVED_THRESHOLD_COUNTS) &&
        (((direction > 0) && (delta < 0)) ||
         ((direction < 0) && (delta > 0))))
    {
      fault = FOUR_PI_ABORT_WRONG_DIRECTION;
      fault_wheel = (MotorDriverWheel)wheel;
      break;
    }
    if (FourWheelSpeedControlTest_TimeReached(now_ms,
                                              s_no_motion_deadline_ms) &&
        (FourWheelSpeedControlTest_AbsCount(delta) <
         FOUR_PI_MOVED_THRESHOLD_COUNTS))
    {
      fault = FOUR_PI_ABORT_NO_MOTION;
      fault_wheel = (MotorDriverWheel)wheel;
      break;
    }
    if (FourWheelSpeedControlTest_AbsFloat(
          speeds[wheel].filtered_counts_per_second) > FOUR_PI_OVERSPEED_CPS)
    {
      fault = FOUR_PI_ABORT_OVERSPEED;
      fault_wheel = (MotorDriverWheel)wheel;
      break;
    }
  }

  taskENTER_CRITICAL();
  if (s_state == expected_state)
  {
    if (FourWheelSpeedControlTest_TimeReached(now_ms, s_phase_deadline_ms))
    {
      FourWheelSpeedControlTest_EnterStopWait(
        (direction > 0) ? FOUR_PI_STATE_WAIT_BETWEEN :
                          FOUR_PI_STATE_WAIT_FINAL,
        now_ms);
      phase_ended = true;
    }
    else if (fault != FOUR_PI_ABORT_NONE)
    {
      MotorDriver_Disable();
      FourWheelSpeedControlTest_ResetControllers();
      s_abort_reason = fault;
      s_fault_wheel = fault_wheel;
      s_state = FOUR_PI_STATE_ABORTED;
    }
    else
    {
      bool outputs_ok = true;

      for (uint32_t wheel = 0U;
           outputs_ok && (wheel < MOTOR_DRIVER_WHEEL_COUNT);
           ++wheel)
      {
        outputs_ok = MotorDriver_SetCommand((MotorDriverWheel)wheel,
                                            commands[wheel]);
        if (!outputs_ok)
        {
          fault_wheel = (MotorDriverWheel)wheel;
        }
      }

      if (!outputs_ok)
      {
        MotorDriver_Disable();
        FourWheelSpeedControlTest_ResetControllers();
        s_abort_reason = EmergencyStop_IsLatched() ?
          FOUR_PI_ABORT_ESTOP : FOUR_PI_ABORT_OUTPUT_FAILURE;
        s_fault_wheel = fault_wheel;
        s_state = FOUR_PI_STATE_ABORTED;
      }
      else
      {
        s_last_control_ms = now_ms;
        FourWheelSpeedControlTest_RecordStatistics(phase_elapsed_ms,
                                                   direction,
                                                   speeds,
                                                   commands);
        if (FourWheelSpeedControlTest_TimeReached(now_ms, s_next_log_ms))
        {
          s_next_log_ms = now_ms + FOUR_PI_LOG_PERIOD_MS;
          log_sample = true;
        }
      }
    }
  }
  taskEXIT_CRITICAL();

  if (phase_ended)
  {
    FourWheelSpeedControlTest_LogPhaseResult(direction);
  }
  else if (log_sample)
  {
    FourWheelSpeedControlTest_LogSample((direction > 0) ? "FWD" : "REV",
                                        phase_elapsed_ms,
                                        controls);
  }
}

static void FourWheelSpeedControlTest_WaitForStop(
  uint32_t now_ms,
  FourWheelSpeedControlTestState state)
{
  bool all_stopped;

  if (!FourWheelSpeedControlTest_GetAllStopped(&all_stopped))
  {
    FourWheelSpeedControlTest_SetAborted(FOUR_PI_ABORT_SNAPSHOT_FAILURE,
                                         MOTOR_DRIVER_WHEEL_COUNT);
  }
  else if (FourWheelSpeedControlTest_TimeReached(now_ms, s_stop_timeout_ms))
  {
    FourWheelSpeedControlTest_SetAborted(FOUR_PI_ABORT_STOP_TIMEOUT,
                                         MOTOR_DRIVER_WHEEL_COUNT);
  }
  else if (FourWheelSpeedControlTest_TimeReached(now_ms,
                                                 s_stop_guard_deadline_ms) &&
           all_stopped)
  {
    if (state == FOUR_PI_STATE_PREPARE_FWD)
    {
      FourWheelSpeedControlTest_StartDrive(1, now_ms);
    }
    else if (state == FOUR_PI_STATE_WAIT_BETWEEN)
    {
      FourWheelSpeedControlTest_StartDrive(-1, now_ms);
    }
    else if (state == FOUR_PI_STATE_WAIT_FINAL)
    {
      taskENTER_CRITICAL();
      if (s_state == FOUR_PI_STATE_WAIT_FINAL)
      {
        MotorDriver_Disable();
        s_state = FOUR_PI_STATE_COMPLETED;
      }
      taskEXIT_CRITICAL();
    }
  }
}

void FourWheelSpeedControlTest_Init(void)
{
  taskENTER_CRITICAL();
  MotorDriver_Disable();
  FourWheelSpeedControlTest_ResetControllers();
  s_state = FOUR_PI_STATE_DISARMED;
  s_abort_reason = FOUR_PI_ABORT_NONE;
  s_fault_wheel = MOTOR_DRIVER_WHEEL_COUNT;
  s_phase_deadline_ms = 0U;
  s_drive_deadline_ms = 0U;
  s_overall_deadline_ms = 0U;
  s_arm_expired_notice = false;
  s_phase_start_ms = 0U;
  s_no_motion_deadline_ms = 0U;
  s_stop_guard_deadline_ms = 0U;
  s_stop_timeout_ms = 0U;
  s_last_control_ms = 0U;
  s_next_log_ms = 0U;
  for (uint32_t wheel = 0U; wheel < MOTOR_DRIVER_WHEEL_COUNT; ++wheel)
  {
    s_phase_start_totals[wheel] = 0;
  }
  FourWheelSpeedControlTest_ResetStatistics();
  taskEXIT_CRITICAL();
}

bool FourWheelSpeedControlTest_Arm(uint32_t now_ms)
{
  bool armed = false;

  if (!EmergencyStop_CanEnableMotor())
  {
    DEBUG_LOG_ERROR("4PI arm rejected: ESTOP input/latch active");
    return false;
  }

  taskENTER_CRITICAL();
  if (s_state == FOUR_PI_STATE_DISARMED)
  {
    MotorDriver_Disable();
    FourWheelSpeedControlTest_ResetControllers();
    s_phase_deadline_ms = now_ms + FOUR_PI_ARM_WINDOW_MS;
    s_abort_reason = FOUR_PI_ABORT_NONE;
    s_fault_wheel = MOTOR_DRIVER_WHEEL_COUNT;
    s_arm_expired_notice = false;
    s_state = FOUR_PI_STATE_ARMED;
    armed = true;
  }
  taskEXIT_CRITICAL();

  if (armed)
  {
    DEBUG_LOG_WARN("4PI test armed for 5 s; send 'g' to start");
    DEBUG_LOG_WARN(
      "Sequence: all +500 cps/4 s, stop, all -500 cps/4 s, stop; limit=30%%");
  }
  return armed;
}

bool FourWheelSpeedControlTest_Start(uint32_t now_ms)
{
  bool queued = false;

  if (!EmergencyStop_CanEnableMotor())
  {
    DEBUG_LOG_ERROR("4PI start rejected: ESTOP input/latch active");
    return false;
  }

  taskENTER_CRITICAL();
  if ((s_state == FOUR_PI_STATE_ARMED) &&
      (!FourWheelSpeedControlTest_TimeReached(now_ms, s_phase_deadline_ms)))
  {
    MotorDriver_Disable();
    s_phase_deadline_ms = now_ms + FOUR_PI_COUNTDOWN_MS;
    s_overall_deadline_ms = now_ms + FOUR_PI_OVERALL_TIMEOUT_MS;
    s_state = FOUR_PI_STATE_COUNTDOWN;
    queued = true;
  }
  taskEXIT_CRITICAL();

  if (queued)
  {
    DEBUG_LOG_WARN("4PI test starts in 3 s; send 's' to cancel");
    DEBUG_LOG_WARN("Keep all four wheels lifted and keep 12 V physical cutoff ready");
    DEBUG_LOG_INFO(
      "4PI config: kp_x1000=%ld ki_x1000=%ld target=%ld cps output_limit=%d/1000 slew=%ld/1000/s",
      (long)FourWheelSpeedControlTest_RoundFloat(
        WHEEL_SPEED_CONTROL_KP_PERMILLE_PER_CPS * 1000.0f),
      (long)FourWheelSpeedControlTest_RoundFloat(
        WHEEL_SPEED_CONTROL_KI_PERMILLE_PER_CPS_SECOND * 1000.0f),
      (long)FourWheelSpeedControlTest_RoundFloat(FOUR_PI_TARGET_CPS),
      WHEEL_SPEED_CONTROL_OUTPUT_LIMIT_PERMILLE,
      (long)FourWheelSpeedControlTest_RoundFloat(
        WHEEL_SPEED_CONTROL_SLEW_PERMILLE_PER_SECOND));
  }
  else
  {
    DEBUG_LOG_WARN("4PI start rejected: send 'q', then 'g' within 5 s");
  }
  return queued;
}

bool FourWheelSpeedControlTest_IsActive(void)
{
  return (s_state != FOUR_PI_STATE_DISARMED);
}

void FourWheelSpeedControlTest_Process(uint32_t now_ms)
{
  FourWheelSpeedControlTestState state;
  FourWheelSpeedControlAbortReason abort_reason = FOUR_PI_ABORT_NONE;
  MotorDriverWheel fault_wheel = MOTOR_DRIVER_WHEEL_COUNT;
  bool completed_notice = false;
  bool aborted_notice = false;
  bool arm_expired_notice = false;

  taskENTER_CRITICAL();
  if (s_state == FOUR_PI_STATE_COMPLETED)
  {
    s_state = FOUR_PI_STATE_DISARMED;
    completed_notice = true;
  }
  else if (s_state == FOUR_PI_STATE_ABORTED)
  {
    abort_reason = s_abort_reason;
    fault_wheel = s_fault_wheel;
    s_state = FOUR_PI_STATE_DISARMED;
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
    DEBUG_LOG_INFO("4PI test completed; all motor outputs disabled");
  }
  if (aborted_notice)
  {
    DEBUG_LOG_ERROR(
      "4PI test aborted: wheel=%s reason=%s; all motor outputs disabled",
      (fault_wheel < MOTOR_DRIVER_WHEEL_COUNT) ?
        s_wheel_names[fault_wheel] : "ALL",
      FourWheelSpeedControlTest_GetAbortReasonName(abort_reason));
  }
  if (arm_expired_notice)
  {
    DEBUG_LOG_INFO("4PI arm window expired; motors remain disabled");
  }

  switch (state)
  {
    case FOUR_PI_STATE_COUNTDOWN:
      if (FourWheelSpeedControlTest_TimeReached(now_ms, s_phase_deadline_ms))
      {
        taskENTER_CRITICAL();
        if (s_state == FOUR_PI_STATE_COUNTDOWN)
        {
          FourWheelSpeedControlTest_EnterStopWait(FOUR_PI_STATE_PREPARE_FWD,
                                                  now_ms);
        }
        taskEXIT_CRITICAL();
      }
      break;

    case FOUR_PI_STATE_PREPARE_FWD:
    case FOUR_PI_STATE_WAIT_BETWEEN:
    case FOUR_PI_STATE_WAIT_FINAL:
      FourWheelSpeedControlTest_WaitForStop(now_ms, state);
      break;

    case FOUR_PI_STATE_FWD_RUNNING:
      FourWheelSpeedControlTest_ProcessDrive(now_ms, 1);
      break;

    case FOUR_PI_STATE_REV_RUNNING:
      FourWheelSpeedControlTest_ProcessDrive(now_ms, -1);
      break;

    default:
      break;
  }
}

void FourWheelSpeedControlTest_Stop(void)
{
  bool was_active;

  taskENTER_CRITICAL();
  was_active = (s_state != FOUR_PI_STATE_DISARMED);
  MotorDriver_Disable();
  FourWheelSpeedControlTest_ResetControllers();
  s_state = FOUR_PI_STATE_DISARMED;
  s_abort_reason = FOUR_PI_ABORT_NONE;
  s_fault_wheel = MOTOR_DRIVER_WHEEL_COUNT;
  s_arm_expired_notice = false;
  taskEXIT_CRITICAL();

  if (was_active)
  {
    DEBUG_LOG_WARN("4PI test cancelled; all motor outputs disabled");
  }
}

void FourWheelSpeedControlTest_SafetyProcess(uint32_t now_ms)
{
  taskENTER_CRITICAL();
  if (EmergencyStop_IsLatched() &&
      (s_state != FOUR_PI_STATE_DISARMED))
  {
    MotorDriver_Disable();
    FourWheelSpeedControlTest_ResetControllers();
    s_abort_reason = FOUR_PI_ABORT_ESTOP;
    s_fault_wheel = MOTOR_DRIVER_WHEEL_COUNT;
    s_state = FOUR_PI_STATE_ABORTED;
  }
  else if ((s_state == FOUR_PI_STATE_ARMED) &&
           FourWheelSpeedControlTest_TimeReached(now_ms,
                                                 s_phase_deadline_ms))
  {
    MotorDriver_Disable();
    s_state = FOUR_PI_STATE_DISARMED;
    s_arm_expired_notice = true;
  }
  else if (((s_state == FOUR_PI_STATE_COUNTDOWN) ||
            (s_state == FOUR_PI_STATE_PREPARE_FWD) ||
            (s_state == FOUR_PI_STATE_FWD_RUNNING) ||
            (s_state == FOUR_PI_STATE_WAIT_BETWEEN) ||
            (s_state == FOUR_PI_STATE_REV_RUNNING) ||
            (s_state == FOUR_PI_STATE_WAIT_FINAL)) &&
           FourWheelSpeedControlTest_TimeReached(now_ms,
                                                 s_overall_deadline_ms))
  {
    MotorDriver_Disable();
    FourWheelSpeedControlTest_ResetControllers();
    s_abort_reason = FOUR_PI_ABORT_OVERALL_TIMEOUT;
    s_fault_wheel = MOTOR_DRIVER_WHEEL_COUNT;
    s_state = FOUR_PI_STATE_ABORTED;
  }
  else if (((s_state == FOUR_PI_STATE_FWD_RUNNING) ||
            (s_state == FOUR_PI_STATE_REV_RUNNING)) &&
           FourWheelSpeedControlTest_TimeReached(now_ms,
                                                 s_drive_deadline_ms))
  {
    MotorDriver_Disable();
    FourWheelSpeedControlTest_ResetControllers();
    s_abort_reason = FOUR_PI_ABORT_DRIVE_TIMEOUT;
    s_fault_wheel = MOTOR_DRIVER_WHEEL_COUNT;
    s_state = FOUR_PI_STATE_ABORTED;
  }
  taskEXIT_CRITICAL();
}

#else

void FourWheelSpeedControlTest_Init(void)
{
}

bool FourWheelSpeedControlTest_Arm(uint32_t now_ms)
{
  (void)now_ms;
  return false;
}

bool FourWheelSpeedControlTest_Start(uint32_t now_ms)
{
  (void)now_ms;
  return false;
}

bool FourWheelSpeedControlTest_IsActive(void)
{
  return false;
}

void FourWheelSpeedControlTest_Process(uint32_t now_ms)
{
  (void)now_ms;
}

void FourWheelSpeedControlTest_Stop(void)
{
}

void FourWheelSpeedControlTest_SafetyProcess(uint32_t now_ms)
{
  (void)now_ms;
}

#endif /* MOTOR_OPEN_LOOP_TEST_ENABLED */
