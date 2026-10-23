/**
 * @file motor_speed_control_test.c
 * @brief Safe automatic +500/stop/-500 counts/s PI validation sequence.
 */

#include "motor_speed_control_test.h"

#if MOTOR_OPEN_LOOP_TEST_ENABLED

#include <stddef.h>

#include "FreeRTOS.h"
#include "debug_log.h"
#include "emergency_stop.h"
#include "encoder_service.h"
#include "task.h"
#include "wheel_speed_control_config.h"
#include "wheel_speed_controller.h"
#include "wheel_speed_estimator.h"

#define MOTOR_PI_ARM_WINDOW_MS          5000U
#define MOTOR_PI_COUNTDOWN_MS           3000U
#define MOTOR_PI_TARGET_CPS              500.0f
#define MOTOR_PI_DRIVE_DURATION_MS       4000U
#define MOTOR_PI_ANALYSIS_START_MS       2000U
#define MOTOR_PI_NO_MOTION_TIMEOUT_MS    1000U
#define MOTOR_PI_DRIVE_MARGIN_MS          500U
#define MOTOR_PI_STOP_GUARD_MS            300U
#define MOTOR_PI_STOP_TIMEOUT_MS         2500U
#define MOTOR_PI_OVERALL_TIMEOUT_MS     20000U
#define MOTOR_PI_LOG_PERIOD_MS            100U
#define MOTOR_PI_MOVED_THRESHOLD_COUNTS     4
#define MOTOR_PI_OVERSPEED_CPS           2200.0f

typedef enum
{
  MOTOR_PI_STATE_DISARMED = 0,
  MOTOR_PI_STATE_ARMED,
  MOTOR_PI_STATE_COUNTDOWN,
  MOTOR_PI_STATE_PREPARE_FWD,
  MOTOR_PI_STATE_FWD_RUNNING,
  MOTOR_PI_STATE_WAIT_BETWEEN,
  MOTOR_PI_STATE_REV_RUNNING,
  MOTOR_PI_STATE_WAIT_FINAL,
  MOTOR_PI_STATE_COMPLETED,
  MOTOR_PI_STATE_ABORTED
} MotorSpeedControlTestState;

typedef enum
{
  MOTOR_PI_ABORT_NONE = 0,
  MOTOR_PI_ABORT_OUTPUT_FAILURE,
  MOTOR_PI_ABORT_SNAPSHOT_FAILURE,
  MOTOR_PI_ABORT_CONTROLLER_FAILURE,
  MOTOR_PI_ABORT_WRONG_DIRECTION,
  MOTOR_PI_ABORT_NO_MOTION,
  MOTOR_PI_ABORT_OVERSPEED,
  MOTOR_PI_ABORT_STOP_TIMEOUT,
  MOTOR_PI_ABORT_DRIVE_TIMEOUT,
  MOTOR_PI_ABORT_OVERALL_TIMEOUT,
  MOTOR_PI_ABORT_ESTOP
} MotorSpeedControlAbortReason;

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

static volatile MotorSpeedControlTestState s_state;
static volatile MotorSpeedControlAbortReason s_abort_reason;
static volatile uint32_t s_phase_deadline_ms;
static volatile uint32_t s_drive_deadline_ms;
static volatile uint32_t s_overall_deadline_ms;
static volatile bool s_arm_expired_notice;
static MotorDriverWheel s_wheel;
static int64_t s_phase_start_total;
static uint32_t s_phase_start_ms;
static uint32_t s_no_motion_deadline_ms;
static uint32_t s_stop_guard_deadline_ms;
static uint32_t s_stop_timeout_ms;
static uint32_t s_last_control_ms;
static uint32_t s_next_log_ms;
static float s_sample_sum_cps;
static float s_sample_min_cps;
static float s_sample_max_cps;
static float s_last_measured_cps;
static uint32_t s_sample_count;
static uint16_t s_max_command_permille;

static bool MotorSpeedControlTest_IsWheelValid(MotorDriverWheel wheel)
{
  return ((uint32_t)wheel < (uint32_t)MOTOR_DRIVER_WHEEL_COUNT);
}

static bool MotorSpeedControlTest_TimeReached(uint32_t now_ms,
                                               uint32_t deadline_ms)
{
  return ((int32_t)(now_ms - deadline_ms) >= 0);
}

static float MotorSpeedControlTest_AbsFloat(float value)
{
  return (value < 0.0f) ? -value : value;
}

static int64_t MotorSpeedControlTest_AbsCount(int64_t value)
{
  return (value < 0) ? -value : value;
}

static int32_t MotorSpeedControlTest_RoundFloat(float value)
{
  return (int32_t)((value >= 0.0f) ? (value + 0.5f) : (value - 0.5f));
}

static const char *MotorSpeedControlTest_GetAbortReasonName(
  MotorSpeedControlAbortReason reason)
{
  switch (reason)
  {
    case MOTOR_PI_ABORT_OUTPUT_FAILURE:
      return "motor output failure";
    case MOTOR_PI_ABORT_SNAPSHOT_FAILURE:
      return "encoder/speed snapshot failure";
    case MOTOR_PI_ABORT_CONTROLLER_FAILURE:
      return "controller update failure";
    case MOTOR_PI_ABORT_WRONG_DIRECTION:
      return "encoder direction disagrees with target";
    case MOTOR_PI_ABORT_NO_MOTION:
      return "no encoder motion";
    case MOTOR_PI_ABORT_OVERSPEED:
      return "overspeed";
    case MOTOR_PI_ABORT_STOP_TIMEOUT:
      return "wheel did not stop between phases";
    case MOTOR_PI_ABORT_DRIVE_TIMEOUT:
      return "drive phase safety timeout";
    case MOTOR_PI_ABORT_OVERALL_TIMEOUT:
      return "overall safety timeout";
    case MOTOR_PI_ABORT_ESTOP:
      return "ESTOP latched";
    default:
      return "unspecified failure";
  }
}

static bool MotorSpeedControlTest_GetWheelTotal(int64_t *total)
{
  EncoderState encoder_states[ENCODER_WHEEL_COUNT];

  if ((total == NULL) || (!MotorSpeedControlTest_IsWheelValid(s_wheel)) ||
      (!EncoderService_GetSnapshot(encoder_states)))
  {
    return false;
  }

  *total = encoder_states[s_encoder_wheels[s_wheel]].accumulated_count;
  return true;
}

static bool MotorSpeedControlTest_GetWheelSpeed(WheelSpeedState *speed)
{
  WheelSpeedState speed_states[ENCODER_WHEEL_COUNT];

  if ((speed == NULL) || (!MotorSpeedControlTest_IsWheelValid(s_wheel)) ||
      (!WheelSpeedEstimator_GetSnapshot(speed_states)))
  {
    return false;
  }

  *speed = speed_states[s_encoder_wheels[s_wheel]];
  return speed->valid;
}

static void MotorSpeedControlTest_ResetStatistics(void)
{
  s_sample_sum_cps = 0.0f;
  s_sample_min_cps = 0.0f;
  s_sample_max_cps = 0.0f;
  s_last_measured_cps = 0.0f;
  s_sample_count = 0U;
  s_max_command_permille = 0U;
}

static void MotorSpeedControlTest_RecordStatistics(uint32_t phase_elapsed_ms,
                                                    float measured_cps,
                                                    int16_t command_permille)
{
  const uint16_t command_magnitude = (uint16_t)
    ((command_permille < 0) ? -command_permille : command_permille);

  s_last_measured_cps = measured_cps;
  if (command_magnitude > s_max_command_permille)
  {
    s_max_command_permille = command_magnitude;
  }

  if (phase_elapsed_ms < MOTOR_PI_ANALYSIS_START_MS)
  {
    return;
  }

  if (s_sample_count == 0U)
  {
    s_sample_min_cps = measured_cps;
    s_sample_max_cps = measured_cps;
  }
  else
  {
    if (measured_cps < s_sample_min_cps)
    {
      s_sample_min_cps = measured_cps;
    }
    if (measured_cps > s_sample_max_cps)
    {
      s_sample_max_cps = measured_cps;
    }
  }

  s_sample_sum_cps += measured_cps;
  ++s_sample_count;
}

static void MotorSpeedControlTest_SetAborted(
  MotorSpeedControlAbortReason reason)
{
  taskENTER_CRITICAL();
  MotorDriver_Disable();
  if (MotorSpeedControlTest_IsWheelValid(s_wheel))
  {
    (void)WheelSpeedController_Reset(s_wheel);
  }
  s_abort_reason = reason;
  s_state = MOTOR_PI_STATE_ABORTED;
  taskEXIT_CRITICAL();
}

static void MotorSpeedControlTest_EnterStopWait(
  MotorSpeedControlTestState next_state,
  uint32_t now_ms)
{
  MotorDriver_Disable();
  (void)WheelSpeedController_Reset(s_wheel);
  s_stop_guard_deadline_ms = now_ms + MOTOR_PI_STOP_GUARD_MS;
  s_stop_timeout_ms = now_ms + MOTOR_PI_STOP_TIMEOUT_MS;
  s_state = next_state;
}

static void MotorSpeedControlTest_LogSample(const char *phase,
                                            uint32_t phase_elapsed_ms,
                                            const WheelSpeedControllerState *control)
{
  DEBUG_LOG_INFO(
    "PI sample: wheel=%s phase=%s t=%lu target=%ld measured=%ld error=%ld ff=%ld p=%ld i=%ld pwm=%d limited=%u",
    s_wheel_names[s_wheel],
    phase,
    (unsigned long)phase_elapsed_ms,
    (long)MotorSpeedControlTest_RoundFloat(
      control->target_counts_per_second),
    (long)MotorSpeedControlTest_RoundFloat(
      control->measured_counts_per_second),
    (long)MotorSpeedControlTest_RoundFloat(
      control->error_counts_per_second),
    (long)MotorSpeedControlTest_RoundFloat(control->feedforward_permille),
    (long)MotorSpeedControlTest_RoundFloat(control->proportional_permille),
    (long)MotorSpeedControlTest_RoundFloat(control->integrator_permille),
    (int)control->command_permille,
    control->limited ? 1U : 0U);
}

static void MotorSpeedControlTest_LogPhaseResult(int8_t direction)
{
  const float target = (direction > 0) ?
    MOTOR_PI_TARGET_CPS : -MOTOR_PI_TARGET_CPS;
  const float average = (s_sample_count > 0U) ?
    (s_sample_sum_cps / (float)s_sample_count) : 0.0f;

  DEBUG_LOG_INFO(
    "PI result: wheel=%s dir=%s target=%ld avg=%ld min=%ld max=%ld final=%ld avg_error=%ld max_pwm=%u samples=%lu",
    s_wheel_names[s_wheel],
    (direction > 0) ? "FWD" : "REV",
    (long)MotorSpeedControlTest_RoundFloat(target),
    (long)MotorSpeedControlTest_RoundFloat(average),
    (long)MotorSpeedControlTest_RoundFloat(s_sample_min_cps),
    (long)MotorSpeedControlTest_RoundFloat(s_sample_max_cps),
    (long)MotorSpeedControlTest_RoundFloat(s_last_measured_cps),
    (long)MotorSpeedControlTest_RoundFloat(target - average),
    (unsigned int)s_max_command_permille,
    (unsigned long)s_sample_count);
}

static void MotorSpeedControlTest_StartDrive(int8_t direction,
                                             uint32_t now_ms)
{
  WheelSpeedState speed;
  int64_t start_total;
  int16_t initial_command = 0;
  const float target = (direction > 0) ?
    MOTOR_PI_TARGET_CPS : -MOTOR_PI_TARGET_CPS;
  const MotorSpeedControlTestState required_state = (direction > 0) ?
    MOTOR_PI_STATE_PREPARE_FWD : MOTOR_PI_STATE_WAIT_BETWEEN;
  const MotorSpeedControlTestState running_state = (direction > 0) ?
    MOTOR_PI_STATE_FWD_RUNNING : MOTOR_PI_STATE_REV_RUNNING;
  bool started = false;

  if ((!MotorSpeedControlTest_GetWheelTotal(&start_total)) ||
      (!MotorSpeedControlTest_GetWheelSpeed(&speed)))
  {
    MotorSpeedControlTest_SetAborted(MOTOR_PI_ABORT_SNAPSHOT_FAILURE);
    return;
  }

  (void)WheelSpeedController_Reset(s_wheel);
  if (!WheelSpeedController_Update(s_wheel,
                                   target,
                                   speed.filtered_counts_per_second,
                                   10U,
                                   &initial_command))
  {
    MotorSpeedControlTest_SetAborted(MOTOR_PI_ABORT_CONTROLLER_FAILURE);
    return;
  }

  taskENTER_CRITICAL();
  if (s_state == required_state)
  {
    MotorSpeedControlTest_ResetStatistics();
    s_phase_start_total = start_total;
    s_phase_start_ms = now_ms;
    s_no_motion_deadline_ms = now_ms + MOTOR_PI_NO_MOTION_TIMEOUT_MS;
    s_phase_deadline_ms = now_ms + MOTOR_PI_DRIVE_DURATION_MS;
    s_drive_deadline_ms = now_ms + MOTOR_PI_DRIVE_DURATION_MS +
      MOTOR_PI_DRIVE_MARGIN_MS;
    s_last_control_ms = now_ms;
    s_next_log_ms = now_ms + MOTOR_PI_LOG_PERIOD_MS;
    s_state = running_state;

    started = MotorDriver_Enable() &&
              MotorDriver_SetCommand(s_wheel, initial_command);
    if (!started)
    {
      MotorDriver_Disable();
      (void)WheelSpeedController_Reset(s_wheel);
      s_abort_reason = EmergencyStop_IsLatched() ?
        MOTOR_PI_ABORT_ESTOP : MOTOR_PI_ABORT_OUTPUT_FAILURE;
      s_state = MOTOR_PI_STATE_ABORTED;
    }
  }
  taskEXIT_CRITICAL();

  if (started)
  {
    DEBUG_LOG_WARN("PI phase started: wheel=%s dir=%s target=%ld cps duration=4 s output_limit=300/1000",
                   s_wheel_names[s_wheel],
                   (direction > 0) ? "FWD" : "REV",
                   (long)MotorSpeedControlTest_RoundFloat(target));
  }
}

static void MotorSpeedControlTest_ProcessDrive(uint32_t now_ms,
                                               int8_t direction)
{
  WheelSpeedState speed;
  WheelSpeedControllerState control;
  int64_t current_total;
  int64_t delta;
  int16_t command;
  uint32_t elapsed_ms;
  uint32_t phase_elapsed_ms;
  bool phase_ended = false;
  bool log_sample = false;
  const float target = (direction > 0) ?
    MOTOR_PI_TARGET_CPS : -MOTOR_PI_TARGET_CPS;
  const MotorSpeedControlTestState expected_state = (direction > 0) ?
    MOTOR_PI_STATE_FWD_RUNNING : MOTOR_PI_STATE_REV_RUNNING;

  if ((!MotorSpeedControlTest_GetWheelTotal(&current_total)) ||
      (!MotorSpeedControlTest_GetWheelSpeed(&speed)))
  {
    MotorSpeedControlTest_SetAborted(MOTOR_PI_ABORT_SNAPSHOT_FAILURE);
    return;
  }

  elapsed_ms = now_ms - s_last_control_ms;
  if (elapsed_ms == 0U)
  {
    return;
  }

  if (!WheelSpeedController_Update(s_wheel,
                                   target,
                                   speed.filtered_counts_per_second,
                                   elapsed_ms,
                                   &command) ||
      (!WheelSpeedController_GetState(s_wheel, &control)))
  {
    MotorSpeedControlTest_SetAborted(MOTOR_PI_ABORT_CONTROLLER_FAILURE);
    return;
  }

  delta = current_total - s_phase_start_total;
  phase_elapsed_ms = now_ms - s_phase_start_ms;

  taskENTER_CRITICAL();
  if (s_state == expected_state)
  {
    if (MotorSpeedControlTest_TimeReached(now_ms, s_phase_deadline_ms))
    {
      MotorSpeedControlTest_EnterStopWait(
        (direction > 0) ? MOTOR_PI_STATE_WAIT_BETWEEN :
                          MOTOR_PI_STATE_WAIT_FINAL,
        now_ms);
      phase_ended = true;
    }
    else if ((MotorSpeedControlTest_AbsCount(delta) >=
              MOTOR_PI_MOVED_THRESHOLD_COUNTS) &&
             (((direction > 0) && (delta < 0)) ||
              ((direction < 0) && (delta > 0))))
    {
      MotorDriver_Disable();
      (void)WheelSpeedController_Reset(s_wheel);
      s_abort_reason = MOTOR_PI_ABORT_WRONG_DIRECTION;
      s_state = MOTOR_PI_STATE_ABORTED;
    }
    else if (MotorSpeedControlTest_TimeReached(now_ms,
                                               s_no_motion_deadline_ms) &&
             (MotorSpeedControlTest_AbsCount(delta) <
              MOTOR_PI_MOVED_THRESHOLD_COUNTS))
    {
      MotorDriver_Disable();
      (void)WheelSpeedController_Reset(s_wheel);
      s_abort_reason = MOTOR_PI_ABORT_NO_MOTION;
      s_state = MOTOR_PI_STATE_ABORTED;
    }
    else if (MotorSpeedControlTest_AbsFloat(
               speed.filtered_counts_per_second) > MOTOR_PI_OVERSPEED_CPS)
    {
      MotorDriver_Disable();
      (void)WheelSpeedController_Reset(s_wheel);
      s_abort_reason = MOTOR_PI_ABORT_OVERSPEED;
      s_state = MOTOR_PI_STATE_ABORTED;
    }
    else if (!MotorDriver_SetCommand(s_wheel, command))
    {
      MotorDriver_Disable();
      (void)WheelSpeedController_Reset(s_wheel);
      s_abort_reason = EmergencyStop_IsLatched() ?
        MOTOR_PI_ABORT_ESTOP : MOTOR_PI_ABORT_OUTPUT_FAILURE;
      s_state = MOTOR_PI_STATE_ABORTED;
    }
    else
    {
      s_last_control_ms = now_ms;
      MotorSpeedControlTest_RecordStatistics(
        phase_elapsed_ms,
        speed.filtered_counts_per_second,
        command);
      if (MotorSpeedControlTest_TimeReached(now_ms, s_next_log_ms))
      {
        s_next_log_ms = now_ms + MOTOR_PI_LOG_PERIOD_MS;
        log_sample = true;
      }
    }
  }
  taskEXIT_CRITICAL();

  if (phase_ended)
  {
    MotorSpeedControlTest_LogPhaseResult(direction);
  }
  else if (log_sample)
  {
    MotorSpeedControlTest_LogSample((direction > 0) ? "FWD" : "REV",
                                    phase_elapsed_ms,
                                    &control);
  }
}

static void MotorSpeedControlTest_WaitForStop(uint32_t now_ms,
                                              MotorSpeedControlTestState state)
{
  WheelSpeedState speed;

  if (!MotorSpeedControlTest_GetWheelSpeed(&speed))
  {
    MotorSpeedControlTest_SetAborted(MOTOR_PI_ABORT_SNAPSHOT_FAILURE);
    return;
  }

  if (MotorSpeedControlTest_TimeReached(now_ms, s_stop_timeout_ms))
  {
    MotorSpeedControlTest_SetAborted(MOTOR_PI_ABORT_STOP_TIMEOUT);
  }
  else if (MotorSpeedControlTest_TimeReached(now_ms,
                                             s_stop_guard_deadline_ms) &&
           speed.stopped)
  {
    if (state == MOTOR_PI_STATE_PREPARE_FWD)
    {
      MotorSpeedControlTest_StartDrive(1, now_ms);
    }
    else if (state == MOTOR_PI_STATE_WAIT_BETWEEN)
    {
      MotorSpeedControlTest_StartDrive(-1, now_ms);
    }
    else if (state == MOTOR_PI_STATE_WAIT_FINAL)
    {
      taskENTER_CRITICAL();
      if (s_state == MOTOR_PI_STATE_WAIT_FINAL)
      {
        MotorDriver_Disable();
        s_state = MOTOR_PI_STATE_COMPLETED;
      }
      taskEXIT_CRITICAL();
    }
  }
}

void MotorSpeedControlTest_Init(void)
{
  taskENTER_CRITICAL();
  MotorDriver_Disable();
  WheelSpeedController_Init();
  s_state = MOTOR_PI_STATE_DISARMED;
  s_abort_reason = MOTOR_PI_ABORT_NONE;
  s_phase_deadline_ms = 0U;
  s_drive_deadline_ms = 0U;
  s_overall_deadline_ms = 0U;
  s_arm_expired_notice = false;
  s_wheel = MOTOR_DRIVER_WHEEL_COUNT;
  s_phase_start_total = 0;
  s_phase_start_ms = 0U;
  s_no_motion_deadline_ms = 0U;
  s_stop_guard_deadline_ms = 0U;
  s_stop_timeout_ms = 0U;
  s_last_control_ms = 0U;
  s_next_log_ms = 0U;
  MotorSpeedControlTest_ResetStatistics();
  taskEXIT_CRITICAL();
}

bool MotorSpeedControlTest_Arm(MotorDriverWheel wheel, uint32_t now_ms)
{
  bool armed = false;

  if ((!MotorSpeedControlTest_IsWheelValid(wheel)) ||
      (!EmergencyStop_CanEnableMotor()))
  {
    DEBUG_LOG_ERROR("PI arm rejected: invalid wheel or ESTOP active");
    return false;
  }

  taskENTER_CRITICAL();
  if (s_state == MOTOR_PI_STATE_DISARMED)
  {
    MotorDriver_Disable();
    (void)WheelSpeedController_Reset(wheel);
    s_wheel = wheel;
    s_phase_deadline_ms = now_ms + MOTOR_PI_ARM_WINDOW_MS;
    s_abort_reason = MOTOR_PI_ABORT_NONE;
    s_arm_expired_notice = false;
    s_state = MOTOR_PI_STATE_ARMED;
    armed = true;
  }
  taskEXIT_CRITICAL();

  if (armed)
  {
    DEBUG_LOG_WARN("PI test armed: wheel=%s for 5 s; send 'g' to start",
                   s_wheel_names[wheel]);
    DEBUG_LOG_WARN("Sequence: +500 cps/4 s, stop, -500 cps/4 s, stop; limit=30%%");
  }
  return armed;
}

bool MotorSpeedControlTest_Start(uint32_t now_ms)
{
  bool queued = false;

  if (!EmergencyStop_CanEnableMotor())
  {
    DEBUG_LOG_ERROR("PI start rejected: ESTOP input/latch active");
    return false;
  }

  taskENTER_CRITICAL();
  if ((s_state == MOTOR_PI_STATE_ARMED) &&
      (!MotorSpeedControlTest_TimeReached(now_ms, s_phase_deadline_ms)))
  {
    MotorDriver_Disable();
    s_phase_deadline_ms = now_ms + MOTOR_PI_COUNTDOWN_MS;
    s_overall_deadline_ms = now_ms + MOTOR_PI_OVERALL_TIMEOUT_MS;
    s_state = MOTOR_PI_STATE_COUNTDOWN;
    queued = true;
  }
  taskEXIT_CRITICAL();

  if (queued)
  {
    DEBUG_LOG_WARN("PI test for %s starts in 3 s; send 's' to cancel",
                   s_wheel_names[s_wheel]);
    DEBUG_LOG_WARN("Keep every wheel lifted and keep 12 V physical cutoff ready");
    DEBUG_LOG_INFO("PI config: kp_x1000=%ld ki_x1000=%ld target=%ld cps output_limit=%d/1000 slew=%ld/1000/s",
                   (long)MotorSpeedControlTest_RoundFloat(
                     WHEEL_SPEED_CONTROL_KP_PERMILLE_PER_CPS * 1000.0f),
                   (long)MotorSpeedControlTest_RoundFloat(
                     WHEEL_SPEED_CONTROL_KI_PERMILLE_PER_CPS_SECOND *
                     1000.0f),
                   (long)MotorSpeedControlTest_RoundFloat(
                     MOTOR_PI_TARGET_CPS),
                   WHEEL_SPEED_CONTROL_OUTPUT_LIMIT_PERMILLE,
                   (long)MotorSpeedControlTest_RoundFloat(
                     WHEEL_SPEED_CONTROL_SLEW_PERMILLE_PER_SECOND));
  }
  else
  {
    DEBUG_LOG_WARN("PI start rejected: select wheel, send 'p', then 'g' within 5 s");
  }
  return queued;
}

bool MotorSpeedControlTest_IsActive(void)
{
  return (s_state != MOTOR_PI_STATE_DISARMED);
}

void MotorSpeedControlTest_Process(uint32_t now_ms)
{
  MotorSpeedControlTestState state;
  MotorSpeedControlAbortReason abort_reason = MOTOR_PI_ABORT_NONE;
  bool completed_notice = false;
  bool aborted_notice = false;
  bool arm_expired_notice = false;

  taskENTER_CRITICAL();
  if (s_state == MOTOR_PI_STATE_COMPLETED)
  {
    s_state = MOTOR_PI_STATE_DISARMED;
    completed_notice = true;
  }
  else if (s_state == MOTOR_PI_STATE_ABORTED)
  {
    abort_reason = s_abort_reason;
    s_state = MOTOR_PI_STATE_DISARMED;
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
    DEBUG_LOG_INFO("PI test completed: wheel=%s; motor outputs disabled",
                   s_wheel_names[s_wheel]);
  }
  if (aborted_notice)
  {
    DEBUG_LOG_ERROR("PI test aborted: wheel=%s reason=%s; motor outputs disabled",
                    s_wheel_names[s_wheel],
                    MotorSpeedControlTest_GetAbortReasonName(abort_reason));
  }
  if (arm_expired_notice)
  {
    DEBUG_LOG_INFO("PI arm window expired; motors remain disabled");
  }

  switch (state)
  {
    case MOTOR_PI_STATE_COUNTDOWN:
      if (MotorSpeedControlTest_TimeReached(now_ms, s_phase_deadline_ms))
      {
        taskENTER_CRITICAL();
        if (s_state == MOTOR_PI_STATE_COUNTDOWN)
        {
          MotorSpeedControlTest_EnterStopWait(MOTOR_PI_STATE_PREPARE_FWD,
                                              now_ms);
        }
        taskEXIT_CRITICAL();
      }
      break;

    case MOTOR_PI_STATE_PREPARE_FWD:
    case MOTOR_PI_STATE_WAIT_BETWEEN:
    case MOTOR_PI_STATE_WAIT_FINAL:
      MotorSpeedControlTest_WaitForStop(now_ms, state);
      break;

    case MOTOR_PI_STATE_FWD_RUNNING:
      MotorSpeedControlTest_ProcessDrive(now_ms, 1);
      break;

    case MOTOR_PI_STATE_REV_RUNNING:
      MotorSpeedControlTest_ProcessDrive(now_ms, -1);
      break;

    default:
      break;
  }
}

void MotorSpeedControlTest_Stop(void)
{
  bool was_active;

  taskENTER_CRITICAL();
  was_active = (s_state != MOTOR_PI_STATE_DISARMED);
  MotorDriver_Disable();
  if (MotorSpeedControlTest_IsWheelValid(s_wheel))
  {
    (void)WheelSpeedController_Reset(s_wheel);
  }
  s_state = MOTOR_PI_STATE_DISARMED;
  s_abort_reason = MOTOR_PI_ABORT_NONE;
  s_arm_expired_notice = false;
  taskEXIT_CRITICAL();

  if (was_active)
  {
    DEBUG_LOG_WARN("PI test cancelled; all motor outputs disabled");
  }
}

void MotorSpeedControlTest_SafetyProcess(uint32_t now_ms)
{
  taskENTER_CRITICAL();
  if (EmergencyStop_IsLatched() &&
      (s_state != MOTOR_PI_STATE_DISARMED))
  {
    MotorDriver_Disable();
    if (MotorSpeedControlTest_IsWheelValid(s_wheel))
    {
      (void)WheelSpeedController_Reset(s_wheel);
    }
    s_abort_reason = MOTOR_PI_ABORT_ESTOP;
    s_state = MOTOR_PI_STATE_ABORTED;
  }
  else if ((s_state == MOTOR_PI_STATE_ARMED) &&
           MotorSpeedControlTest_TimeReached(now_ms, s_phase_deadline_ms))
  {
    MotorDriver_Disable();
    s_state = MOTOR_PI_STATE_DISARMED;
    s_arm_expired_notice = true;
  }
  else if (((s_state == MOTOR_PI_STATE_COUNTDOWN) ||
            (s_state == MOTOR_PI_STATE_PREPARE_FWD) ||
            (s_state == MOTOR_PI_STATE_FWD_RUNNING) ||
            (s_state == MOTOR_PI_STATE_WAIT_BETWEEN) ||
            (s_state == MOTOR_PI_STATE_REV_RUNNING) ||
            (s_state == MOTOR_PI_STATE_WAIT_FINAL)) &&
           MotorSpeedControlTest_TimeReached(now_ms, s_overall_deadline_ms))
  {
    MotorDriver_Disable();
    (void)WheelSpeedController_Reset(s_wheel);
    s_abort_reason = MOTOR_PI_ABORT_OVERALL_TIMEOUT;
    s_state = MOTOR_PI_STATE_ABORTED;
  }
  else if (((s_state == MOTOR_PI_STATE_FWD_RUNNING) ||
            (s_state == MOTOR_PI_STATE_REV_RUNNING)) &&
           MotorSpeedControlTest_TimeReached(now_ms, s_drive_deadline_ms))
  {
    MotorDriver_Disable();
    (void)WheelSpeedController_Reset(s_wheel);
    s_abort_reason = MOTOR_PI_ABORT_DRIVE_TIMEOUT;
    s_state = MOTOR_PI_STATE_ABORTED;
  }
  taskEXIT_CRITICAL();
}

#else

void MotorSpeedControlTest_Init(void)
{
}

bool MotorSpeedControlTest_Arm(MotorDriverWheel wheel, uint32_t now_ms)
{
  (void)wheel;
  (void)now_ms;
  return false;
}

bool MotorSpeedControlTest_Start(uint32_t now_ms)
{
  (void)now_ms;
  return false;
}

bool MotorSpeedControlTest_IsActive(void)
{
  return false;
}

void MotorSpeedControlTest_Process(uint32_t now_ms)
{
  (void)now_ms;
}

void MotorSpeedControlTest_Stop(void)
{
}

void MotorSpeedControlTest_SafetyProcess(uint32_t now_ms)
{
  (void)now_ms;
}

#endif /* MOTOR_OPEN_LOOP_TEST_ENABLED */
