/**
 * @file motor_characterization_test.c
 * @brief Safe, Debug-only characterization of each wheel's open-loop speed.
 */

#include "motor_characterization_test.h"

#if MOTOR_OPEN_LOOP_TEST_ENABLED

#include <stddef.h>

#include "FreeRTOS.h"
#include "chassis_config.h"
#include "debug_log.h"
#include "emergency_stop.h"
#include "encoder_service.h"
#include "motor_driver.h"
#include "task.h"
#include "wheel_speed_estimator.h"

#define MOTOR_CHAR_ARM_WINDOW_MS             5000U
#define MOTOR_CHAR_COUNTDOWN_MS              3000U
#define MOTOR_CHAR_START_CHECK_MS             400U
#define MOTOR_CHAR_SETTLE_MS                  600U
#define MOTOR_CHAR_MEASURE_MS                1000U
#define MOTOR_CHAR_STOP_GUARD_MS              300U
#define MOTOR_CHAR_STOP_TIMEOUT_MS           2500U
#define MOTOR_CHAR_DRIVE_MARGIN_MS            200U
#define MOTOR_CHAR_OVERALL_TIMEOUT_MS      120000U
#define MOTOR_CHAR_MOVED_THRESHOLD_COUNTS       4
#define MOTOR_CHAR_MIN_VALID_CPS                26U

typedef enum
{
  MOTOR_CHAR_STATE_DISARMED = 0,
  MOTOR_CHAR_STATE_ARMED,
  MOTOR_CHAR_STATE_COUNTDOWN,
  MOTOR_CHAR_STATE_PREPARE_STOP,
  MOTOR_CHAR_STATE_START_CHECK,
  MOTOR_CHAR_STATE_SETTLING,
  MOTOR_CHAR_STATE_MEASURING,
  MOTOR_CHAR_STATE_WAIT_STOP,
  MOTOR_CHAR_STATE_COMPLETED,
  MOTOR_CHAR_STATE_ABORTED
} MotorCharacterizationState;

typedef enum
{
  MOTOR_CHAR_ABORT_NONE = 0,
  MOTOR_CHAR_ABORT_OUTPUT_FAILURE,
  MOTOR_CHAR_ABORT_ENCODER_FAILURE,
  MOTOR_CHAR_ABORT_WRONG_DIRECTION,
  MOTOR_CHAR_ABORT_STOP_TIMEOUT,
  MOTOR_CHAR_ABORT_POINT_TIMEOUT,
  MOTOR_CHAR_ABORT_OVERALL_TIMEOUT,
  MOTOR_CHAR_ABORT_ESTOP
} MotorCharacterizationAbortReason;

static const int16_t s_duty_levels_permille[] = {
  50, 75, 100, 125, 150, 200, 250, 300
};

static const MotorDriverWheel s_motor_wheels[MOTOR_DRIVER_WHEEL_COUNT] = {
  MOTOR_DRIVER_WHEEL_LF,
  MOTOR_DRIVER_WHEEL_LR,
  MOTOR_DRIVER_WHEEL_RF,
  MOTOR_DRIVER_WHEEL_RR
};

static const EncoderWheel s_encoder_wheels[MOTOR_DRIVER_WHEEL_COUNT] = {
  ENCODER_WHEEL_LF,
  ENCODER_WHEEL_LR,
  ENCODER_WHEEL_RF,
  ENCODER_WHEEL_RR
};

static const char *const s_wheel_names[MOTOR_DRIVER_WHEEL_COUNT] = {
  "LF", "LR", "RF", "RR"
};

static volatile MotorCharacterizationState s_state;
static volatile MotorCharacterizationAbortReason s_abort_reason;
static volatile uint32_t s_phase_deadline_ms;
static volatile uint32_t s_drive_deadline_ms;
static volatile uint32_t s_overall_deadline_ms;
static volatile bool s_arm_expired_notice;
static uint32_t s_wheel_index;
static uint32_t s_duty_index;
static int8_t s_direction;
static int64_t s_point_start_total;
static uint32_t s_point_start_ms;
static int64_t s_measurement_start_total;
static uint32_t s_measurement_start_ms;
static uint32_t s_stop_guard_deadline_ms;
static uint32_t s_stop_timeout_ms;

static bool MotorCharacterizationTest_TimeReached(uint32_t now,
                                                   uint32_t deadline)
{
  return ((int32_t)(now - deadline) >= 0);
}

static int64_t MotorCharacterizationTest_AbsCount(int64_t value)
{
  return (value < 0) ? -value : value;
}

static bool MotorCharacterizationTest_DirectionMatches(int64_t delta)
{
  return ((s_direction > 0) && (delta > 0)) ||
         ((s_direction < 0) && (delta < 0));
}

static const char *MotorCharacterizationTest_GetDirectionName(void)
{
  return (s_direction > 0) ? "FWD" : "REV";
}

static const char *MotorCharacterizationTest_GetAbortReasonName(
  MotorCharacterizationAbortReason reason)
{
  switch (reason)
  {
    case MOTOR_CHAR_ABORT_OUTPUT_FAILURE:
      return "motor output failure";
    case MOTOR_CHAR_ABORT_ENCODER_FAILURE:
      return "encoder/speed snapshot failure";
    case MOTOR_CHAR_ABORT_WRONG_DIRECTION:
      return "encoder direction disagrees with command";
    case MOTOR_CHAR_ABORT_STOP_TIMEOUT:
      return "wheel did not stop between points";
    case MOTOR_CHAR_ABORT_POINT_TIMEOUT:
      return "per-point safety timeout";
    case MOTOR_CHAR_ABORT_OVERALL_TIMEOUT:
      return "overall safety timeout";
    case MOTOR_CHAR_ABORT_ESTOP:
      return "ESTOP latched";
    default:
      return "unspecified failure";
  }
}

static int32_t MotorCharacterizationTest_RoundFloat(float value)
{
  return (int32_t)((value >= 0.0f) ? (value + 0.5f) : (value - 0.5f));
}

static bool MotorCharacterizationTest_GetWheelTotal(uint32_t wheel_index,
                                                     int64_t *total)
{
  EncoderState encoder_states[ENCODER_WHEEL_COUNT];

  if ((wheel_index >= MOTOR_DRIVER_WHEEL_COUNT) || (total == NULL) ||
      (!EncoderService_GetSnapshot(encoder_states)))
  {
    return false;
  }

  *total = encoder_states[s_encoder_wheels[wheel_index]].accumulated_count;
  return true;
}

static void MotorCharacterizationTest_SetAborted(
  MotorCharacterizationAbortReason reason)
{
  taskENTER_CRITICAL();
  MotorDriver_Disable();
  s_abort_reason = reason;
  s_state = MOTOR_CHAR_STATE_ABORTED;
  taskEXIT_CRITICAL();
}

static void MotorCharacterizationTest_EnterWaitStop(uint32_t now_ms)
{
  s_stop_guard_deadline_ms = now_ms + MOTOR_CHAR_STOP_GUARD_MS;
  s_stop_timeout_ms = now_ms + MOTOR_CHAR_STOP_TIMEOUT_MS;
  s_state = MOTOR_CHAR_STATE_WAIT_STOP;
}

static void MotorCharacterizationTest_EnterPrepareStop(uint32_t now_ms)
{
  MotorDriver_Disable();
  s_stop_guard_deadline_ms = now_ms + MOTOR_CHAR_STOP_GUARD_MS;
  s_stop_timeout_ms = now_ms + MOTOR_CHAR_STOP_TIMEOUT_MS;
  s_state = MOTOR_CHAR_STATE_PREPARE_STOP;
}

static bool MotorCharacterizationTest_HasValidAverageSpeed(int64_t delta,
                                                            uint32_t elapsed_ms)
{
  const uint64_t absolute_counts =
    (uint64_t)MotorCharacterizationTest_AbsCount(delta);

  return (elapsed_ms > 0U) &&
         ((absolute_counts * 1000ULL) >=
          ((uint64_t)MOTOR_CHAR_MIN_VALID_CPS * elapsed_ms));
}

static void MotorCharacterizationTest_LogResult(const char *status,
                                                 int64_t delta,
                                                 uint32_t elapsed_ms)
{
  char delta_text[DEBUG_LOG_INT64_TEXT_SIZE];
  float average_cps = 0.0f;

  if (elapsed_ms > 0U)
  {
    average_cps = ((float)delta * 1000.0f) / (float)elapsed_ms;
  }

  DebugLog_FormatInt64(delta, delta_text);
  DEBUG_LOG_INFO(
    "CHAR result: wheel=%s dir=%s pwm=%d/1000 status=%s elapsed=%lu delta=%s cps=%ld rpm_x10=%ld mmps=%ld",
    s_wheel_names[s_wheel_index],
    MotorCharacterizationTest_GetDirectionName(),
    (int)s_duty_levels_permille[s_duty_index],
    status,
    (unsigned long)elapsed_ms,
    delta_text,
    (long)MotorCharacterizationTest_RoundFloat(average_cps),
    (long)MotorCharacterizationTest_RoundFloat(
      average_cps * 600.0f /
      (float)CHASSIS_ENCODER_COUNTS_PER_WHEEL_REV),
    (long)MotorCharacterizationTest_RoundFloat(
      average_cps * CHASSIS_WHEEL_CIRCUMFERENCE_M * 1000.0f /
      (float)CHASSIS_ENCODER_COUNTS_PER_WHEEL_REV));
}

static void MotorCharacterizationTest_StartPoint(uint32_t now_ms)
{
  int64_t start_total;
  const MotorDriverWheel wheel = s_motor_wheels[s_wheel_index];
  const int16_t duty = s_duty_levels_permille[s_duty_index];
  const int16_t command = (int16_t)(duty * s_direction);
  bool started = false;

  if (!MotorCharacterizationTest_GetWheelTotal(s_wheel_index, &start_total))
  {
    MotorCharacterizationTest_SetAborted(MOTOR_CHAR_ABORT_ENCODER_FAILURE);
    return;
  }

  taskENTER_CRITICAL();
  if ((s_state == MOTOR_CHAR_STATE_PREPARE_STOP) ||
      (s_state == MOTOR_CHAR_STATE_WAIT_STOP))
  {
    MotorDriver_Disable();

    /* Publish the active phase and hard deadline before applying power. */
    s_point_start_total = start_total;
    s_point_start_ms = now_ms;
    s_phase_deadline_ms = now_ms + MOTOR_CHAR_START_CHECK_MS;
    s_drive_deadline_ms = now_ms + MOTOR_CHAR_START_CHECK_MS +
      MOTOR_CHAR_SETTLE_MS + MOTOR_CHAR_MEASURE_MS +
      MOTOR_CHAR_DRIVE_MARGIN_MS;
    s_state = MOTOR_CHAR_STATE_START_CHECK;

    started = MotorDriver_Enable() &&
              MotorDriver_SetCommand(wheel, command);
    if (!started)
    {
      MotorDriver_Disable();
      s_abort_reason = EmergencyStop_IsLatched() ?
        MOTOR_CHAR_ABORT_ESTOP : MOTOR_CHAR_ABORT_OUTPUT_FAILURE;
      s_state = MOTOR_CHAR_STATE_ABORTED;
    }
  }
  taskEXIT_CRITICAL();

  if (started)
  {
    DEBUG_LOG_INFO("CHAR point start: wheel=%s dir=%s pwm=%d/1000",
                   s_wheel_names[s_wheel_index],
                   MotorCharacterizationTest_GetDirectionName(),
                   (int)duty);
  }
}

static void MotorCharacterizationTest_CheckStart(uint32_t now_ms)
{
  int64_t current_total;
  int64_t delta;
  bool no_start = false;
  bool wrong_direction = false;

  if (!MotorCharacterizationTest_GetWheelTotal(s_wheel_index, &current_total))
  {
    MotorCharacterizationTest_SetAborted(MOTOR_CHAR_ABORT_ENCODER_FAILURE);
    return;
  }

  taskENTER_CRITICAL();
  if ((s_state == MOTOR_CHAR_STATE_START_CHECK) &&
      MotorCharacterizationTest_TimeReached(now_ms, s_phase_deadline_ms))
  {
    delta = current_total - s_point_start_total;
    if (MotorCharacterizationTest_AbsCount(delta) <
        MOTOR_CHAR_MOVED_THRESHOLD_COUNTS)
    {
      MotorDriver_Disable();
      MotorCharacterizationTest_EnterWaitStop(now_ms);
      no_start = true;
    }
    else if (!MotorCharacterizationTest_DirectionMatches(delta))
    {
      MotorDriver_Disable();
      s_abort_reason = MOTOR_CHAR_ABORT_WRONG_DIRECTION;
      s_state = MOTOR_CHAR_STATE_ABORTED;
      wrong_direction = true;
    }
    else
    {
      s_phase_deadline_ms = now_ms + MOTOR_CHAR_SETTLE_MS;
      s_state = MOTOR_CHAR_STATE_SETTLING;
    }
  }
  taskEXIT_CRITICAL();

  if (no_start)
  {
    MotorCharacterizationTest_LogResult("NO_START",
                                        current_total - s_point_start_total,
                                        now_ms - s_point_start_ms);
  }
  (void)wrong_direction;
}

static void MotorCharacterizationTest_BeginMeasurement(uint32_t now_ms)
{
  int64_t current_total;

  if (!MotorCharacterizationTest_GetWheelTotal(s_wheel_index, &current_total))
  {
    MotorCharacterizationTest_SetAborted(MOTOR_CHAR_ABORT_ENCODER_FAILURE);
    return;
  }

  taskENTER_CRITICAL();
  if ((s_state == MOTOR_CHAR_STATE_SETTLING) &&
      MotorCharacterizationTest_TimeReached(now_ms, s_phase_deadline_ms))
  {
    s_measurement_start_total = current_total;
    s_measurement_start_ms = now_ms;
    s_phase_deadline_ms = now_ms + MOTOR_CHAR_MEASURE_MS;
    s_state = MOTOR_CHAR_STATE_MEASURING;
  }
  taskEXIT_CRITICAL();
}

static void MotorCharacterizationTest_FinishMeasurement(uint32_t now_ms)
{
  int64_t current_total;
  int64_t delta = 0;
  uint32_t elapsed_ms = 0U;
  bool report = false;
  bool wrong_direction = false;
  bool valid_speed = false;

  if (!MotorCharacterizationTest_GetWheelTotal(s_wheel_index, &current_total))
  {
    MotorCharacterizationTest_SetAborted(MOTOR_CHAR_ABORT_ENCODER_FAILURE);
    return;
  }

  taskENTER_CRITICAL();
  if ((s_state == MOTOR_CHAR_STATE_MEASURING) &&
      MotorCharacterizationTest_TimeReached(now_ms, s_phase_deadline_ms))
  {
    MotorDriver_Disable();
    delta = current_total - s_measurement_start_total;
    elapsed_ms = now_ms - s_measurement_start_ms;
    if ((MotorCharacterizationTest_AbsCount(delta) >=
         MOTOR_CHAR_MOVED_THRESHOLD_COUNTS) &&
        (!MotorCharacterizationTest_DirectionMatches(delta)))
    {
      s_abort_reason = MOTOR_CHAR_ABORT_WRONG_DIRECTION;
      s_state = MOTOR_CHAR_STATE_ABORTED;
      wrong_direction = true;
    }
    else
    {
      valid_speed = MotorCharacterizationTest_HasValidAverageSpeed(delta,
                                                                   elapsed_ms);
      MotorCharacterizationTest_EnterWaitStop(now_ms);
      report = true;
    }
  }
  taskEXIT_CRITICAL();

  if (report)
  {
    MotorCharacterizationTest_LogResult(valid_speed ? "OK" : "LOW_SPEED",
                                        delta,
                                        elapsed_ms);
  }
  (void)wrong_direction;
}

static void MotorCharacterizationTest_AdvancePoint(uint32_t now_ms)
{
  bool completed = false;
  bool start_next = false;

  taskENTER_CRITICAL();
  if (s_state == MOTOR_CHAR_STATE_WAIT_STOP)
  {
    ++s_duty_index;
    if (s_duty_index >=
        (sizeof(s_duty_levels_permille) / sizeof(s_duty_levels_permille[0])))
    {
      s_duty_index = 0U;
      ++s_wheel_index;
    }

    if (s_wheel_index >= MOTOR_DRIVER_WHEEL_COUNT)
    {
      MotorDriver_Disable();
      s_state = MOTOR_CHAR_STATE_COMPLETED;
      completed = true;
    }
    else
    {
      start_next = true;
    }
  }
  taskEXIT_CRITICAL();

  if (start_next)
  {
    MotorCharacterizationTest_StartPoint(now_ms);
  }
  (void)completed;
}

static void MotorCharacterizationTest_WaitForStop(uint32_t now_ms)
{
  WheelSpeedState speed_states[ENCODER_WHEEL_COUNT];
  const EncoderWheel wheel = s_encoder_wheels[s_wheel_index];

  if (!WheelSpeedEstimator_GetSnapshot(speed_states))
  {
    MotorCharacterizationTest_SetAborted(MOTOR_CHAR_ABORT_ENCODER_FAILURE);
    return;
  }

  if (MotorCharacterizationTest_TimeReached(now_ms, s_stop_timeout_ms))
  {
    MotorCharacterizationTest_SetAborted(MOTOR_CHAR_ABORT_STOP_TIMEOUT);
  }
  else if (MotorCharacterizationTest_TimeReached(now_ms,
                                                  s_stop_guard_deadline_ms) &&
           speed_states[wheel].valid && speed_states[wheel].stopped)
  {
    MotorCharacterizationTest_AdvancePoint(now_ms);
  }
}

static void MotorCharacterizationTest_WaitForInitialStop(uint32_t now_ms)
{
  WheelSpeedState speed_states[ENCODER_WHEEL_COUNT];
  const EncoderWheel wheel = s_encoder_wheels[s_wheel_index];

  if (!WheelSpeedEstimator_GetSnapshot(speed_states))
  {
    MotorCharacterizationTest_SetAborted(MOTOR_CHAR_ABORT_ENCODER_FAILURE);
    return;
  }

  if (MotorCharacterizationTest_TimeReached(now_ms, s_stop_timeout_ms))
  {
    MotorCharacterizationTest_SetAborted(MOTOR_CHAR_ABORT_STOP_TIMEOUT);
  }
  else if (MotorCharacterizationTest_TimeReached(now_ms,
                                                  s_stop_guard_deadline_ms) &&
           speed_states[wheel].valid && speed_states[wheel].stopped)
  {
    MotorCharacterizationTest_StartPoint(now_ms);
  }
}

void MotorCharacterizationTest_Init(void)
{
  taskENTER_CRITICAL();
  MotorDriver_Disable();
  s_state = MOTOR_CHAR_STATE_DISARMED;
  s_abort_reason = MOTOR_CHAR_ABORT_NONE;
  s_phase_deadline_ms = 0U;
  s_drive_deadline_ms = 0U;
  s_overall_deadline_ms = 0U;
  s_arm_expired_notice = false;
  s_wheel_index = 0U;
  s_duty_index = 0U;
  s_direction = 1;
  s_point_start_total = 0;
  s_point_start_ms = 0U;
  s_measurement_start_total = 0;
  s_measurement_start_ms = 0U;
  s_stop_guard_deadline_ms = 0U;
  s_stop_timeout_ms = 0U;
  taskEXIT_CRITICAL();
}

bool MotorCharacterizationTest_Arm(uint32_t now_ms)
{
  bool armed = false;

  if (!EmergencyStop_CanEnableMotor())
  {
    DEBUG_LOG_ERROR("Characterization rejected: ESTOP input/latch active");
    return false;
  }

  taskENTER_CRITICAL();
  if (s_state == MOTOR_CHAR_STATE_DISARMED)
  {
    MotorDriver_Disable();
    s_phase_deadline_ms = now_ms + MOTOR_CHAR_ARM_WINDOW_MS;
    s_arm_expired_notice = false;
    s_abort_reason = MOTOR_CHAR_ABORT_NONE;
    s_state = MOTOR_CHAR_STATE_ARMED;
    armed = true;
  }
  taskEXIT_CRITICAL();

  if (armed)
  {
    DEBUG_LOG_WARN("Characterization armed for 5 s; send 'f' or 'r'");
    DEBUG_LOG_WARN("Scan: one wheel at a time, PWM 5/7.5/10/12.5/15/20/25/30%%");
    DEBUG_LOG_WARN("Motor supply assumption: VM=12 V (not instrument-verified)");
  }
  return armed;
}

bool MotorCharacterizationTest_Start(int8_t direction, uint32_t now_ms)
{
  bool queued = false;

  if (((direction != 1) && (direction != -1)) ||
      (!EmergencyStop_CanEnableMotor()))
  {
    return false;
  }

  taskENTER_CRITICAL();
  if ((s_state == MOTOR_CHAR_STATE_ARMED) &&
      (!MotorCharacterizationTest_TimeReached(now_ms, s_phase_deadline_ms)))
  {
    MotorDriver_Disable();
    s_direction = direction;
    s_wheel_index = 0U;
    s_duty_index = 0U;
    s_phase_deadline_ms = now_ms + MOTOR_CHAR_COUNTDOWN_MS;
    s_overall_deadline_ms = now_ms + MOTOR_CHAR_OVERALL_TIMEOUT_MS;
    s_state = MOTOR_CHAR_STATE_COUNTDOWN;
    queued = true;
  }
  taskEXIT_CRITICAL();

  if (queued)
  {
    DEBUG_LOG_WARN("Characterization %s starts in 3 s; send 's' to cancel",
                   MotorCharacterizationTest_GetDirectionName());
    DEBUG_LOG_WARN("Keep all four wheels lifted; maximum run time is 120 s");
    DEBUG_LOG_INFO("CHAR scan begin: dir=%s VM=UNMEASURED_ASSUMED_12V counts_per_rev=%lu wheel_diameter_mm=%lu",
                   MotorCharacterizationTest_GetDirectionName(),
                   (unsigned long)CHASSIS_ENCODER_COUNTS_PER_WHEEL_REV,
                   (unsigned long)CHASSIS_WHEEL_DIAMETER_MM);
    DEBUG_LOG_INFO("CHAR timing: start_check_ms=%lu settle_ms=%lu sample_ms=%lu stop_guard_ms=%lu stop_timeout_ms=%lu min_valid_cps=%lu",
                   (unsigned long)MOTOR_CHAR_START_CHECK_MS,
                   (unsigned long)MOTOR_CHAR_SETTLE_MS,
                   (unsigned long)MOTOR_CHAR_MEASURE_MS,
                   (unsigned long)MOTOR_CHAR_STOP_GUARD_MS,
                   (unsigned long)MOTOR_CHAR_STOP_TIMEOUT_MS,
                   (unsigned long)MOTOR_CHAR_MIN_VALID_CPS);
  }
  return queued;
}

bool MotorCharacterizationTest_IsActive(void)
{
  return (s_state != MOTOR_CHAR_STATE_DISARMED);
}

bool MotorCharacterizationTest_IsArmed(void)
{
  return (s_state == MOTOR_CHAR_STATE_ARMED);
}

void MotorCharacterizationTest_Process(uint32_t now_ms)
{
  MotorCharacterizationState state;
  MotorCharacterizationAbortReason abort_reason = MOTOR_CHAR_ABORT_NONE;
  bool completed_notice = false;
  bool aborted_notice = false;
  bool arm_expired_notice = false;

  taskENTER_CRITICAL();
  if (s_state == MOTOR_CHAR_STATE_COMPLETED)
  {
    s_state = MOTOR_CHAR_STATE_DISARMED;
    completed_notice = true;
  }
  else if (s_state == MOTOR_CHAR_STATE_ABORTED)
  {
    abort_reason = s_abort_reason;
    s_state = MOTOR_CHAR_STATE_DISARMED;
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
    DEBUG_LOG_INFO("Characterization %s completed: 32 points; motors disabled",
                   MotorCharacterizationTest_GetDirectionName());
  }
  if (aborted_notice)
  {
    DEBUG_LOG_ERROR("Characterization aborted: reason=%s context=%s/%s/%d; motors disabled",
                    MotorCharacterizationTest_GetAbortReasonName(abort_reason),
                    s_wheel_names[s_wheel_index],
                    MotorCharacterizationTest_GetDirectionName(),
                    (int)s_duty_levels_permille[s_duty_index]);
  }
  if (arm_expired_notice)
  {
    DEBUG_LOG_INFO("Characterization arm window expired; motors remain disabled");
  }

  switch (state)
  {
    case MOTOR_CHAR_STATE_COUNTDOWN:
      if (MotorCharacterizationTest_TimeReached(now_ms, s_phase_deadline_ms))
      {
        taskENTER_CRITICAL();
        if (s_state == MOTOR_CHAR_STATE_COUNTDOWN)
        {
          MotorCharacterizationTest_EnterPrepareStop(now_ms);
        }
        taskEXIT_CRITICAL();
      }
      break;

    case MOTOR_CHAR_STATE_PREPARE_STOP:
      MotorCharacterizationTest_WaitForInitialStop(now_ms);
      break;

    case MOTOR_CHAR_STATE_START_CHECK:
      MotorCharacterizationTest_CheckStart(now_ms);
      break;

    case MOTOR_CHAR_STATE_SETTLING:
      MotorCharacterizationTest_BeginMeasurement(now_ms);
      break;

    case MOTOR_CHAR_STATE_MEASURING:
      MotorCharacterizationTest_FinishMeasurement(now_ms);
      break;

    case MOTOR_CHAR_STATE_WAIT_STOP:
      MotorCharacterizationTest_WaitForStop(now_ms);
      break;

    default:
      break;
  }
}

void MotorCharacterizationTest_Stop(void)
{
  bool was_active;

  taskENTER_CRITICAL();
  was_active = (s_state != MOTOR_CHAR_STATE_DISARMED);
  MotorDriver_Disable();
  s_state = MOTOR_CHAR_STATE_DISARMED;
  s_abort_reason = MOTOR_CHAR_ABORT_NONE;
  s_arm_expired_notice = false;
  taskEXIT_CRITICAL();

  if (was_active)
  {
    DEBUG_LOG_WARN("Characterization cancelled; all motor outputs disabled");
  }
}

void MotorCharacterizationTest_SafetyProcess(uint32_t now_ms)
{
  taskENTER_CRITICAL();
  if (EmergencyStop_IsLatched() &&
      (s_state != MOTOR_CHAR_STATE_DISARMED))
  {
    MotorDriver_Disable();
    s_abort_reason = MOTOR_CHAR_ABORT_ESTOP;
    s_state = MOTOR_CHAR_STATE_ABORTED;
  }
  else if ((s_state == MOTOR_CHAR_STATE_ARMED) &&
           MotorCharacterizationTest_TimeReached(now_ms,
                                                  s_phase_deadline_ms))
  {
    MotorDriver_Disable();
    s_state = MOTOR_CHAR_STATE_DISARMED;
    s_arm_expired_notice = true;
  }
  else if (((s_state == MOTOR_CHAR_STATE_COUNTDOWN) ||
            (s_state == MOTOR_CHAR_STATE_PREPARE_STOP) ||
            (s_state == MOTOR_CHAR_STATE_START_CHECK) ||
            (s_state == MOTOR_CHAR_STATE_SETTLING) ||
            (s_state == MOTOR_CHAR_STATE_MEASURING) ||
            (s_state == MOTOR_CHAR_STATE_WAIT_STOP)) &&
           MotorCharacterizationTest_TimeReached(now_ms,
                                                  s_overall_deadline_ms))
  {
    MotorDriver_Disable();
    s_abort_reason = MOTOR_CHAR_ABORT_OVERALL_TIMEOUT;
    s_state = MOTOR_CHAR_STATE_ABORTED;
  }
  else if (((s_state == MOTOR_CHAR_STATE_START_CHECK) ||
            (s_state == MOTOR_CHAR_STATE_SETTLING) ||
            (s_state == MOTOR_CHAR_STATE_MEASURING)) &&
           MotorCharacterizationTest_TimeReached(now_ms, s_drive_deadline_ms))
  {
    MotorDriver_Disable();
    s_abort_reason = MOTOR_CHAR_ABORT_POINT_TIMEOUT;
    s_state = MOTOR_CHAR_STATE_ABORTED;
  }
  taskEXIT_CRITICAL();
}

#else

void MotorCharacterizationTest_Init(void)
{
}

bool MotorCharacterizationTest_Arm(uint32_t now_ms)
{
  (void)now_ms;
  return false;
}

bool MotorCharacterizationTest_Start(int8_t direction, uint32_t now_ms)
{
  (void)direction;
  (void)now_ms;
  return false;
}

bool MotorCharacterizationTest_IsActive(void)
{
  return false;
}

bool MotorCharacterizationTest_IsArmed(void)
{
  return false;
}

void MotorCharacterizationTest_Process(uint32_t now_ms)
{
  (void)now_ms;
}

void MotorCharacterizationTest_Stop(void)
{
}

void MotorCharacterizationTest_SafetyProcess(uint32_t now_ms)
{
  (void)now_ms;
}

#endif /* MOTOR_OPEN_LOOP_TEST_ENABLED */
