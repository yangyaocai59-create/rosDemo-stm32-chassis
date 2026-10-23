/**
 * @file motor_open_loop_test.c
 * @brief Debug-only, explicitly armed four-wheel motor open-loop test.
 */

#include "motor_open_loop_test.h"
#include "motion_trace.h"

#if MOTOR_OPEN_LOOP_TEST_ENABLED

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "FreeRTOS.h"
#include "chassis_controller.h"
#include "chassis_ground_test.h"
#include "chassis_motion_test.h"
#include "chassis_runtime.h"
#include "chassis_runtime_test.h"
#include "debug_log.h"
#include "emergency_stop.h"
#include "encoder_service.h"
#include "fault_injection_test.h"
#include "four_wheel_speed_control_test.h"
#include "imu_static_test.h"
#include "motor_characterization_test.h"
#include "motor_driver.h"
#include "motor_speed_control_test.h"
#include "stm32f4xx_hal.h"
#include "task.h"
#include "usart.h"

#define MOTOR_TEST_COMMAND_PERMILLE  150
#define MOTOR_TEST_DURATION_MS       800U
#define MOTOR_TEST_ARM_WINDOW_MS     5000U
#define MOTOR_TEST_VALIDATION_DELAY_MS 3000U

typedef enum
{
  MOTOR_TEST_STATE_DISARMED = 0,
  MOTOR_TEST_STATE_ARMED,
  MOTOR_TEST_STATE_RUNNING,
  MOTOR_TEST_STATE_COMPLETED,
  MOTOR_TEST_STATE_VALIDATION_COUNTDOWN,
  MOTOR_TEST_STATE_VALIDATION_RUNNING,
  MOTOR_TEST_STATE_VALIDATION_COMPLETED
} MotorTestState;

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

static volatile MotorTestState s_state;
static volatile MotorDriverWheel s_selected_wheel;
static volatile uint32_t s_deadline_ms;
static volatile bool s_arm_expired_notice;
static int64_t s_start_total;
static int64_t s_validation_start_totals[MOTOR_DRIVER_WHEEL_COUNT];
static int16_t s_active_command;

static bool MotorOpenLoopTest_IsWheelValid(MotorDriverWheel wheel)
{
  return ((uint32_t)wheel < (uint32_t)MOTOR_DRIVER_WHEEL_COUNT);
}

static const char *MotorOpenLoopTest_GetWheelName(MotorDriverWheel wheel)
{
  if (!MotorOpenLoopTest_IsWheelValid(wheel))
  {
    return "NONE";
  }
  return s_wheel_names[wheel];
}

static bool MotorOpenLoopTest_TimeReached(uint32_t now, uint32_t deadline)
{
  return ((int32_t)(now - deadline) >= 0);
}

static bool MotorOpenLoopTest_IsAnySpeedControlActive(void)
{
  return MotorSpeedControlTest_IsActive() ||
         FourWheelSpeedControlTest_IsActive() ||
         ChassisMotionTest_IsActive() ||
         ChassisGroundTest_IsActive();
}

static bool MotorOpenLoopTest_IsValidationState(MotorTestState state)
{
  return ((state == MOTOR_TEST_STATE_VALIDATION_COUNTDOWN) ||
          (state == MOTOR_TEST_STATE_VALIDATION_RUNNING) ||
          (state == MOTOR_TEST_STATE_VALIDATION_COMPLETED));
}

static bool MotorOpenLoopTest_GetAllTotals(
  int64_t totals[MOTOR_DRIVER_WHEEL_COUNT])
{
  EncoderState states[ENCODER_WHEEL_COUNT];

  if ((totals == NULL) || (!EncoderService_GetSnapshot(states)))
  {
    return false;
  }

  for (uint32_t wheel = 0U; wheel < MOTOR_DRIVER_WHEEL_COUNT; ++wheel)
  {
    totals[wheel] = states[s_encoder_wheels[wheel]].accumulated_count;
  }
  return true;
}

static bool MotorOpenLoopTest_GetWheelTotal(MotorDriverWheel wheel,
                                            int64_t *total)
{
  EncoderState states[ENCODER_WHEEL_COUNT];

  if ((!MotorOpenLoopTest_IsWheelValid(wheel)) ||
      (total == NULL) ||
      (!EncoderService_GetSnapshot(states)))
  {
    return false;
  }

  *total = states[s_encoder_wheels[wheel]].accumulated_count;
  return true;
}

static void MotorOpenLoopTest_ReportResult(const char *reason)
{
  const MotorDriverWheel wheel = s_selected_wheel;
  char delta_text[DEBUG_LOG_INT64_TEXT_SIZE];
  int64_t end_total;

  if (MotorOpenLoopTest_GetWheelTotal(wheel, &end_total))
  {
    DebugLog_FormatInt64(end_total - s_start_total, delta_text);
    DEBUG_LOG_INFO("%s test %s: command=%d/1000 encoder_delta=%s",
                   MotorOpenLoopTest_GetWheelName(wheel),
                   reason,
                   (int)s_active_command,
                   delta_text);
  }
  else
  {
    DEBUG_LOG_WARN("%s test %s: encoder snapshot unavailable",
                   MotorOpenLoopTest_GetWheelName(wheel),
                   reason);
  }
}

static void MotorOpenLoopTest_ReportValidationResult(const char *reason)
{
  int64_t end_totals[MOTOR_DRIVER_WHEEL_COUNT];
  char delta_text[MOTOR_DRIVER_WHEEL_COUNT][DEBUG_LOG_INT64_TEXT_SIZE];

  if (!MotorOpenLoopTest_GetAllTotals(end_totals))
  {
    DEBUG_LOG_WARN("4W validation %s: encoder snapshot unavailable", reason);
    return;
  }

  for (uint32_t wheel = 0U; wheel < MOTOR_DRIVER_WHEEL_COUNT; ++wheel)
  {
    DebugLog_FormatInt64(end_totals[wheel] -
                         s_validation_start_totals[wheel],
                         delta_text[wheel]);
  }

  DEBUG_LOG_INFO("4W validation %s: LF=%s LR=%s RF=%s RR=%s",
                 reason,
                 delta_text[MOTOR_DRIVER_WHEEL_LF],
                 delta_text[MOTOR_DRIVER_WHEEL_LR],
                 delta_text[MOTOR_DRIVER_WHEEL_RF],
                 delta_text[MOTOR_DRIVER_WHEEL_RR]);
}

static void MotorOpenLoopTest_Select(MotorDriverWheel wheel)
{
  if (!ChassisRuntime_CanRunDiagnostics())
  {
    DEBUG_LOG_WARN("Wheel selection rejected: chassis runtime is not IDLE");
    return;
  }
  if (MotorCharacterizationTest_IsActive())
  {
    DEBUG_LOG_WARN("Wheel selection rejected: characterization active; send 's'");
    return;
  }
  if (MotorOpenLoopTest_IsAnySpeedControlActive())
  {
    DEBUG_LOG_WARN("Wheel selection rejected: PI test active; send 's'");
    return;
  }

  taskENTER_CRITICAL();
  if ((s_state == MOTOR_TEST_STATE_RUNNING) ||
      MotorOpenLoopTest_IsValidationState(s_state))
  {
    taskEXIT_CRITICAL();
    DEBUG_LOG_WARN("Wheel selection rejected: test running; send 's' first");
    return;
  }

  MotorDriver_Disable();
  s_selected_wheel = wheel;
  s_state = MOTOR_TEST_STATE_DISARMED;
  s_deadline_ms = 0U;
  s_arm_expired_notice = false;
  taskEXIT_CRITICAL();

  DEBUG_LOG_INFO("Selected %s; motor disabled. Send 'a' for pulse or 'p' for PI",
                 MotorOpenLoopTest_GetWheelName(wheel));
}

static void MotorOpenLoopTest_Arm(uint32_t now)
{
  MotorDriverWheel wheel;

  if (!ChassisRuntime_CanRunDiagnostics())
  {
    DEBUG_LOG_WARN("Single-wheel arm rejected: chassis runtime is not IDLE");
    return;
  }

  if (MotorCharacterizationTest_IsActive())
  {
    DEBUG_LOG_WARN("Single-wheel arm rejected: characterization active; send 's'");
    return;
  }
  if (MotorOpenLoopTest_IsAnySpeedControlActive())
  {
    DEBUG_LOG_WARN("Single-wheel arm rejected: PI test active; send 's'");
    return;
  }

  taskENTER_CRITICAL();
  wheel = s_selected_wheel;
  if ((s_state == MOTOR_TEST_STATE_RUNNING) ||
      MotorOpenLoopTest_IsValidationState(s_state))
  {
    taskEXIT_CRITICAL();
    DEBUG_LOG_WARN("%s test is already running; send 's' to stop",
                   MotorOpenLoopTest_GetWheelName(wheel));
    return;
  }
  if (!MotorOpenLoopTest_IsWheelValid(wheel))
  {
    taskEXIT_CRITICAL();
    DEBUG_LOG_WARN("No wheel selected; send 1=LF, 2=LR, 3=RF, or 4=RR");
    return;
  }

  MotorDriver_Disable();
  s_deadline_ms = now + MOTOR_TEST_ARM_WINDOW_MS;
  s_state = MOTOR_TEST_STATE_ARMED;
  s_arm_expired_notice = false;
  taskEXIT_CRITICAL();

  DEBUG_LOG_WARN("%s test armed for 5 s; send 'f' or 'r'",
                 MotorOpenLoopTest_GetWheelName(wheel));
}

static void MotorOpenLoopTest_Start(int16_t command, uint32_t now)
{
  const MotorDriverWheel wheel = s_selected_wheel;
  int64_t start_total;
  bool started = false;

  if (!MotorOpenLoopTest_GetWheelTotal(wheel, &start_total))
  {
    DEBUG_LOG_ERROR("Motor test rejected: select a valid wheel first");
    return;
  }

  taskENTER_CRITICAL();
  if ((s_selected_wheel == wheel) &&
      (s_state == MOTOR_TEST_STATE_ARMED) &&
      (!MotorOpenLoopTest_TimeReached(now, s_deadline_ms)))
  {
    s_start_total = start_total;
    s_active_command = command;
    s_deadline_ms = now + MOTOR_TEST_DURATION_MS;
    s_state = MOTOR_TEST_STATE_RUNNING;

    started = MotorDriver_Enable() &&
              MotorDriver_SetCommand(wheel, command);
    if (!started)
    {
      MotorDriver_Disable();
      s_state = MOTOR_TEST_STATE_DISARMED;
    }
  }
  taskEXIT_CRITICAL();

  if (started)
  {
    DEBUG_LOG_WARN("%s test started: physical=%s duty=15%% duration=800 ms",
                   MotorOpenLoopTest_GetWheelName(wheel),
                   (command > 0) ? "FORWARD" : "REVERSE");
  }
  else
  {
    DEBUG_LOG_WARN("%s test rejected: send 'a', then 'f' or 'r' within 5 s",
                   MotorOpenLoopTest_GetWheelName(wheel));
  }
}

static void MotorOpenLoopTest_QueueValidation(uint32_t now)
{
  if (!ChassisRuntime_CanRunDiagnostics())
  {
    DEBUG_LOG_WARN("4W validation rejected: chassis runtime is not IDLE");
    return;
  }
  if (MotorCharacterizationTest_IsActive())
  {
    DEBUG_LOG_WARN("4W validation rejected: characterization active; send 's'");
    return;
  }
  if (MotorOpenLoopTest_IsAnySpeedControlActive())
  {
    DEBUG_LOG_WARN("4W validation rejected: PI test active; send 's'");
    return;
  }

  taskENTER_CRITICAL();
  if ((s_state == MOTOR_TEST_STATE_RUNNING) ||
      MotorOpenLoopTest_IsValidationState(s_state))
  {
    taskEXIT_CRITICAL();
    DEBUG_LOG_WARN("4W validation rejected: another motor test is active");
    return;
  }

  MotorDriver_Disable();
  s_selected_wheel = MOTOR_DRIVER_WHEEL_COUNT;
  s_state = MOTOR_TEST_STATE_VALIDATION_COUNTDOWN;
  s_deadline_ms = now + MOTOR_TEST_VALIDATION_DELAY_MS;
  s_arm_expired_notice = false;
  taskEXIT_CRITICAL();

  DEBUG_LOG_WARN("4W FORWARD validation starts in 3 s; send 's' to cancel");
}

static void MotorOpenLoopTest_ArmCharacterization(uint32_t now)
{
  bool available = false;

  if (!ChassisRuntime_CanRunDiagnostics())
  {
    DEBUG_LOG_WARN("Characterization rejected: chassis runtime is not IDLE");
    return;
  }

  if (MotorCharacterizationTest_IsActive())
  {
    DEBUG_LOG_WARN("Characterization arm rejected: scan already active; send 's'");
    return;
  }
  if (MotorOpenLoopTest_IsAnySpeedControlActive())
  {
    DEBUG_LOG_WARN("Characterization rejected: PI test active; send 's'");
    return;
  }

  taskENTER_CRITICAL();
  if ((s_state != MOTOR_TEST_STATE_RUNNING) &&
      (!MotorOpenLoopTest_IsValidationState(s_state)))
  {
    MotorDriver_Disable();
    s_state = MOTOR_TEST_STATE_DISARMED;
    s_arm_expired_notice = false;
    available = true;
  }
  taskEXIT_CRITICAL();

  if (!available)
  {
    DEBUG_LOG_WARN("Characterization rejected: another motor test is active");
    return;
  }

  (void)MotorCharacterizationTest_Arm(now);
}

static void MotorOpenLoopTest_ArmSpeedControl(uint32_t now)
{
  MotorDriverWheel wheel;
  bool available = false;

  if (!ChassisRuntime_CanRunDiagnostics())
  {
    DEBUG_LOG_WARN("PI arm rejected: chassis runtime is not IDLE");
    return;
  }

  if (MotorCharacterizationTest_IsActive())
  {
    DEBUG_LOG_WARN("PI arm rejected: characterization active; send 's'");
    return;
  }
  if (MotorOpenLoopTest_IsAnySpeedControlActive())
  {
    DEBUG_LOG_WARN("PI arm rejected: PI test already active; send 's'");
    return;
  }

  taskENTER_CRITICAL();
  wheel = s_selected_wheel;
  if ((s_state != MOTOR_TEST_STATE_RUNNING) &&
      (!MotorOpenLoopTest_IsValidationState(s_state)) &&
      MotorOpenLoopTest_IsWheelValid(wheel))
  {
    MotorDriver_Disable();
    s_state = MOTOR_TEST_STATE_DISARMED;
    s_arm_expired_notice = false;
    available = true;
  }
  taskEXIT_CRITICAL();

  if (!available)
  {
    DEBUG_LOG_WARN("PI arm rejected: select wheel 1/2/3/4 or stop active test");
    return;
  }

  (void)MotorSpeedControlTest_Arm(wheel, now);
}

static void MotorOpenLoopTest_ArmFourWheelSpeedControl(uint32_t now)
{
  bool available = false;

  if (!ChassisRuntime_CanRunDiagnostics())
  {
    DEBUG_LOG_WARN("4PI arm rejected: chassis runtime is not IDLE");
    return;
  }

  if (MotorCharacterizationTest_IsActive())
  {
    DEBUG_LOG_WARN("4PI arm rejected: characterization active; send 's'");
    return;
  }
  if (MotorOpenLoopTest_IsAnySpeedControlActive())
  {
    DEBUG_LOG_WARN("4PI arm rejected: PI test already active; send 's'");
    return;
  }

  taskENTER_CRITICAL();
  if ((s_state != MOTOR_TEST_STATE_RUNNING) &&
      (!MotorOpenLoopTest_IsValidationState(s_state)))
  {
    MotorDriver_Disable();
    s_selected_wheel = MOTOR_DRIVER_WHEEL_COUNT;
    s_state = MOTOR_TEST_STATE_DISARMED;
    s_arm_expired_notice = false;
    available = true;
  }
  taskEXIT_CRITICAL();

  if (!available)
  {
    DEBUG_LOG_WARN("4PI arm rejected: another motor test is active");
    return;
  }

  (void)FourWheelSpeedControlTest_Arm(now);
}

static void MotorOpenLoopTest_ArmChassisMotion(uint32_t now)
{
  bool available = false;

  if (!ChassisRuntime_CanRunDiagnostics())
  {
    DEBUG_LOG_WARN("KIN arm rejected: chassis runtime is not IDLE");
    return;
  }

  if (MotorCharacterizationTest_IsActive())
  {
    DEBUG_LOG_WARN("KIN arm rejected: characterization active; send 's'");
    return;
  }
  if (MotorOpenLoopTest_IsAnySpeedControlActive())
  {
    DEBUG_LOG_WARN("KIN arm rejected: closed-loop test active; send 's'");
    return;
  }

  taskENTER_CRITICAL();
  if ((s_state != MOTOR_TEST_STATE_RUNNING) &&
      (!MotorOpenLoopTest_IsValidationState(s_state)))
  {
    MotorDriver_Disable();
    s_selected_wheel = MOTOR_DRIVER_WHEEL_COUNT;
    s_state = MOTOR_TEST_STATE_DISARMED;
    s_arm_expired_notice = false;
    available = true;
  }
  taskEXIT_CRITICAL();

  if (!available)
  {
    DEBUG_LOG_WARN("KIN arm rejected: another motor test is active");
    return;
  }

  (void)ChassisMotionTest_Arm(now);
}

static void MotorOpenLoopTest_ArmGroundMotion(uint32_t now)
{
  bool available = false;

  if (!ChassisRuntime_CanRunDiagnostics())
  {
    DEBUG_LOG_WARN("GROUND arm rejected: chassis runtime is not IDLE");
    return;
  }

  if (MotorCharacterizationTest_IsActive())
  {
    DEBUG_LOG_WARN("GROUND arm rejected: characterization active; send 's'");
    return;
  }
  if (MotorOpenLoopTest_IsAnySpeedControlActive())
  {
    DEBUG_LOG_WARN("GROUND arm rejected: another closed-loop test active; send 's'");
    return;
  }

  taskENTER_CRITICAL();
  if ((s_state != MOTOR_TEST_STATE_RUNNING) &&
      (!MotorOpenLoopTest_IsValidationState(s_state)))
  {
    MotorDriver_Disable();
    s_selected_wheel = MOTOR_DRIVER_WHEEL_COUNT;
    s_state = MOTOR_TEST_STATE_DISARMED;
    s_arm_expired_notice = false;
    available = true;
  }
  taskEXIT_CRITICAL();

  if (!available)
  {
    DEBUG_LOG_WARN("GROUND arm rejected: another motor test is active");
    return;
  }

  (void)ChassisGroundTest_Arm(now);
}

static void MotorOpenLoopTest_StartGroundMotion(
  ChassisGroundTestAction action,
  uint32_t now)
{
  if (!ChassisGroundTest_IsArmed())
  {
    DEBUG_LOG_WARN("GROUND action rejected: send 'b' first");
    return;
  }
  (void)ChassisGroundTest_Start(action, now);
}

static void MotorOpenLoopTest_HandleDirectionCommand(int16_t command,
                                                     uint32_t now)
{
  if (MotorCharacterizationTest_IsArmed())
  {
    if (!MotorCharacterizationTest_Start((command > 0) ? 1 : -1, now))
    {
      DEBUG_LOG_WARN("Characterization start rejected; send 'c' to re-arm");
    }
  }
  else if (MotorCharacterizationTest_IsActive())
  {
    DEBUG_LOG_WARN("Direction command rejected: characterization active; send 's'");
  }
  else if (MotorOpenLoopTest_IsAnySpeedControlActive())
  {
    DEBUG_LOG_WARN("Direction command rejected: PI test active; send 's'");
  }
  else
  {
    MotorOpenLoopTest_Start(command, now);
  }
}

static void MotorOpenLoopTest_StartValidation(uint32_t now)
{
  int64_t start_totals[MOTOR_DRIVER_WHEEL_COUNT];
  bool started = false;

  if (!MotorOpenLoopTest_GetAllTotals(start_totals))
  {
    taskENTER_CRITICAL();
    MotorDriver_Disable();
    s_state = MOTOR_TEST_STATE_DISARMED;
    taskEXIT_CRITICAL();
    DEBUG_LOG_ERROR("4W validation rejected: encoder snapshot unavailable");
    return;
  }

  taskENTER_CRITICAL();
  if ((s_state == MOTOR_TEST_STATE_VALIDATION_COUNTDOWN) &&
      MotorOpenLoopTest_TimeReached(now, s_deadline_ms))
  {
    for (uint32_t wheel = 0U; wheel < MOTOR_DRIVER_WHEEL_COUNT; ++wheel)
    {
      s_validation_start_totals[wheel] = start_totals[wheel];
    }

    s_deadline_ms = now + MOTOR_TEST_DURATION_MS;
    s_state = MOTOR_TEST_STATE_VALIDATION_RUNNING;
    started = MotorDriver_Enable();

    for (uint32_t wheel = 0U;
         started && (wheel < MOTOR_DRIVER_WHEEL_COUNT);
         ++wheel)
    {
      started = MotorDriver_SetCommand((MotorDriverWheel)wheel,
                                       MOTOR_TEST_COMMAND_PERMILLE);
    }

    if (!started)
    {
      MotorDriver_Disable();
      s_state = MOTOR_TEST_STATE_DISARMED;
    }
  }
  taskEXIT_CRITICAL();

  if (started)
  {
    DEBUG_LOG_WARN("4W validation started: all FORWARD duty=15%% duration=800 ms");
  }
  else
  {
    DEBUG_LOG_ERROR("4W validation failed to start; all motors disabled");
  }
}

static void MotorOpenLoopTest_Stop(void)
{
  const MotorDriverWheel wheel = s_selected_wheel;
  MotorTestState previous_state;
  bool was_running;
  bool was_validation;

  taskENTER_CRITICAL();
  previous_state = s_state;
  was_running = (previous_state == MOTOR_TEST_STATE_RUNNING);
  was_validation = MotorOpenLoopTest_IsValidationState(previous_state);
  MotorDriver_Disable();
  s_state = MOTOR_TEST_STATE_DISARMED;
  s_arm_expired_notice = false;
  taskEXIT_CRITICAL();

  if (was_running)
  {
    MotorOpenLoopTest_ReportResult("stopped by command");
  }
  else if (was_validation)
  {
    if (previous_state == MOTOR_TEST_STATE_VALIDATION_RUNNING)
    {
      MotorOpenLoopTest_ReportValidationResult("stopped by command");
    }
    else
    {
      DEBUG_LOG_INFO("4W validation cancelled; all motor outputs disabled");
    }
  }
  else
  {
    DEBUG_LOG_INFO("%s test disarmed; all motor outputs disabled",
                   MotorOpenLoopTest_GetWheelName(wheel));
  }
}

void MotorOpenLoopTest_Init(void)
{
  taskENTER_CRITICAL();
  MotorDriver_Disable();
  s_state = MOTOR_TEST_STATE_DISARMED;
  s_selected_wheel = MOTOR_DRIVER_WHEEL_COUNT;
  s_deadline_ms = 0U;
  s_arm_expired_notice = false;
  s_start_total = 0;
  for (uint32_t wheel = 0U; wheel < MOTOR_DRIVER_WHEEL_COUNT; ++wheel)
  {
    s_validation_start_totals[wheel] = 0;
  }
  s_active_command = 0;
  taskEXIT_CRITICAL();

  MotorCharacterizationTest_Init();
  MotorSpeedControlTest_Init();
  FourWheelSpeedControlTest_Init();
  ChassisMotionTest_Init();
  ChassisGroundTest_Init();
  ChassisRuntimeTest_Init();

  DEBUG_LOG_WARN("Motor diagnostics ready (Debug only)");
  DEBUG_LOG_WARN("Commands: v=open-loop c=scan p=singlePI q=4PI k=liftedKIN b=ground");
  DEBUG_LOG_WARN("Selection: 1=LF 2=LR 3=RF 4=RR");
  DEBUG_LOG_WARN("Single wheel: a=arm, f=FORWARD, r=REVERSE; s=stop/cancel");
  DEBUG_LOG_WARN("Characterization: c then f=forward scan or r=reverse scan");
  DEBUG_LOG_WARN("PI validation: select 1/2/3/4, then p, then g");
  DEBUG_LOG_WARN("4-wheel PI sync: q then g; s=immediate stop/cancel");
  DEBUG_LOG_WARN("Chassis kinematics: k then g; lifted-wheel test only");
  DEBUG_LOG_WARN("Ground: b then w/x=straight a/d=pivot j/l=rolling turn; s=stop");
  DEBUG_LOG_WARN("Runtime ownership: n then g; 1 s forward + 250 ms timeout stop");
  DEBUG_LOG_WARN("Health injection: z then m/u/1/2/3/4; z0=disable z?=status");
  DEBUG_LOG_WARN("IMU stationary baseline: i=start 5 s collection; s=cancel");
  DEBUG_LOG_INFO("Motion trace: stop first, then t=dump last runtime motion (read-only)");
  DEBUG_LOG_WARN("Pulse is fixed at 15%% for 800 ms; keep all wheels lifted");
}

void MotorOpenLoopTest_Process(void)
{
  uint8_t command;
  bool command_received;
  bool completed = false;
  bool validation_completed = false;
  bool validation_start = false;
  bool arm_expired = false;
  const uint32_t now = HAL_GetTick();

  FaultInjectionTest_Process(now);
  command_received =
    (HAL_UART_Receive(&huart1, &command, 1U, 0U) == HAL_OK);

  if (command_received &&
      FaultInjectionTest_HandleCommand(command, now))
  {
    return;
  }

  if (command_received && FaultInjectionTest_IsEnabled() &&
      (command != 's') && (command != 'S') &&
      (command != '\r') && (command != '\n'))
  {
    DEBUG_LOG_WARN(
      "Command rejected while fault injection is enabled; use z0 (s remains emergency stop)");
    return;
  }

  if (command_received && ((command == 'i') || (command == 'I')))
  {
    const bool motion_active = MotorOpenLoopTest_IsActive() ||
                               ChassisRuntime_IsActive() ||
                               ChassisRuntimeTest_IsActive();
    (void)ImuStaticTest_Start(now, motion_active, IMU_STATIC_SOURCE_UART);
    return;
  }

  /* A queued stop command must win over a countdown expiring this cycle. */
  if (command_received && ((command == 's') || (command == 'S')))
  {
    ImuStaticTest_Cancel();
    if (ChassisRuntimeTest_IsActive())
    {
      ChassisRuntimeTest_Stop();
    }
    else if (ChassisGroundTest_IsActive())
    {
      ChassisGroundTest_Stop();
    }
    else if (ChassisMotionTest_IsActive())
    {
      ChassisMotionTest_Stop();
    }
    else if (FourWheelSpeedControlTest_IsActive())
    {
      FourWheelSpeedControlTest_Stop();
    }
    else if (MotorSpeedControlTest_IsActive())
    {
      MotorSpeedControlTest_Stop();
    }
    else if (MotorCharacterizationTest_IsActive())
    {
      MotorCharacterizationTest_Stop();
    }
    else if (ChassisRuntime_IsActive())
    {
      ChassisRuntime_Stop(now);
    }
    else
    {
      MotorOpenLoopTest_Stop();
    }
    return;
  }

  ChassisRuntimeTest_Process(now);
  MotorCharacterizationTest_Process(now);
  MotorSpeedControlTest_Process(now);
  FourWheelSpeedControlTest_Process(now);
  ChassisMotionTest_Process(now);
  ChassisGroundTest_Process(now);

  taskENTER_CRITICAL();
  if (s_state == MOTOR_TEST_STATE_COMPLETED)
  {
    s_state = MOTOR_TEST_STATE_DISARMED;
    completed = true;
  }
  else if (s_state == MOTOR_TEST_STATE_VALIDATION_COMPLETED)
  {
    s_state = MOTOR_TEST_STATE_DISARMED;
    validation_completed = true;
  }
  else if ((s_state == MOTOR_TEST_STATE_VALIDATION_COUNTDOWN) &&
           MotorOpenLoopTest_TimeReached(now, s_deadline_ms))
  {
    validation_start = true;
  }
  if (s_arm_expired_notice)
  {
    s_arm_expired_notice = false;
    arm_expired = true;
  }
  taskEXIT_CRITICAL();

  if (completed)
  {
    MotorOpenLoopTest_ReportResult("completed");
  }
  if (validation_completed)
  {
    MotorOpenLoopTest_ReportValidationResult("completed");
  }
  if (validation_start)
  {
    MotorOpenLoopTest_StartValidation(now);
  }
  if (arm_expired)
  {
    DEBUG_LOG_INFO("%s arm window expired; all motors remain disabled",
                   MotorOpenLoopTest_GetWheelName(s_selected_wheel));
  }

  if (!command_received)
  {
    return;
  }


  if (ImuStaticTest_IsActive())
  {
    if ((command != '\r') && (command != '\n'))
    {
      DEBUG_LOG_WARN("IMU STATIC active: keep still or send 's' to cancel");
    }
    return;
  }

  if (ChassisRuntimeTest_IsActive())
  {
    if ((command == 'g') || (command == 'G'))
    {
      (void)ChassisRuntimeTest_Start(HAL_GetTick());
    }
    else if ((command != '\r') && (command != '\n'))
    {
      DEBUG_LOG_WARN("RUNTIME test active: only 'g' start or 's' cancel is accepted");
    }
    return;
  }

  switch (command)
  {
    case 't':
    case 'T':
      if (MotorOpenLoopTest_IsActive() || !MotionTrace_RequestDump())
      {
        DEBUG_LOG_WARN("TRACE unavailable: stop chassis/diagnostics first; a completed runtime motion is required");
      }
      break;
    case 'c':
    case 'C':
      MotorOpenLoopTest_ArmCharacterization(HAL_GetTick());
      break;

    case 'v':
    case 'V':
      MotorOpenLoopTest_QueueValidation(HAL_GetTick());
      break;

    case 'p':
    case 'P':
      MotorOpenLoopTest_ArmSpeedControl(HAL_GetTick());
      break;

    case 'q':
    case 'Q':
      MotorOpenLoopTest_ArmFourWheelSpeedControl(HAL_GetTick());
      break;

    case 'k':
    case 'K':
      MotorOpenLoopTest_ArmChassisMotion(HAL_GetTick());
      break;

    case 'b':
    case 'B':
      MotorOpenLoopTest_ArmGroundMotion(HAL_GetTick());
      break;

    case 'n':
    case 'N':
      if (MotorOpenLoopTest_IsActive())
      {
        DEBUG_LOG_WARN("RUNTIME test rejected: another motor diagnostic is active");
      }
      else
      {
        (void)ChassisRuntimeTest_Arm(HAL_GetTick());
      }
      break;

    case 'g':
    case 'G':
      if (ChassisGroundTest_IsActive())
      {
        DEBUG_LOG_WARN("GROUND action required: w/x=straight a/d=pivot j/l=rolling turn");
      }
      else if (ChassisMotionTest_IsActive())
      {
        (void)ChassisMotionTest_Start(HAL_GetTick());
      }
      else if (FourWheelSpeedControlTest_IsActive())
      {
        (void)FourWheelSpeedControlTest_Start(HAL_GetTick());
      }
      else
      {
        (void)MotorSpeedControlTest_Start(HAL_GetTick());
      }
      break;

    case '1':
      MotorOpenLoopTest_Select(MOTOR_DRIVER_WHEEL_LF);
      break;

    case '2':
      MotorOpenLoopTest_Select(MOTOR_DRIVER_WHEEL_LR);
      break;

    case '3':
      MotorOpenLoopTest_Select(MOTOR_DRIVER_WHEEL_RF);
      break;

    case '4':
      MotorOpenLoopTest_Select(MOTOR_DRIVER_WHEEL_RR);
      break;

    case 'a':
    case 'A':
      if (ChassisGroundTest_IsArmed())
      {
        MotorOpenLoopTest_StartGroundMotion(
          CHASSIS_GROUND_ACTION_TURN_LEFT,
          HAL_GetTick());
      }
      else
      {
        MotorOpenLoopTest_Arm(HAL_GetTick());
      }
      break;

    case 'w':
    case 'W':
      MotorOpenLoopTest_StartGroundMotion(CHASSIS_GROUND_ACTION_FORWARD,
                                          HAL_GetTick());
      break;

    case 'x':
    case 'X':
      MotorOpenLoopTest_StartGroundMotion(CHASSIS_GROUND_ACTION_REVERSE,
                                          HAL_GetTick());
      break;

    case 'd':
    case 'D':
      MotorOpenLoopTest_StartGroundMotion(CHASSIS_GROUND_ACTION_TURN_RIGHT,
                                          HAL_GetTick());
      break;

    case 'j':
    case 'J':
      MotorOpenLoopTest_StartGroundMotion(CHASSIS_GROUND_ACTION_ARC_LEFT,
                                          HAL_GetTick());
      break;

    case 'l':
    case 'L':
      MotorOpenLoopTest_StartGroundMotion(CHASSIS_GROUND_ACTION_ARC_RIGHT,
                                          HAL_GetTick());
      break;

    case 'f':
    case 'F':
      MotorOpenLoopTest_HandleDirectionCommand(MOTOR_TEST_COMMAND_PERMILLE,
                                               HAL_GetTick());
      break;

    case 'r':
    case 'R':
      MotorOpenLoopTest_HandleDirectionCommand(-MOTOR_TEST_COMMAND_PERMILLE,
                                               HAL_GetTick());
      break;

    case '\r':
    case '\n':
      break;

    default:
      DEBUG_LOG_WARN("Unknown command 0x%02X; use v/c/p/q/k/b/n/i/g/w/x/a/d/j/l/1/2/3/4/f/r/s/z",
                     (unsigned int)command);
      break;
  }
}

void MotorOpenLoopTest_SafetyProcess(void)
{
  const uint32_t now = HAL_GetTick();

  taskENTER_CRITICAL();
  if (EmergencyStop_IsLatched() &&
      (s_state != MOTOR_TEST_STATE_DISARMED))
  {
    MotorDriver_Disable();
    s_state = MOTOR_TEST_STATE_DISARMED;
    s_arm_expired_notice = false;
  }
  else if ((s_state == MOTOR_TEST_STATE_RUNNING) &&
      MotorOpenLoopTest_TimeReached(now, s_deadline_ms))
  {
    MotorDriver_Disable();
    s_state = MOTOR_TEST_STATE_COMPLETED;
  }
  else if ((s_state == MOTOR_TEST_STATE_VALIDATION_RUNNING) &&
           MotorOpenLoopTest_TimeReached(now, s_deadline_ms))
  {
    MotorDriver_Disable();
    s_state = MOTOR_TEST_STATE_VALIDATION_COMPLETED;
  }
  else if ((s_state == MOTOR_TEST_STATE_ARMED) &&
           MotorOpenLoopTest_TimeReached(now, s_deadline_ms))
  {
    MotorDriver_Disable();
    s_state = MOTOR_TEST_STATE_DISARMED;
    s_arm_expired_notice = true;
  }
  taskEXIT_CRITICAL();

  MotorCharacterizationTest_SafetyProcess(now);
  MotorSpeedControlTest_SafetyProcess(now);
  FourWheelSpeedControlTest_SafetyProcess(now);
  ChassisMotionTest_SafetyProcess(now);
  ChassisGroundTest_SafetyProcess(now);
  ChassisRuntimeTest_SafetyProcess(now);
}

bool MotorOpenLoopTest_IsActive(void)
{
  MotorTestState state;

  taskENTER_CRITICAL();
  state = s_state;
  taskEXIT_CRITICAL();
  return (state != MOTOR_TEST_STATE_DISARMED) ||
         MotorCharacterizationTest_IsActive() ||
         MotorSpeedControlTest_IsActive() ||
         FourWheelSpeedControlTest_IsActive() ||
         ChassisMotionTest_IsActive() ||
         ChassisGroundTest_IsActive();
}

void MotorOpenLoopTest_StopAll(uint32_t now_ms)
{
  if (ChassisRuntimeTest_IsActive())
  {
    ChassisRuntimeTest_Stop();
  }
  if (ChassisGroundTest_IsActive())
  {
    ChassisGroundTest_Stop();
  }
  if (ChassisMotionTest_IsActive())
  {
    ChassisMotionTest_Stop();
  }
  if (FourWheelSpeedControlTest_IsActive())
  {
    FourWheelSpeedControlTest_Stop();
  }
  if (MotorSpeedControlTest_IsActive())
  {
    MotorSpeedControlTest_Stop();
  }
  if (MotorCharacterizationTest_IsActive())
  {
    MotorCharacterizationTest_Stop();
  }
  if (ChassisRuntime_IsActive())
  {
    ChassisRuntime_Stop(now_ms);
  }
  MotorOpenLoopTest_Stop();
}

#else

void MotorOpenLoopTest_Init(void)
{
}

void MotorOpenLoopTest_Process(void)
{
}

void MotorOpenLoopTest_SafetyProcess(void)
{
}

void MotorOpenLoopTest_StopAll(uint32_t now_ms)
{
  (void)now_ms;
}

bool MotorOpenLoopTest_IsActive(void)
{
  return false;
}

#endif /* MOTOR_OPEN_LOOP_TEST_ENABLED */
