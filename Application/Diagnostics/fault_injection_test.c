/**
 * @file fault_injection_test.c
 * @brief Debug-only, software fault injection for the health manager.
 */

#include "fault_injection_test.h"

#include <stddef.h>

#if FAULT_INJECTION_TEST_ENABLED

#include "FreeRTOS.h"
#include "chassis_runtime.h"
#include "debug_log.h"
#include "task.h"

#define FAULT_INJECTION_TASK_BIT(task) (1UL << (uint32_t)(task))
#define FAULT_INJECTION_PREFIX_TIMEOUT_MS 5000U

static uint32_t s_suppressed_heartbeat_mask;
static uint32_t s_zero_feedback_wheel_mask;
static bool s_prefix_pending;
static uint32_t s_prefix_deadline_ms;

static bool FaultInjectionTest_TimeReached(uint32_t now_ms,
                                           uint32_t deadline_ms)
{
  return ((int32_t)(now_ms - deadline_ms) >= 0);
}

static const char *FaultInjectionTest_GetTaskName(uint32_t mask)
{
  if (mask == FAULT_INJECTION_TASK_BIT(FAULT_MANAGER_TASK_MOTOR))
  {
    return "motorTask";
  }
  if (mask == FAULT_INJECTION_TASK_BIT(FAULT_MANAGER_TASK_USB_RX))
  {
    return "usbRxTask";
  }
  return "none";
}

static const char *FaultInjectionTest_GetWheelName(uint32_t mask)
{
  switch (mask)
  {
    case (1UL << MOTOR_DRIVER_WHEEL_LF):
      return "LF";
    case (1UL << MOTOR_DRIVER_WHEEL_LR):
      return "LR";
    case (1UL << MOTOR_DRIVER_WHEEL_RF):
      return "RF";
    case (1UL << MOTOR_DRIVER_WHEEL_RR):
      return "RR";
    default:
      return "none";
  }
}

static void FaultInjectionTest_ReportStatus(void)
{
  uint32_t heartbeat_mask;
  uint32_t wheel_mask;

  taskENTER_CRITICAL();
  heartbeat_mask = s_suppressed_heartbeat_mask;
  wheel_mask = s_zero_feedback_wheel_mask;
  taskEXIT_CRITICAL();

  DEBUG_LOG_WARN(
    "FAULT INJECTION status: heartbeat=%s wheel_feedback_zero=%s",
    FaultInjectionTest_GetTaskName(heartbeat_mask),
    FaultInjectionTest_GetWheelName(wheel_mask));
}

void FaultInjectionTest_Init(void)
{
  taskENTER_CRITICAL();
  s_suppressed_heartbeat_mask = 0U;
  s_zero_feedback_wheel_mask = 0U;
  s_prefix_pending = false;
  s_prefix_deadline_ms = 0U;
  taskEXIT_CRITICAL();

  DEBUG_LOG_WARN(
    "Fault injection ready (Debug only): z then m/u/1/2/3/4, z0=disable, z?=status");
}

void FaultInjectionTest_Process(uint32_t now_ms)
{
  bool expired = false;

  taskENTER_CRITICAL();
  if (s_prefix_pending &&
      FaultInjectionTest_TimeReached(now_ms, s_prefix_deadline_ms))
  {
    s_prefix_pending = false;
    expired = true;
  }
  taskEXIT_CRITICAL();

  if (expired)
  {
    DEBUG_LOG_INFO("Fault injection command prefix expired; no state changed");
  }
}

bool FaultInjectionTest_HandleCommand(uint8_t command, uint32_t now_ms)
{
  bool prefix_pending;
  bool valid_selector = true;
  uint32_t heartbeat_mask = 0U;
  uint32_t wheel_mask = 0U;
  ChassisRuntimeStatus runtime = { 0 };

  taskENTER_CRITICAL();
  prefix_pending = s_prefix_pending;
  taskEXIT_CRITICAL();

  if (!prefix_pending)
  {
    if ((command != 'z') && (command != 'Z'))
    {
      return false;
    }

    taskENTER_CRITICAL();
    s_prefix_pending = true;
    s_prefix_deadline_ms = now_ms + FAULT_INJECTION_PREFIX_TIMEOUT_MS;
    taskEXIT_CRITICAL();
    DEBUG_LOG_WARN(
      "FAULT INJECTION select: m=motor heartbeat u=USB heartbeat 1=LF 2=LR 3=RF 4=RR 0=off ?=status");
    return true;
  }

  taskENTER_CRITICAL();
  s_prefix_pending = false;
  taskEXIT_CRITICAL();

  if ((command == '0') || (command == 's') || (command == 'S'))
  {
    taskENTER_CRITICAL();
    s_suppressed_heartbeat_mask = 0U;
    s_zero_feedback_wheel_mask = 0U;
    taskEXIT_CRITICAL();
    DEBUG_LOG_INFO(
      "FAULT INJECTION disabled; any latched health fault still requires USB CLEAR_FAULT");
    return true;
  }

  if (command == '?')
  {
    FaultInjectionTest_ReportStatus();
    return true;
  }

  if ((!ChassisRuntime_GetStatus(&runtime)) ||
      (runtime.state != CHASSIS_RUNTIME_STATE_IDLE) ||
      runtime.diagnostics_active)
  {
    DEBUG_LOG_WARN(
      "FAULT INJECTION rejected: chassis must be IDLE with diagnostics inactive");
    return true;
  }

  switch (command)
  {
    case 'm':
    case 'M':
      heartbeat_mask =
        FAULT_INJECTION_TASK_BIT(FAULT_MANAGER_TASK_MOTOR);
      break;
    case 'u':
    case 'U':
      heartbeat_mask =
        FAULT_INJECTION_TASK_BIT(FAULT_MANAGER_TASK_USB_RX);
      break;
    case '1':
      wheel_mask = (1UL << MOTOR_DRIVER_WHEEL_LF);
      break;
    case '2':
      wheel_mask = (1UL << MOTOR_DRIVER_WHEEL_LR);
      break;
    case '3':
      wheel_mask = (1UL << MOTOR_DRIVER_WHEEL_RF);
      break;
    case '4':
      wheel_mask = (1UL << MOTOR_DRIVER_WHEEL_RR);
      break;
    default:
      valid_selector = false;
      break;
  }

  if (!valid_selector)
  {
    DEBUG_LOG_WARN("FAULT INJECTION unknown selector 0x%02X; no state changed",
                   (unsigned int)command);
  }
  else
  {
    taskENTER_CRITICAL();
    s_suppressed_heartbeat_mask = heartbeat_mask;
    s_zero_feedback_wheel_mask = wheel_mask;
    taskEXIT_CRITICAL();
    DEBUG_LOG_WARN(
      "FAULT INJECTION enabled: heartbeat=%s wheel_feedback_zero=%s",
      FaultInjectionTest_GetTaskName(heartbeat_mask),
      FaultInjectionTest_GetWheelName(wheel_mask));
  }
  return true;
}

bool FaultInjectionTest_ShouldReportHeartbeat(FaultManagerTask task)
{
  bool report;

  if ((uint32_t)task >= (uint32_t)FAULT_MANAGER_TASK_COUNT)
  {
    return true;
  }
  taskENTER_CRITICAL();
  report = (s_suppressed_heartbeat_mask &
            FAULT_INJECTION_TASK_BIT(task)) == 0U;
  taskEXIT_CRITICAL();
  return report;
}

void FaultInjectionTest_FilterMeasuredSpeed(
  const float source[MOTOR_DRIVER_WHEEL_COUNT],
  float filtered[MOTOR_DRIVER_WHEEL_COUNT])
{
  uint32_t wheel_mask;

  if (filtered == NULL)
  {
    return;
  }
  taskENTER_CRITICAL();
  wheel_mask = s_zero_feedback_wheel_mask;
  taskEXIT_CRITICAL();

  for (uint32_t wheel = 0U; wheel < MOTOR_DRIVER_WHEEL_COUNT; ++wheel)
  {
    filtered[wheel] = ((source == NULL) ||
      ((wheel_mask & (1UL << wheel)) != 0U)) ? 0.0f : source[wheel];
  }
}

bool FaultInjectionTest_IsEnabled(void)
{
  bool enabled;

  taskENTER_CRITICAL();
  enabled = (s_suppressed_heartbeat_mask != 0U) ||
            (s_zero_feedback_wheel_mask != 0U);
  taskEXIT_CRITICAL();
  return enabled;
}

#else

void FaultInjectionTest_Init(void)
{
}

void FaultInjectionTest_Process(uint32_t now_ms)
{
  (void)now_ms;
}

bool FaultInjectionTest_HandleCommand(uint8_t command, uint32_t now_ms)
{
  (void)command;
  (void)now_ms;
  return false;
}

bool FaultInjectionTest_ShouldReportHeartbeat(FaultManagerTask task)
{
  (void)task;
  return true;
}

void FaultInjectionTest_FilterMeasuredSpeed(
  const float source[MOTOR_DRIVER_WHEEL_COUNT],
  float filtered[MOTOR_DRIVER_WHEEL_COUNT])
{
  if (filtered == NULL)
  {
    return;
  }
  for (uint32_t wheel = 0U; wheel < MOTOR_DRIVER_WHEEL_COUNT; ++wheel)
  {
    filtered[wheel] = (source == NULL) ? 0.0f : source[wheel];
  }
}

bool FaultInjectionTest_IsEnabled(void)
{
  return false;
}

#endif /* FAULT_INJECTION_TEST_ENABLED */
