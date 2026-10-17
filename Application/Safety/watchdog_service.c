/**
 * @file watchdog_service.c
 * @brief IWDG refresh arbitration based on live system/task health.
 */

#include "watchdog_service.h"

#include <stddef.h>

#include "FreeRTOS.h"
#include "board_config.h"
#include "debug_log.h"
#include "fault_manager.h"
#include "iwdg.h"
#include "task.h"

#define WATCHDOG_SERVICE_REFRESH_PERIOD_MS 250U
#define WATCHDOG_SERVICE_ALL_TASK_MASK \
  ((1UL << (uint32_t)FAULT_MANAGER_TASK_COUNT) - 1UL)

_Static_assert(FAULT_MANAGER_TASK_COUNT > 0,
               "Watchdog requires at least one supervised task");
_Static_assert(FAULT_MANAGER_TASK_COUNT < 32,
               "Task mask must fit in uint32_t");

static WatchdogServiceStatus s_status;

static bool WatchdogService_IsHealthEligible(
  const FaultManagerStatus *health)
{
  return (health != NULL) &&
         health->initialized &&
         health->system_init_complete &&
         ((health->seen_task_mask & WATCHDOG_SERVICE_ALL_TASK_MASK) ==
          WATCHDOG_SERVICE_ALL_TASK_MASK) &&
         ((health->stale_task_mask & WATCHDOG_SERVICE_ALL_TASK_MASK) == 0U);
}

void WatchdogService_Init(uint32_t now_ms)
{
  taskENTER_CRITICAL();
  s_status.initialized = true;
  s_status.armed = false;
  s_status.health_eligible = false;
  s_status.refresh_failed = false;
  s_status.last_refresh_ms = now_ms;
  s_status.refresh_count = 0U;
  s_status.withhold_event_count = 0U;
  taskEXIT_CRITICAL();

  DEBUG_LOG_INFO(
    "IWDG arbiter ready: CubeMX timeout nominal=4s, refresh=%lums, owner=safetyTask",
    (unsigned long)WATCHDOG_SERVICE_REFRESH_PERIOD_MS);
}

void WatchdogService_Process(uint32_t now_ms)
{
  FaultManagerStatus health = { 0 };
  bool previous_eligible;
  bool armed;
  bool refresh_failed;
  bool health_eligible;
  bool should_refresh = false;
  bool report_armed = false;
  bool report_withheld = false;
  bool report_recovered = false;
  bool report_refresh_failure = false;

  (void)FaultManager_GetStatus(&health);
  health_eligible = WatchdogService_IsHealthEligible(&health);

  taskENTER_CRITICAL();
  if (!s_status.initialized)
  {
    taskEXIT_CRITICAL();
    return;
  }
  previous_eligible = s_status.health_eligible;
  armed = s_status.armed;
  refresh_failed = s_status.refresh_failed;
  s_status.health_eligible = health_eligible;

  if (health_eligible && (!armed))
  {
    s_status.armed = true;
    armed = true;
    report_armed = true;
  }
  else if (armed && previous_eligible && (!health_eligible))
  {
    ++s_status.withhold_event_count;
    report_withheld = true;
  }
  else if (armed && (!previous_eligible) && health_eligible)
  {
    report_recovered = true;
  }

  if (armed && health_eligible && (!refresh_failed) &&
      (report_armed || report_recovered ||
       ((now_ms - s_status.last_refresh_ms) >=
        WATCHDOG_SERVICE_REFRESH_PERIOD_MS)))
  {
    should_refresh = true;
  }
  taskEXIT_CRITICAL();

  if (report_armed)
  {
    DEBUG_LOG_INFO(
      "IWDG refresh armed: system init complete, task_mask=0x%02lX",
      (unsigned long)(health.seen_task_mask & WATCHDOG_SERVICE_ALL_TASK_MASK));
  }
  if (report_withheld)
  {
    DEBUG_LOG_ERROR(
      "IWDG refresh withheld: seen=0x%02lX stale=0x%02lX system_init=%u",
      (unsigned long)(health.seen_task_mask & WATCHDOG_SERVICE_ALL_TASK_MASK),
      (unsigned long)(health.stale_task_mask & WATCHDOG_SERVICE_ALL_TASK_MASK),
      health.system_init_complete ? 1U : 0U);
  }
  if (report_recovered)
  {
    DEBUG_LOG_INFO(
      "IWDG refresh resumed: all supervised task heartbeats are current");
  }

  if (should_refresh)
  {
    if (HAL_IWDG_Refresh(&hiwdg) == HAL_OK)
    {
      taskENTER_CRITICAL();
      s_status.last_refresh_ms = now_ms;
      ++s_status.refresh_count;
      taskEXIT_CRITICAL();
    }
    else
    {
      taskENTER_CRITICAL();
      if (!s_status.refresh_failed)
      {
        s_status.refresh_failed = true;
        report_refresh_failure = true;
      }
      taskEXIT_CRITICAL();
      Board_EmergencyMotorShutdown();
      if (report_refresh_failure)
      {
        DEBUG_LOG_ERROR(
          "IWDG refresh failed: motor outputs disabled; hardware reset pending");
      }
    }
  }
}

bool WatchdogService_GetStatus(WatchdogServiceStatus *status)
{
  if (status == NULL)
  {
    return false;
  }

  taskENTER_CRITICAL();
  *status = s_status;
  taskEXIT_CRITICAL();
  return status->initialized;
}
