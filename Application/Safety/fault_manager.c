/**
 * @file fault_manager.c
 * @brief Central task-health and production wheel-stall supervision.
 */

#include "fault_manager.h"

#include <stddef.h>

#include "FreeRTOS.h"
#include "debug_log.h"
#include "task.h"
#include "wheel_speed_controller.h"
#include "wheel_speed_control_config.h"
#include "pivot_feedback_guard.h"

#define FAULT_MANAGER_TASK_BIT(task) (1UL << (uint32_t)(task))

#define FAULT_MANAGER_STARTUP_GRACE_MS             3000U
#define FAULT_MANAGER_MOTOR_TIMEOUT_MS               250U
#define FAULT_MANAGER_SAFETY_TIMEOUT_MS              100U
#define FAULT_MANAGER_USB_RX_TIMEOUT_MS              250U
#define FAULT_MANAGER_TELEMETRY_TIMEOUT_MS          1000U
#define FAULT_MANAGER_IMU_TIMEOUT_MS                2500U
#define FAULT_MANAGER_BATTERY_TIMEOUT_MS            2500U

#define FAULT_MANAGER_CRITICAL_TASK_MASK \
  (FAULT_MANAGER_TASK_BIT(FAULT_MANAGER_TASK_MOTOR) | \
   FAULT_MANAGER_TASK_BIT(FAULT_MANAGER_TASK_USB_RX))

#define FAULT_MANAGER_STALL_STARTUP_GRACE_MS        1000U
#define FAULT_MANAGER_STALL_DETECT_MS                750U
#define FAULT_MANAGER_STALL_MIN_TARGET_CPS          200.0f
#define FAULT_MANAGER_STALL_MIN_OUTPUT_PERMILLE      100
#define FAULT_MANAGER_STALL_MIN_SPEED_RATIO            0.15f

static const uint32_t s_task_timeouts_ms[FAULT_MANAGER_TASK_COUNT] = {
  [FAULT_MANAGER_TASK_MOTOR] = FAULT_MANAGER_MOTOR_TIMEOUT_MS,
  [FAULT_MANAGER_TASK_SAFETY] = FAULT_MANAGER_SAFETY_TIMEOUT_MS,
  [FAULT_MANAGER_TASK_USB_RX] = FAULT_MANAGER_USB_RX_TIMEOUT_MS,
  [FAULT_MANAGER_TASK_TELEMETRY] = FAULT_MANAGER_TELEMETRY_TIMEOUT_MS,
  [FAULT_MANAGER_TASK_IMU] = FAULT_MANAGER_IMU_TIMEOUT_MS,
  [FAULT_MANAGER_TASK_BATTERY] = FAULT_MANAGER_BATTERY_TIMEOUT_MS
};

static FaultManagerStatus s_status;
static uint32_t s_start_ms;
static bool s_startup_reported;
static uint32_t s_last_heartbeat_ms[FAULT_MANAGER_TASK_COUNT];
static bool s_motion_active;
static uint32_t s_motion_start_ms;
static bool s_stall_condition_active[MOTOR_DRIVER_WHEEL_COUNT];
static uint32_t s_stall_condition_start_ms[MOTOR_DRIVER_WHEEL_COUNT];
static float s_previous_targets[MOTOR_DRIVER_WHEEL_COUNT];
static PivotFeedbackGuard s_pivot_guard[MOTOR_DRIVER_WHEEL_COUNT];
static uint32_t s_pivot_warning_mask;

static float FaultManager_AbsFloat(float value)
{
  return (value < 0.0f) ? -value : value;
}

static int32_t FaultManager_AbsI16(int16_t value)
{
  return (value < 0) ? -(int32_t)value : (int32_t)value;
}

static bool FaultManager_TargetDirectionChanged(float previous,
                                                float current)
{
  return ((previous > 0.0f) && (current < 0.0f)) ||
         ((previous < 0.0f) && (current > 0.0f));
}

static void FaultManager_ResetMotionTrackingLocked(void)
{
  s_motion_active = false;
  s_motion_start_ms = 0U;
  s_status.current_stalled_wheel_mask = 0U;
  s_pivot_warning_mask=0U;
  for (uint32_t wheel = 0U; wheel < MOTOR_DRIVER_WHEEL_COUNT; ++wheel)
  {
    s_stall_condition_active[wheel] = false;
    s_stall_condition_start_ms[wheel] = 0U;
    s_previous_targets[wheel] = 0.0f;
    s_pivot_guard[wheel]=(PivotFeedbackGuard){0};
  }
}

static FaultManagerFaultFlags FaultManager_LatchLocked(
  FaultManagerFaultFlags faults,
  uint32_t now_ms)
{
  const FaultManagerFaultFlags new_faults =
    faults & ~s_status.latched_faults;

  if (new_faults != FAULT_MANAGER_FAULT_NONE)
  {
    if (s_status.latched_faults == FAULT_MANAGER_FAULT_NONE)
    {
      s_status.first_fault_ms = now_ms;
    }
    s_status.latched_faults |= new_faults;
    s_status.last_fault_ms = now_ms;
    ++s_status.trip_count;
  }
  return new_faults;
}

void FaultManager_Init(uint32_t now_ms)
{
  taskENTER_CRITICAL();
  s_status.initialized = true;
  s_status.system_init_complete = false;
  s_status.latched_faults = FAULT_MANAGER_FAULT_NONE;
  s_status.seen_task_mask = 0U;
  s_status.stale_task_mask = 0U;
  s_status.critical_stale_task_mask = 0U;
  s_status.latched_stalled_wheel_mask = 0U;
  s_status.current_stalled_wheel_mask = 0U;
  s_status.first_fault_ms = 0U;
  s_status.last_fault_ms = 0U;
  s_status.trip_count = 0U;
  s_start_ms = now_ms;
  s_startup_reported = false;
  for (uint32_t task = 0U; task < (uint32_t)FAULT_MANAGER_TASK_COUNT; ++task)
  {
    s_last_heartbeat_ms[task] = now_ms;
  }
  FaultManager_ResetMotionTrackingLocked();
  taskEXIT_CRITICAL();

  DEBUG_LOG_INFO(
    "Fault manager ready: task startup grace=%lums, wheel stall target>=%ldcps grace=%lums detect=%lums",
    (unsigned long)FAULT_MANAGER_STARTUP_GRACE_MS,
    (long)FAULT_MANAGER_STALL_MIN_TARGET_CPS,
    (unsigned long)FAULT_MANAGER_STALL_STARTUP_GRACE_MS,
    (unsigned long)FAULT_MANAGER_STALL_DETECT_MS);
#if CHASSIS_CONTROL_REVISION >= 201
  DEBUG_LOG_INFO("Continuous feedback guard: grace=%lums warn=%lums persistent-low=%lums recovery=%lums; all motion modes",
#else
  DEBUG_LOG_INFO("Unified pivot guard: grace=%lums warn=%lums persistent-low=%lums recovery=%lums; speed-only",
#endif
    (unsigned long)PIVOT_GUARD_GRACE_MS,(unsigned long)PIVOT_GUARD_WARN_MS,
    (unsigned long)PIVOT_GUARD_STOP_MS,(unsigned long)PIVOT_GUARD_RECOVER_MS);
}

void FaultManager_Heartbeat(FaultManagerTask task, uint32_t now_ms)
{
  if ((uint32_t)task >= (uint32_t)FAULT_MANAGER_TASK_COUNT)
  {
    return;
  }

  taskENTER_CRITICAL();
  s_last_heartbeat_ms[task] = now_ms;
  s_status.seen_task_mask |= FAULT_MANAGER_TASK_BIT(task);
  taskEXIT_CRITICAL();
}

void FaultManager_MarkSystemInitComplete(uint32_t now_ms)
{
  (void)now_ms;
  taskENTER_CRITICAL();
  s_status.system_init_complete = true;
  taskEXIT_CRITICAL();
}

void FaultManager_ObserveMotionWithModes(
  uint32_t now_ms,
  bool runtime_active,
  const float target_counts_per_second[MOTOR_DRIVER_WHEEL_COUNT],
  const float measured_counts_per_second[MOTOR_DRIVER_WHEEL_COUNT],
  const int16_t output_permille[MOTOR_DRIVER_WHEEL_COUNT], uint32_t pivot_mask)
{
  uint32_t newly_stalled_mask = 0U;
  FaultManagerFaultFlags new_faults = FAULT_MANAGER_FAULT_NONE;

  taskENTER_CRITICAL();
  if ((!s_status.initialized) || (!runtime_active) ||
      (target_counts_per_second == NULL) ||
      (measured_counts_per_second == NULL) ||
      (output_permille == NULL))
  {
    FaultManager_ResetMotionTrackingLocked();
    taskEXIT_CRITICAL();
    return;
  }

  if (!s_motion_active)
  {
    s_motion_active = true;
    s_motion_start_ms = now_ms;
    for (uint32_t wheel = 0U; wheel < MOTOR_DRIVER_WHEEL_COUNT; ++wheel)
    {
      s_previous_targets[wheel] = target_counts_per_second[wheel];
    }
  }

  s_status.current_stalled_wheel_mask = 0U;
  const uint32_t previous_warning_mask=s_pivot_warning_mask;
  s_pivot_warning_mask=0U;
  for (uint32_t wheel = 0U; wheel < MOTOR_DRIVER_WHEEL_COUNT; ++wheel)
  {
    const float target = target_counts_per_second[wheel];
    const float target_magnitude = FaultManager_AbsFloat(target);
    const float directed_speed = (target >= 0.0f) ?
      measured_counts_per_second[wheel] :
      -measured_counts_per_second[wheel];
    const float minimum_speed =
      target_magnitude * FAULT_MANAGER_STALL_MIN_SPEED_RATIO;
    const bool direction_changed = FaultManager_TargetDirectionChanged(
      s_previous_targets[wheel], target);
    const bool grace_complete =
      ((now_ms - s_motion_start_ms) >=
       FAULT_MANAGER_STALL_STARTUP_GRACE_MS);
    const bool unified_owned=(pivot_mask & (1UL<<wheel))!=0U;
    const bool unified_failed=PivotFeedbackGuard_Update(&s_pivot_guard[wheel],
      CHASSIS_CONTROL_V2 || unified_owned,target,measured_counts_per_second[wheel],now_ms);
    if (s_pivot_guard[wheel].warning) s_pivot_warning_mask|=1UL<<wheel;
    if (unified_owned || (CHASSIS_CONTROL_V2 && unified_failed))
    {
      s_stall_condition_active[wheel]=false;
      s_stall_condition_start_ms[wheel]=0U;
      /* Pivot feedback qualification has exactly one owner. */
      s_previous_targets[wheel]=target;
      if (unified_failed)
      {
        const uint32_t bit=1UL<<wheel;
        s_status.current_stalled_wheel_mask|=bit;
        if (!(s_status.latched_stalled_wheel_mask&bit)) newly_stalled_mask|=bit;
        s_status.latched_stalled_wheel_mask|=bit;
      }
      continue;
    }
    const bool stall_condition =
      grace_complete &&
      (!direction_changed) &&
      (target_magnitude >= FAULT_MANAGER_STALL_MIN_TARGET_CPS) &&
      (directed_speed < minimum_speed) &&
      (FaultManager_AbsI16(output_permille[wheel]) >=
       FAULT_MANAGER_STALL_MIN_OUTPUT_PERMILLE);
    s_previous_targets[wheel] = target;
    if (!stall_condition)
    {
      s_stall_condition_active[wheel] = false;
      s_stall_condition_start_ms[wheel] = 0U;
      continue;
    }

    if (!s_stall_condition_active[wheel])
    {
      s_stall_condition_active[wheel] = true;
      s_stall_condition_start_ms[wheel] = now_ms;
    }
    else if ((now_ms - s_stall_condition_start_ms[wheel]) >=
             FAULT_MANAGER_STALL_DETECT_MS)
    {
      const uint32_t wheel_bit = (1UL << wheel);
      s_status.current_stalled_wheel_mask |= wheel_bit;
      if ((s_status.latched_stalled_wheel_mask & wheel_bit) == 0U)
      {
        newly_stalled_mask |= wheel_bit;
      }
      s_status.latched_stalled_wheel_mask |= wheel_bit;
    }
  }

  if (s_status.current_stalled_wheel_mask != 0U)
  {
    new_faults = FaultManager_LatchLocked(
      FAULT_MANAGER_FAULT_WHEEL_STALL, now_ms);
  }
  const uint32_t warning_mask=s_pivot_warning_mask;
  taskEXIT_CRITICAL();

  if (previous_warning_mask!=warning_mask)
#if CHASSIS_CONTROL_REVISION >= 201
    DEBUG_LOG_WARN("Feedback guard suspect_mask=0x%02lX; persistent low feedback latches at %lums",
#else
    DEBUG_LOG_WARN("Pivot feedback guard suspect_mask=0x%02lX; persistent low feedback latches at %lums",
#endif
      (unsigned long)warning_mask,(unsigned long)PIVOT_GUARD_STOP_MS);

  if ((new_faults & FAULT_MANAGER_FAULT_WHEEL_STALL) != 0U)
  {
    DEBUG_LOG_ERROR(
      "Fault manager latched wheel stall: wheel_mask=0x%02lX; production motion inhibited",
      (unsigned long)newly_stalled_mask);
  }
}

void FaultManager_ObserveMotion(uint32_t now_ms,bool active,
  const float target[MOTOR_DRIVER_WHEEL_COUNT],
  const float measured[MOTOR_DRIVER_WHEEL_COUNT],
  const int16_t output[MOTOR_DRIVER_WHEEL_COUNT])
{
  uint32_t mask=0U;
  taskENTER_CRITICAL();
  for (uint32_t w=0U; w<MOTOR_DRIVER_WHEEL_COUNT; ++w)
    if (WheelSpeedController_IsPivotMode((MotorDriverWheel)w)) mask|=1UL<<w;
  taskEXIT_CRITICAL();
  FaultManager_ObserveMotionWithModes(now_ms,active,target,measured,output,mask);
}

void FaultManager_Process(uint32_t now_ms)
{
  uint32_t heartbeat_ms[FAULT_MANAGER_TASK_COUNT];
  uint32_t seen_mask;
  uint32_t previous_stale_mask;
  uint32_t stale_mask = 0U;
  uint32_t critical_stale_mask;
  bool system_init_complete;
  bool startup_grace_complete;
  FaultManagerFaultFlags faults = FAULT_MANAGER_FAULT_NONE;
  FaultManagerFaultFlags new_faults;
  bool report_startup = false;

  taskENTER_CRITICAL();
  if (!s_status.initialized)
  {
    taskEXIT_CRITICAL();
    return;
  }
  for (uint32_t task = 0U; task < (uint32_t)FAULT_MANAGER_TASK_COUNT; ++task)
  {
    heartbeat_ms[task] = s_last_heartbeat_ms[task];
  }
  seen_mask = s_status.seen_task_mask;
  previous_stale_mask = s_status.stale_task_mask;
  system_init_complete = s_status.system_init_complete;
  startup_grace_complete =
    ((now_ms - s_start_ms) >= FAULT_MANAGER_STARTUP_GRACE_MS);
  taskEXIT_CRITICAL();

  for (uint32_t task = 0U; task < (uint32_t)FAULT_MANAGER_TASK_COUNT; ++task)
  {
    const uint32_t task_bit = FAULT_MANAGER_TASK_BIT(task);

    if (((seen_mask & task_bit) == 0U) ?
        startup_grace_complete :
        /* A same-priority producer can publish a newer heartbeat after the
         * caller sampled now. Signed modular age treats that as fresh, while
         * preserving real timeouts and wrap (intervals must be < 2^31 ms). */
        ((int32_t)(now_ms - heartbeat_ms[task]) >
         (int32_t)s_task_timeouts_ms[task]))
    {
      stale_mask |= task_bit;
    }
  }
  critical_stale_mask = stale_mask & FAULT_MANAGER_CRITICAL_TASK_MASK;
  if (critical_stale_mask != 0U)
  {
    faults |= FAULT_MANAGER_FAULT_TASK_HEARTBEAT;
  }
  if (startup_grace_complete && (!system_init_complete))
  {
    faults |= FAULT_MANAGER_FAULT_SYSTEM_INIT;
  }

  taskENTER_CRITICAL();
  s_status.stale_task_mask = stale_mask;
  s_status.critical_stale_task_mask = critical_stale_mask;
  if (startup_grace_complete && (!s_startup_reported))
  {
    s_startup_reported = true;
    report_startup = true;
  }
  new_faults = FaultManager_LatchLocked(faults, now_ms);
  taskEXIT_CRITICAL();

  if (report_startup)
  {
    DEBUG_LOG_INFO(
      "Task health armed: seen_mask=0x%02lX stale_mask=0x%02lX system_init=%u",
      (unsigned long)seen_mask,
      (unsigned long)stale_mask,
      system_init_complete ? 1U : 0U);
  }
  if (stale_mask != previous_stale_mask)
  {
    DEBUG_LOG_WARN("Task health changed: stale_mask=0x%02lX critical=0x%02lX",
                   (unsigned long)stale_mask,
                   (unsigned long)critical_stale_mask);
  }
  if ((new_faults & FAULT_MANAGER_FAULT_TASK_HEARTBEAT) != 0U)
  {
    DEBUG_LOG_ERROR(
      "Fault manager latched critical task timeout: task_mask=0x%02lX",
      (unsigned long)critical_stale_mask);
  }
  if ((new_faults & FAULT_MANAGER_FAULT_SYSTEM_INIT) != 0U)
  {
    DEBUG_LOG_ERROR("Fault manager latched system initialization timeout");
  }
}

bool FaultManager_HasCriticalFault(void)
{
  bool has_fault;

  taskENTER_CRITICAL();
  has_fault = s_status.initialized &&
    (s_status.latched_faults != FAULT_MANAGER_FAULT_NONE);
  taskEXIT_CRITICAL();
  return has_fault;
}

bool FaultManager_CanEnableMotion(void)
{
  bool can_enable;

  taskENTER_CRITICAL();
  can_enable = s_status.initialized &&
    s_status.system_init_complete &&
    (s_status.latched_faults == FAULT_MANAGER_FAULT_NONE) &&
    (s_status.critical_stale_task_mask == 0U) &&
    (s_status.current_stalled_wheel_mask == 0U);
  taskEXIT_CRITICAL();
  return can_enable;
}

bool FaultManager_Clear(uint32_t now_ms)
{
  bool cleared = false;
  bool had_latched_fault = false;

  FaultManager_Process(now_ms);
  taskENTER_CRITICAL();
  if (s_status.initialized &&
      s_status.system_init_complete &&
      (s_status.critical_stale_task_mask == 0U) &&
      (s_status.current_stalled_wheel_mask == 0U))
  {
    had_latched_fault = (s_status.latched_faults != FAULT_MANAGER_FAULT_NONE);
    s_status.latched_faults = FAULT_MANAGER_FAULT_NONE;
    s_status.latched_stalled_wheel_mask = 0U;
    s_status.first_fault_ms = 0U;
    cleared = true;
  }
  taskEXIT_CRITICAL();

  if (cleared && had_latched_fault)
  {
    DEBUG_LOG_INFO("Fault manager latch cleared; live health checks are valid");
  }
  return cleared;
}

bool FaultManager_GetStatus(FaultManagerStatus *status)
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
