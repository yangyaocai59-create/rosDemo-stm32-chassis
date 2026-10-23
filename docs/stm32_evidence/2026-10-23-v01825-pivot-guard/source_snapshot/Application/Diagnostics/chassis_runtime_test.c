/**
 * @file chassis_runtime_test.c
 * @brief Debug-only production runtime heartbeat/timeout validation.
 */

#include "chassis_runtime_test.h"

#if MOTOR_OPEN_LOOP_TEST_ENABLED

#include "FreeRTOS.h"
#include "chassis_runtime.h"
#include "debug_log.h"
#include "emergency_stop.h"
#include "task.h"

#define RUNTIME_TEST_ARM_WINDOW_MS       5000U
#define RUNTIME_TEST_COUNTDOWN_MS        2000U
#define RUNTIME_TEST_ACTIVE_TIMEOUT_MS    300U
#define RUNTIME_TEST_DRIVE_MS            1000U
#define RUNTIME_TEST_HEARTBEAT_MS           50U
#define RUNTIME_TEST_TIMEOUT_MARGIN_MS      80U
#define RUNTIME_TEST_OVERALL_MS           5000U
#define RUNTIME_TEST_LINEAR_M_S          0.040f

typedef enum
{
  RUNTIME_TEST_STATE_DISARMED = 0,
  RUNTIME_TEST_STATE_ARMED,
  RUNTIME_TEST_STATE_COUNTDOWN,
  RUNTIME_TEST_STATE_WAIT_ACTIVE,
  RUNTIME_TEST_STATE_DRIVING,
  RUNTIME_TEST_STATE_WAIT_TIMEOUT
} ChassisRuntimeTestState;

static volatile ChassisRuntimeTestState s_state;
static uint32_t s_deadline_ms;
static uint32_t s_overall_deadline_ms;
static uint32_t s_next_heartbeat_ms;

static bool ChassisRuntimeTest_TimeReached(uint32_t now_ms,
                                           uint32_t deadline_ms)
{
  return ((int32_t)(now_ms - deadline_ms) >= 0);
}

static void ChassisRuntimeTest_Abort(const char *reason)
{
  ChassisRuntime_Stop(0U);
  taskENTER_CRITICAL();
  s_state = RUNTIME_TEST_STATE_DISARMED;
  taskEXIT_CRITICAL();
  DEBUG_LOG_ERROR("RUNTIME test aborted: %s; motors disabled", reason);
}

void ChassisRuntimeTest_Init(void)
{
  taskENTER_CRITICAL();
  s_state = RUNTIME_TEST_STATE_DISARMED;
  s_deadline_ms = 0U;
  s_overall_deadline_ms = 0U;
  s_next_heartbeat_ms = 0U;
  taskEXIT_CRITICAL();
}

bool ChassisRuntimeTest_Arm(uint32_t now_ms)
{
  bool armed = false;

  if ((!EmergencyStop_CanEnableMotor()) ||
      (!ChassisRuntime_CanRunDiagnostics()))
  {
    DEBUG_LOG_WARN("RUNTIME test arm rejected: runtime not safely IDLE");
    return false;
  }

  taskENTER_CRITICAL();
  if (s_state == RUNTIME_TEST_STATE_DISARMED)
  {
    s_deadline_ms = now_ms + RUNTIME_TEST_ARM_WINDOW_MS;
    s_state = RUNTIME_TEST_STATE_ARMED;
    armed = true;
  }
  taskEXIT_CRITICAL();

  if (armed)
  {
    DEBUG_LOG_WARN("RUNTIME test armed for 5 s: send 'g' to start");
    DEBUG_LOG_WARN("Test drives forward at 40 mm/s for 1 s, then verifies 250 ms timeout stop");
  }
  return armed;
}

bool ChassisRuntimeTest_Start(uint32_t now_ms)
{
  bool queued = false;

  taskENTER_CRITICAL();
  if ((s_state == RUNTIME_TEST_STATE_ARMED) &&
      (!ChassisRuntimeTest_TimeReached(now_ms, s_deadline_ms)))
  {
    s_deadline_ms = now_ms + RUNTIME_TEST_COUNTDOWN_MS;
    s_overall_deadline_ms = now_ms + RUNTIME_TEST_OVERALL_MS;
    s_state = RUNTIME_TEST_STATE_COUNTDOWN;
    queued = true;
  }
  taskEXIT_CRITICAL();

  if (queued)
  {
    DEBUG_LOG_WARN("RUNTIME forward test starts in 2 s; send 's' to cancel");
  }
  else
  {
    DEBUG_LOG_WARN("RUNTIME start rejected: send 'n', then 'g' within 5 s");
  }
  return queued;
}

void ChassisRuntimeTest_Process(uint32_t now_ms)
{
  ChassisRuntimeStatus runtime;
  ChassisRuntimeTestState state = s_state;

  if (state == RUNTIME_TEST_STATE_DISARMED)
  {
    return;
  }
  if (!ChassisRuntime_GetStatus(&runtime))
  {
    ChassisRuntimeTest_Abort("runtime status unavailable");
    return;
  }

  if ((state == RUNTIME_TEST_STATE_ARMED) &&
      ChassisRuntimeTest_TimeReached(now_ms, s_deadline_ms))
  {
    taskENTER_CRITICAL();
    s_state = RUNTIME_TEST_STATE_DISARMED;
    taskEXIT_CRITICAL();
    DEBUG_LOG_INFO("RUNTIME test arm window expired; motors remain disabled");
    return;
  }
  if ((state != RUNTIME_TEST_STATE_ARMED) &&
      ChassisRuntimeTest_TimeReached(now_ms, s_overall_deadline_ms))
  {
    ChassisRuntimeTest_Abort("overall timeout");
    return;
  }

  switch (state)
  {
    case RUNTIME_TEST_STATE_COUNTDOWN:
      if (ChassisRuntimeTest_TimeReached(now_ms, s_deadline_ms))
      {
        if (!ChassisRuntime_SetVelocity(RUNTIME_TEST_LINEAR_M_S,
                                        0.0f,
                                        now_ms))
        {
          ChassisRuntimeTest_Abort("initial command rejected");
          return;
        }
        s_next_heartbeat_ms = now_ms + RUNTIME_TEST_HEARTBEAT_MS;
        s_deadline_ms = now_ms + RUNTIME_TEST_ACTIVE_TIMEOUT_MS;
        s_state = RUNTIME_TEST_STATE_WAIT_ACTIVE;
      }
      break;

    case RUNTIME_TEST_STATE_WAIT_ACTIVE:
      if (runtime.state == CHASSIS_RUNTIME_STATE_ACTIVE)
      {
        s_deadline_ms = now_ms + RUNTIME_TEST_DRIVE_MS;
        s_next_heartbeat_ms = now_ms;
        s_state = RUNTIME_TEST_STATE_DRIVING;
        DEBUG_LOG_WARN("RUNTIME ACTIVE: forward heartbeat=50 ms duration=1000 ms");
      }
      else if ((runtime.state == CHASSIS_RUNTIME_STATE_FAULT) ||
               (runtime.state == CHASSIS_RUNTIME_STATE_ESTOP))
      {
        ChassisRuntimeTest_Abort("runtime failed to enter ACTIVE");
      }
      else if (ChassisRuntimeTest_TimeReached(now_ms, s_deadline_ms))
      {
        ChassisRuntimeTest_Abort("ACTIVE transition timeout");
      }
      else if (ChassisRuntimeTest_TimeReached(now_ms, s_next_heartbeat_ms))
      {
        (void)ChassisRuntime_SetVelocity(RUNTIME_TEST_LINEAR_M_S,
                                         0.0f,
                                         now_ms);
        s_next_heartbeat_ms = now_ms + RUNTIME_TEST_HEARTBEAT_MS;
      }
      break;

    case RUNTIME_TEST_STATE_DRIVING:
      if (runtime.state != CHASSIS_RUNTIME_STATE_ACTIVE)
      {
        ChassisRuntimeTest_Abort("runtime left ACTIVE before heartbeat ended");
      }
      else if (ChassisRuntimeTest_TimeReached(now_ms, s_deadline_ms))
      {
        s_deadline_ms = now_ms + CHASSIS_RUNTIME_COMMAND_TIMEOUT_MS +
          RUNTIME_TEST_TIMEOUT_MARGIN_MS;
        s_state = RUNTIME_TEST_STATE_WAIT_TIMEOUT;
        DEBUG_LOG_WARN("RUNTIME heartbeat intentionally stopped; expecting timeout in 250 ms");
      }
      else if (ChassisRuntimeTest_TimeReached(now_ms, s_next_heartbeat_ms))
      {
        if (!ChassisRuntime_SetVelocity(RUNTIME_TEST_LINEAR_M_S,
                                        0.0f,
                                        now_ms))
        {
          ChassisRuntimeTest_Abort("heartbeat command rejected");
          return;
        }
        s_next_heartbeat_ms = now_ms + RUNTIME_TEST_HEARTBEAT_MS;
      }
      break;

    case RUNTIME_TEST_STATE_WAIT_TIMEOUT:
      if ((runtime.state == CHASSIS_RUNTIME_STATE_IDLE) &&
          (runtime.last_stop_reason ==
           CHASSIS_RUNTIME_STOP_COMMAND_TIMEOUT))
      {
        taskENTER_CRITICAL();
        s_state = RUNTIME_TEST_STATE_DISARMED;
        taskEXIT_CRITICAL();
        DEBUG_LOG_INFO("RUNTIME test completed: heartbeat timeout returned ACTIVE -> IDLE; motors disabled");
      }
      else if ((runtime.state == CHASSIS_RUNTIME_STATE_FAULT) ||
               (runtime.state == CHASSIS_RUNTIME_STATE_ESTOP))
      {
        ChassisRuntimeTest_Abort("fault while waiting for timeout stop");
      }
      else if (ChassisRuntimeTest_TimeReached(now_ms, s_deadline_ms))
      {
        ChassisRuntimeTest_Abort("250 ms command timeout was not enforced");
      }
      break;

    default:
      break;
  }
}

void ChassisRuntimeTest_SafetyProcess(uint32_t now_ms)
{
  if ((s_state != RUNTIME_TEST_STATE_DISARMED) &&
      ((!EmergencyStop_CanEnableMotor()) ||
       ((s_state != RUNTIME_TEST_STATE_ARMED) &&
        ChassisRuntimeTest_TimeReached(now_ms, s_overall_deadline_ms))))
  {
    ChassisRuntimeTest_Abort(
      EmergencyStop_CanEnableMotor() ? "safety timeout" : "ESTOP active");
  }
}

void ChassisRuntimeTest_Stop(void)
{
  const bool was_active = (s_state != RUNTIME_TEST_STATE_DISARMED);

  ChassisRuntime_Stop(0U);
  taskENTER_CRITICAL();
  s_state = RUNTIME_TEST_STATE_DISARMED;
  taskEXIT_CRITICAL();
  if (was_active)
  {
    DEBUG_LOG_WARN("RUNTIME test cancelled; motors disabled");
  }
}

bool ChassisRuntimeTest_IsArmed(void)
{
  return (s_state == RUNTIME_TEST_STATE_ARMED);
}

bool ChassisRuntimeTest_IsActive(void)
{
  return (s_state != RUNTIME_TEST_STATE_DISARMED);
}

#else

void ChassisRuntimeTest_Init(void)
{
}

bool ChassisRuntimeTest_Arm(uint32_t now_ms)
{
  (void)now_ms;
  return false;
}

bool ChassisRuntimeTest_Start(uint32_t now_ms)
{
  (void)now_ms;
  return false;
}

void ChassisRuntimeTest_Process(uint32_t now_ms)
{
  (void)now_ms;
}

void ChassisRuntimeTest_SafetyProcess(uint32_t now_ms)
{
  (void)now_ms;
}

void ChassisRuntimeTest_Stop(void)
{
}

bool ChassisRuntimeTest_IsArmed(void)
{
  return false;
}

bool ChassisRuntimeTest_IsActive(void)
{
  return false;
}

#endif /* MOTOR_OPEN_LOOP_TEST_ENABLED */
