/**
 * @file chassis_runtime.c
 * @brief Production chassis command ownership and safety state machine.
 */

#include "chassis_runtime.h"

#include <stddef.h>

#include "FreeRTOS.h"
#include "debug_log.h"
#include "emergency_stop.h"
#include "fault_manager.h"
#include "chassis_motion_profile.h"
#include "task.h"
#if CHASSIS_CONTROL_REVISION >= 203
#include "chassis_yaw_feedback.h"
static YawRateControlState s_yaw_control;
bool ChassisRuntime_GetYawControl(YawRateControlState *state)
{
  if (!state) return false;
  taskENTER_CRITICAL(); *state=s_yaw_control; taskEXIT_CRITICAL(); return true;
}
#endif

#define CHASSIS_RUNTIME_ZERO_LINEAR_M_S       0.0005f
#define CHASSIS_RUNTIME_ZERO_ANGULAR_RAD_S    0.0005f

static ChassisRuntimeStatus s_status;
static uint32_t s_profile_time_ms;
static uint32_t s_motion_generation;

static ChassisRuntimeEvent s_events[CHASSIS_RUNTIME_EVENT_CAPACITY];
static uint32_t s_event_head, s_event_count, s_event_sequence, s_event_overwritten;

/* Only bounded RAM writes under the lock; no logging or output ownership. */
static void ChassisRuntime_RecordEvent(ChassisRuntimeEventKind kind, uint32_t now_ms)
{
  taskENTER_CRITICAL();
  s_events[s_event_head] = (ChassisRuntimeEvent){
    ++s_event_sequence, now_ms, s_motion_generation, s_status.command_sequence,
    kind, s_status.state};
  s_event_head = (s_event_head + 1U) % CHASSIS_RUNTIME_EVENT_CAPACITY;
  if (s_event_count < CHASSIS_RUNTIME_EVENT_CAPACITY) ++s_event_count;
  else ++s_event_overwritten;
  taskEXIT_CRITICAL();
}

uint32_t ChassisRuntime_GetEvents(ChassisRuntimeEvent events[CHASSIS_RUNTIME_EVENT_CAPACITY],
                                 uint32_t *overwritten)
{
  if (events == NULL || overwritten == NULL) return 0U;
  taskENTER_CRITICAL();
  const uint32_t count = s_event_count;
  for (uint32_t i=0U; i<count; ++i)
    events[i] = s_events[(s_event_head + CHASSIS_RUNTIME_EVENT_CAPACITY - count + i)
                         % CHASSIS_RUNTIME_EVENT_CAPACITY];
  *overwritten = s_event_overwritten;
  taskEXIT_CRITICAL();
  return count;
}

static void ChassisRuntime_ResetProfileLocked(void)
{
#if CHASSIS_CONTROL_REVISION >= 203
  YawRateControl_Reset(&s_yaw_control);
#endif
  s_status.applied_command.linear_x_m_s = 0.0f;
  s_status.applied_command.angular_z_rad_s = 0.0f;
  s_profile_time_ms = 0U;
}

static float ChassisRuntime_Abs(float value)
{
  return (value < 0.0f) ? -value : value;
}

static int32_t ChassisRuntime_Round(float value)
{
  return (int32_t)((value >= 0.0f) ? (value + 0.5f) : (value - 0.5f));
}

static bool ChassisRuntime_TimeReached(uint32_t now_ms,
                                       uint32_t deadline_ms)
{
  return ((int32_t)(now_ms - deadline_ms) >= 0);
}

static bool ChassisRuntime_IsCommandFiniteAndBounded(
  const ChassisVelocityCommand *command)
{
  return (command != NULL) &&
         (command->linear_x_m_s == command->linear_x_m_s) &&
         (command->angular_z_rad_s == command->angular_z_rad_s) &&
         (ChassisRuntime_Abs(command->linear_x_m_s) <=
          CHASSIS_RUNTIME_MAX_LINEAR_M_S) &&
         (ChassisRuntime_Abs(command->angular_z_rad_s) <=
          CHASSIS_RUNTIME_MAX_ANGULAR_RAD_S);
}

static bool ChassisRuntime_IsZeroCommand(
  const ChassisVelocityCommand *command)
{
  return (ChassisRuntime_Abs(command->linear_x_m_s) <
          CHASSIS_RUNTIME_ZERO_LINEAR_M_S) &&
         (ChassisRuntime_Abs(command->angular_z_rad_s) <
          CHASSIS_RUNTIME_ZERO_ANGULAR_RAD_S);
}

static void ChassisRuntime_SetFaultInternal(ChassisRuntimeFault fault,
  ChassisControllerFault controller_fault, const uint32_t *expected_generation)
{
  bool log_fault = false;

  taskENTER_CRITICAL();
  if ((expected_generation != NULL) &&
      (*expected_generation != s_motion_generation))
  { taskEXIT_CRITICAL(); return; }
  ChassisController_Stop();
  ++s_motion_generation;
  if ((s_status.state != CHASSIS_RUNTIME_STATE_FAULT) ||
      (s_status.fault != fault) ||
      (s_status.controller_fault != controller_fault))
  {
    log_fault = true;
  }
  s_status.state = CHASSIS_RUNTIME_STATE_FAULT;
  s_status.fault = fault;
  s_status.controller_fault = controller_fault;
  s_status.last_stop_reason = CHASSIS_RUNTIME_STOP_FAULT;
  s_status.command_valid = false;
  ChassisRuntime_ResetProfileLocked();
  s_status.requested_command.linear_x_m_s = 0.0f;
  s_status.requested_command.angular_z_rad_s = 0.0f;
  taskEXIT_CRITICAL();

  if (log_fault)
  {
    DEBUG_LOG_ERROR("Chassis runtime FAULT: %s, controller=%s; motors disabled",
                    ChassisRuntime_GetFaultName(fault),
                    ChassisController_GetFaultName(controller_fault));
  }
}

static void ChassisRuntime_SetFault(ChassisRuntimeFault fault,
                                    ChassisControllerFault controller_fault)
{
  ChassisRuntime_SetFaultInternal(fault, controller_fault, NULL);
}

static void ChassisRuntime_SetCurrentFault(uint32_t generation,
  ChassisRuntimeFault fault, ChassisControllerFault controller_fault)
{
  ChassisRuntime_SetFaultInternal(fault, controller_fault, &generation);
}

static void ChassisRuntime_SetEstop(void)
{
  bool log_estop = false;

  taskENTER_CRITICAL();
  ChassisController_Stop();
  ++s_motion_generation;
  if (s_status.state != CHASSIS_RUNTIME_STATE_ESTOP)
  {
    log_estop = true;
  }
  s_status.state = CHASSIS_RUNTIME_STATE_ESTOP;
  s_status.fault = CHASSIS_RUNTIME_FAULT_NONE;
  s_status.controller_fault = CHASSIS_CONTROLLER_FAULT_ESTOP;
  s_status.last_stop_reason = CHASSIS_RUNTIME_STOP_ESTOP;
  s_status.command_valid = false;
  ChassisRuntime_ResetProfileLocked();
  s_status.requested_command.linear_x_m_s = 0.0f;
  s_status.requested_command.angular_z_rad_s = 0.0f;
  taskEXIT_CRITICAL();

  if (log_estop)
  {
    DEBUG_LOG_ERROR("Chassis runtime ESTOP: motors disabled until MCU reset");
  }
}

static void ChassisRuntime_SetIdleInternal(ChassisRuntimeStopReason reason,
  bool log_transition, const uint32_t *expected_generation)
{
  bool was_active;

  taskENTER_CRITICAL();
  if ((expected_generation != NULL) &&
      (*expected_generation != s_motion_generation))
  { taskEXIT_CRITICAL(); return; }
  ChassisController_Stop();
  ++s_motion_generation;
  was_active = (s_status.state == CHASSIS_RUNTIME_STATE_ACTIVE);
  s_status.state = CHASSIS_RUNTIME_STATE_IDLE;
  s_status.fault = CHASSIS_RUNTIME_FAULT_NONE;
  s_status.controller_fault = CHASSIS_CONTROLLER_FAULT_NONE;
  s_status.last_stop_reason = reason;
  s_status.command_valid = false;
  ChassisRuntime_ResetProfileLocked();
  s_status.requested_command.linear_x_m_s = 0.0f;
  s_status.requested_command.angular_z_rad_s = 0.0f;
  taskEXIT_CRITICAL();

  if (log_transition && was_active)
  {
    if (reason == CHASSIS_RUNTIME_STOP_COMMAND_TIMEOUT)
    {
      DEBUG_LOG_WARN("Chassis runtime command timeout after %lu ms; motors disabled",
                     (unsigned long)CHASSIS_RUNTIME_COMMAND_TIMEOUT_MS);
    }
    else
    {
      DEBUG_LOG_INFO("Chassis runtime ACTIVE -> IDLE: %s; motors disabled",
                     ChassisRuntime_GetStopReasonName(reason));
    }
  }
}

static void ChassisRuntime_SetIdle(ChassisRuntimeStopReason reason,
                                   bool log_transition)
{
  ChassisRuntime_SetIdleInternal(reason, log_transition, NULL);
}

static void ChassisRuntime_SetCurrentIdle(uint32_t generation,
                                         ChassisRuntimeStopReason reason)
{
  ChassisRuntime_SetIdleInternal(reason, true, &generation);
}

bool ChassisRuntime_Init(uint32_t now_ms)
{
  (void)now_ms;
#if CHASSIS_CONTROL_REVISION >= 203
  ChassisYawFeedback_Init();
#endif
  ChassisController_Stop();

  taskENTER_CRITICAL();
  s_status.state = CHASSIS_RUNTIME_STATE_INIT;
  s_event_head = s_event_count = s_event_sequence = s_event_overwritten = 0U;
  ++s_motion_generation;
  ChassisRuntime_ResetProfileLocked();
  s_status.fault = CHASSIS_RUNTIME_FAULT_NONE;
  s_status.last_stop_reason = CHASSIS_RUNTIME_STOP_NONE;
  s_status.controller_fault = CHASSIS_CONTROLLER_FAULT_NONE;
  s_status.requested_command.linear_x_m_s = 0.0f;
  s_status.requested_command.angular_z_rad_s = 0.0f;
  s_status.command_valid = false;
  s_status.diagnostics_active = false;
  s_status.last_command_ms = 0U;
  s_status.command_sequence = 0U;
  s_status.accepted_commands = 0U;
  s_status.rejected_commands = 0U;
  s_status.state = EmergencyStop_CanEnableMotor() ?
    CHASSIS_RUNTIME_STATE_IDLE : CHASSIS_RUNTIME_STATE_ESTOP;
  if (s_status.state == CHASSIS_RUNTIME_STATE_ESTOP)
  {
    s_status.controller_fault = CHASSIS_CONTROLLER_FAULT_ESTOP;
    s_status.last_stop_reason = CHASSIS_RUNTIME_STOP_ESTOP;
  }
  taskEXIT_CRITICAL();

  DEBUG_LOG_INFO(
    "Chassis runtime ready: state=%s limits=%ldmm/s,%ldmrad/s timeout=%lums",
    ChassisRuntime_GetStateName(s_status.state),
    (long)ChassisRuntime_Round(CHASSIS_RUNTIME_MAX_LINEAR_M_S * 1000.0f),
    (long)ChassisRuntime_Round(CHASSIS_RUNTIME_MAX_ANGULAR_RAD_S * 1000.0f),
    (unsigned long)CHASSIS_RUNTIME_COMMAND_TIMEOUT_MS);
  return (s_status.state == CHASSIS_RUNTIME_STATE_IDLE);
}

bool ChassisRuntime_SetVelocity(float linear_x_m_s,
                                float angular_z_rad_s,
                                uint32_t now_ms)
{
  const ChassisVelocityCommand command = {
    linear_x_m_s,
    angular_z_rad_s
  };
  bool accepted = false;
  bool invalid = false;
  const bool health_allows_motion = FaultManager_CanEnableMotion();

  taskENTER_CRITICAL();
  if ((s_status.state == CHASSIS_RUNTIME_STATE_IDLE) ||
      (s_status.state == CHASSIS_RUNTIME_STATE_ACTIVE))
  {
    if ((!health_allows_motion) || s_status.diagnostics_active)
    {
      ++s_status.rejected_commands;
    }
    else if (!ChassisRuntime_IsCommandFiniteAndBounded(&command))
    {
      ++s_status.rejected_commands;
      invalid = true;
    }
    else
    {
      s_status.requested_command = command;
      s_status.last_command_ms = now_ms;
      ++s_status.command_sequence;
      ++s_status.accepted_commands;
      s_status.command_valid = true;
      accepted = true;
    }
  }
  else
  {
    ++s_status.rejected_commands;
  }
  taskEXIT_CRITICAL();

  if (invalid)
  {
    ChassisRuntime_SetFault(CHASSIS_RUNTIME_FAULT_INVALID_COMMAND,
                            CHASSIS_CONTROLLER_FAULT_INVALID_COMMAND);
  }
  return accepted;
}

void ChassisRuntime_Stop(uint32_t now_ms)
{
  ChassisRuntime_RecordEvent(CHASSIS_EVENT_STOP_REQUEST, now_ms);
  (void)now_ms;
  if (EmergencyStop_IsLatched())
  {
    ChassisRuntime_SetEstop();
  }
  else
  {
    ChassisRuntime_SetIdle(CHASSIS_RUNTIME_STOP_EXPLICIT, true);
  }
}

void ChassisRuntime_StopForTransportLoss(uint32_t now_ms)
{
  ChassisRuntime_RecordEvent(CHASSIS_EVENT_TRANSPORT_LOSS, now_ms);
  (void)now_ms;
  if (EmergencyStop_IsLatched())
  {
    ChassisRuntime_SetEstop();
  }
  else
  {
    ChassisRuntime_SetIdle(CHASSIS_RUNTIME_STOP_TRANSPORT_LOST, true);
  }
}

void ChassisRuntime_LatchHealthFault(uint32_t now_ms)
{
#if CHASSIS_CONTROL_REVISION >= 203
  /* Keep the first fault event instead of flooding the ring every 10 ms. */
  ChassisControllerSafetyState controller={0};
  taskENTER_CRITICAL();
  const bool already_latched=s_status.state==CHASSIS_RUNTIME_STATE_FAULT &&
    s_status.fault==CHASSIS_RUNTIME_FAULT_HEALTH_MANAGER &&
    ChassisController_GetSafetyState(&controller) && !controller.active;
  taskEXIT_CRITICAL();
  if (already_latched) return;
#endif
  ChassisRuntime_RecordEvent(CHASSIS_EVENT_HEALTH_FAULT, now_ms);
  (void)now_ms;
  if (EmergencyStop_IsLatched())
  {
    ChassisRuntime_SetEstop();
  }
  else
  {
    ChassisRuntime_SetFault(CHASSIS_RUNTIME_FAULT_HEALTH_MANAGER,
                            CHASSIS_CONTROLLER_FAULT_NONE);
  }
}

bool ChassisRuntime_ClearFault(uint32_t now_ms)
{
  bool cleared = false;

  if (!EmergencyStop_CanEnableMotor())
  {
    return false;
  }

  taskENTER_CRITICAL();
  const bool can_attempt_clear =
    (s_status.state == CHASSIS_RUNTIME_STATE_FAULT) &&
    (!s_status.diagnostics_active) &&
    (!ChassisController_IsActive());
  taskEXIT_CRITICAL();
  if ((!can_attempt_clear) || (!FaultManager_Clear(now_ms)))
  {
    return false;
  }

  taskENTER_CRITICAL();
  if ((s_status.state == CHASSIS_RUNTIME_STATE_FAULT) &&
      (!s_status.diagnostics_active) &&
      (!ChassisController_IsActive()))
  {
    s_status.state = CHASSIS_RUNTIME_STATE_IDLE;
    s_status.fault = CHASSIS_RUNTIME_FAULT_NONE;
    s_status.controller_fault = CHASSIS_CONTROLLER_FAULT_NONE;
    s_status.last_stop_reason = CHASSIS_RUNTIME_STOP_NONE;
    s_status.command_valid = false;
    cleared = true;
  }
  taskEXIT_CRITICAL();

  if (cleared)
  {
    DEBUG_LOG_INFO("Chassis runtime FAULT -> IDLE: software fault cleared");
  }
  return cleared;
}

void ChassisRuntime_Process(uint32_t now_ms, bool diagnostics_active)
{
#if CHASSIS_CONTROL_REVISION >= 203
  YawRateControlState yaw_control;
#endif
  ChassisRuntimeState state;
  ChassisVelocityCommand command;
  ChassisVelocityCommand applied;
  ChassisVelocityCommand shaped;
  uint32_t command_sequence;
  uint32_t generation;
  uint32_t profile_time;
  uint32_t last_command_ms;
  bool command_valid;

  taskENTER_CRITICAL();
  s_status.diagnostics_active = diagnostics_active;
  state = s_status.state;
  if (diagnostics_active && (state != CHASSIS_RUNTIME_STATE_ACTIVE))
  {
    s_status.command_valid = false;
    s_status.last_stop_reason = CHASSIS_RUNTIME_STOP_DIAGNOSTIC;
  }
  command = s_status.requested_command;
  applied = s_status.applied_command;
  profile_time = s_profile_time_ms;
  command_sequence = s_status.command_sequence;
  generation = s_motion_generation;
  last_command_ms = s_status.last_command_ms;
  command_valid = s_status.command_valid;
#if CHASSIS_CONTROL_REVISION >= 203
  yaw_control=s_yaw_control;
#endif
  taskEXIT_CRITICAL();

  if (!EmergencyStop_CanEnableMotor())
  {
    ChassisRuntime_SetEstop();
    return;
  }
  if (diagnostics_active)
  {
    if (state == CHASSIS_RUNTIME_STATE_ACTIVE)
    {
      ChassisRuntime_SetCurrentFault(generation, CHASSIS_RUNTIME_FAULT_DIAGNOSTIC_CONFLICT,
                              CHASSIS_CONTROLLER_FAULT_NONE);
    }
    return;
  }
  if ((state == CHASSIS_RUNTIME_STATE_INIT) ||
      (state == CHASSIS_RUNTIME_STATE_FAULT) ||
      (state == CHASSIS_RUNTIME_STATE_ESTOP))
  {
    return;
  }
  if (!command_valid)
  {
    if (state == CHASSIS_RUNTIME_STATE_ACTIVE)
    {
      ChassisRuntime_SetCurrentIdle(generation, CHASSIS_RUNTIME_STOP_EXPLICIT);
    }
    return;
  }
  if (ChassisRuntime_TimeReached(
        now_ms,
        last_command_ms + CHASSIS_RUNTIME_COMMAND_TIMEOUT_MS))
  {
    ChassisRuntime_RecordEvent(CHASSIS_EVENT_COMMAND_TIMEOUT, now_ms);
    ChassisRuntime_SetCurrentIdle(generation, CHASSIS_RUNTIME_STOP_COMMAND_TIMEOUT);
    return;
  }
  if (ChassisRuntime_IsZeroCommand(&command))
  {
    ChassisRuntime_RecordEvent(CHASSIS_EVENT_ZERO_COMMAND, now_ms);
    ChassisRuntime_SetCurrentIdle(generation, CHASSIS_RUNTIME_STOP_ZERO_COMMAND);
    return;
  }

  if (state == CHASSIS_RUNTIME_STATE_IDLE)
  {
    const ChassisVelocityCommand zero = {0.0f, 0.0f};
    (void)ChassisMotionProfile_Step(&zero, &command, 10U, &shaped);
    bool attempted = false;
    bool started = false;
    /* Serialize the start with STOP/health shutdown. No UART wait occurs in
     * the controller. A stale snapshot must never restart a stopped chassis. */
    taskENTER_CRITICAL();
    if ((s_status.state == CHASSIS_RUNTIME_STATE_IDLE) &&
        s_status.command_valid &&
        (s_motion_generation == generation) &&
        (s_status.command_sequence == command_sequence))
    {
      attempted = true;
      started = ChassisController_Start(&shaped, now_ms);
      if (started)
      {
        s_status.state = CHASSIS_RUNTIME_STATE_ACTIVE;
        ChassisRuntime_RecordEvent(CHASSIS_EVENT_START, now_ms);
        s_status.fault = CHASSIS_RUNTIME_FAULT_NONE;
        s_status.controller_fault = CHASSIS_CONTROLLER_FAULT_NONE;
        s_status.last_stop_reason = CHASSIS_RUNTIME_STOP_NONE;
        s_status.applied_command = shaped;
        s_profile_time_ms = now_ms;
      }
    }
    taskEXIT_CRITICAL();
    if (!started)
    {
      if (!attempted) { return; }
      ChassisControllerStatus controller = { 0 };
      (void)ChassisController_GetStatus(&controller);
      ChassisRuntime_SetCurrentFault(generation, CHASSIS_RUNTIME_FAULT_CONTROLLER_START,
                              controller.fault);
      return;
    }

    DEBUG_LOG_INFO("Chassis runtime IDLE -> ACTIVE: vx=%ldmm/s wz=%ldmrad/s",
                   (long)ChassisRuntime_Round(command.linear_x_m_s * 1000.0f),
                   (long)ChassisRuntime_Round(command.angular_z_rad_s *
                                              1000.0f));
    return;
  }

  if (state == CHASSIS_RUNTIME_STATE_ACTIVE)
  {
    ChassisControllerStatus controller = { 0 };
    const uint32_t elapsed_ms = now_ms - profile_time;
    if (elapsed_ms == 0U) { return; }
    if (!ChassisMotionProfile_Step(&applied, &command, elapsed_ms, &shaped))
    {
      ChassisRuntime_SetCurrentFault(generation, CHASSIS_RUNTIME_FAULT_CONTROLLER_RUNTIME,
                              CHASSIS_CONTROLLER_FAULT_CONTROL_TIMING);
      return;
    }
    bool updated = false;
#if CHASSIS_CONTROL_REVISION >= 203
    ChassisYawFeedback yaw_feedback={0};
    (void)ChassisYawFeedback_Get(&yaw_feedback);
    (void)ChassisController_GetStatus(&controller);
    bool wheel_authority=controller.limited_mask==0U;
    for (uint32_t w=0U;w<MOTOR_DRIVER_WHEEL_COUNT;++w) {
      const float target=controller.targets.counts_per_second[w];
      const float measured=controller.measured_counts_per_second[w];
      if (!isfinite(target) || !isfinite(measured) ||
          fabsf(target-measured)>fmaxf(75.0f,.25f*fabsf(target))) wheel_authority=false;
    }
    const bool yaw_valid=yaw_feedback.ready && yaw_feedback.valid &&
      (uint32_t)(now_ms-yaw_feedback.sample_time_ms)<=40U;
    ChassisVelocityCommand assisted=shaped;
    assisted.angular_z_rad_s+=YawRateControl_Step(&yaw_control,
      shaped.linear_x_m_s,shaped.angular_z_rad_s,yaw_feedback.rate_rad_s,
      yaw_valid,!wheel_authority,elapsed_ms);
#endif
    taskENTER_CRITICAL();
    if ((s_status.state == CHASSIS_RUNTIME_STATE_ACTIVE) &&
        s_status.command_valid &&
        (s_motion_generation == generation) &&
        (s_status.command_sequence == command_sequence))
    {
#if CHASSIS_CONTROL_REVISION >= 203
      updated = ChassisController_SetCommand(&assisted, now_ms);
#else
      updated = ChassisController_SetCommand(&shaped, now_ms);
#endif
      if (updated)
      {
#if CHASSIS_CONTROL_REVISION >= 203
        s_yaw_control=yaw_control;
#endif
        /* Keep the nominal profile independent of the feedback correction. */
        s_status.applied_command = shaped;
        s_profile_time_ms = now_ms;
      }
    }
    taskEXIT_CRITICAL();
    if (!updated)
    {
      /* An explicit/safety stop invalidated this iteration. */
      if (!ChassisRuntime_IsActive()) { return; }
#if CHASSIS_CONTROL_V2
      /* A newer velocity command is normal supersession, not a controller
       * failure. It will be shaped from the last committed state next tick. */
      taskENTER_CRITICAL();
      const bool superseded=(s_motion_generation!=generation) ||
                            (s_status.command_sequence!=command_sequence);
      taskEXIT_CRITICAL();
      if (superseded) { return; }
#endif
    }

    if ((!updated) ||
        (!ChassisController_GetStatus(&controller)) ||
        (!controller.active) ||
        (controller.fault != CHASSIS_CONTROLLER_FAULT_NONE))
    {
      ChassisRuntime_SetCurrentFault(generation, CHASSIS_RUNTIME_FAULT_CONTROLLER_RUNTIME,
                              controller.fault);
      return;
    }
    ChassisController_Process(now_ms);
    if ((!ChassisController_GetStatus(&controller)) ||
        (!controller.active) ||
        (controller.fault != CHASSIS_CONTROLLER_FAULT_NONE))
    {
      ChassisRuntime_SetCurrentFault(generation, CHASSIS_RUNTIME_FAULT_CONTROLLER_RUNTIME,
                              controller.fault);
    }
  }
}

void ChassisRuntime_SafetyProcess(uint32_t now_ms)
{
  /* Only these ownership/timeout fields are needed. Do not copy two full
   * command/PI/telemetry snapshots into this nested safety call's stack. */
  ChassisControllerSafetyState controller = { 0 };

  taskENTER_CRITICAL();
  const ChassisRuntimeState state = s_status.state;
  const bool command_valid = s_status.command_valid;
  const uint32_t last_command_ms = s_status.last_command_ms;
  const uint32_t generation = s_motion_generation;
  taskEXIT_CRITICAL();
  if (!EmergencyStop_CanEnableMotor())
  {
    ChassisRuntime_SetEstop();
    return;
  }
  if (state != CHASSIS_RUNTIME_STATE_ACTIVE)
  {
    return;
  }
  if ((!command_valid) ||
      ChassisRuntime_TimeReached(
        now_ms,
        last_command_ms + CHASSIS_RUNTIME_COMMAND_TIMEOUT_MS))
  {
    ChassisRuntime_RecordEvent(CHASSIS_EVENT_COMMAND_TIMEOUT, now_ms);
    ChassisRuntime_SetCurrentIdle(generation, CHASSIS_RUNTIME_STOP_COMMAND_TIMEOUT);
    return;
  }
  if ((!ChassisController_GetSafetyState(&controller)) ||
      (!controller.active) ||
      (controller.fault != CHASSIS_CONTROLLER_FAULT_NONE))
  {
    ChassisRuntime_SetCurrentFault(generation, CHASSIS_RUNTIME_FAULT_CONTROLLER_RUNTIME,
                            controller.fault);
  }
}

bool ChassisRuntime_IsActive(void)
{
  bool active;

  taskENTER_CRITICAL();
  active = (s_status.state == CHASSIS_RUNTIME_STATE_ACTIVE);
  taskEXIT_CRITICAL();
  return active;
}

bool ChassisRuntime_CanRunDiagnostics(void)
{
  ChassisRuntimeState state;

  taskENTER_CRITICAL();
  state = s_status.state;
  taskEXIT_CRITICAL();
  return (state == CHASSIS_RUNTIME_STATE_IDLE) &&
         (!ChassisController_IsActive()) &&
         EmergencyStop_CanEnableMotor();
}

bool ChassisRuntime_GetStatus(ChassisRuntimeStatus *status)
{
  if (status == NULL)
  {
    return false;
  }
  taskENTER_CRITICAL();
  *status = s_status;
  taskEXIT_CRITICAL();
  return true;
}

const char *ChassisRuntime_GetStateName(ChassisRuntimeState state)
{
  switch (state)
  {
    case CHASSIS_RUNTIME_STATE_INIT:
      return "INIT";
    case CHASSIS_RUNTIME_STATE_IDLE:
      return "IDLE";
    case CHASSIS_RUNTIME_STATE_ACTIVE:
      return "ACTIVE";
    case CHASSIS_RUNTIME_STATE_FAULT:
      return "FAULT";
    case CHASSIS_RUNTIME_STATE_ESTOP:
      return "ESTOP";
    default:
      return "UNKNOWN";
  }
}

const char *ChassisRuntime_GetFaultName(ChassisRuntimeFault fault)
{
  switch (fault)
  {
    case CHASSIS_RUNTIME_FAULT_NONE:
      return "none";
    case CHASSIS_RUNTIME_FAULT_INVALID_COMMAND:
      return "invalid velocity command";
    case CHASSIS_RUNTIME_FAULT_DIAGNOSTIC_CONFLICT:
      return "diagnostic ownership conflict";
    case CHASSIS_RUNTIME_FAULT_CONTROLLER_START:
      return "controller start failure";
    case CHASSIS_RUNTIME_FAULT_CONTROLLER_RUNTIME:
      return "controller runtime failure";
    case CHASSIS_RUNTIME_FAULT_HEALTH_MANAGER:
      return "health manager fault";
    default:
      return "unknown";
  }
}

const char *ChassisRuntime_GetStopReasonName(ChassisRuntimeStopReason reason)
{
  switch (reason)
  {
    case CHASSIS_RUNTIME_STOP_NONE:
      return "none";
    case CHASSIS_RUNTIME_STOP_ZERO_COMMAND:
      return "zero command";
    case CHASSIS_RUNTIME_STOP_EXPLICIT:
      return "explicit stop";
    case CHASSIS_RUNTIME_STOP_COMMAND_TIMEOUT:
      return "command timeout";
    case CHASSIS_RUNTIME_STOP_DIAGNOSTIC:
      return "diagnostic ownership";
    case CHASSIS_RUNTIME_STOP_ESTOP:
      return "ESTOP";
    case CHASSIS_RUNTIME_STOP_FAULT:
      return "fault";
    case CHASSIS_RUNTIME_STOP_TRANSPORT_LOST:
      return "transport lost";
    default:
      return "unknown";
  }
}
