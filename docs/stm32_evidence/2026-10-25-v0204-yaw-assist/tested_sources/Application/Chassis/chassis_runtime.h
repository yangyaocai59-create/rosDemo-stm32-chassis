/**
 * @file chassis_runtime.h
 * @brief Production chassis command ownership and safety state machine.
 */

#ifndef APPLICATION_CHASSIS_CHASSIS_RUNTIME_H_
#define APPLICATION_CHASSIS_CHASSIS_RUNTIME_H_

#include <stdbool.h>
#include <stdint.h>

#include "chassis_controller.h"
#if CHASSIS_CONTROL_REVISION >= 203
#include "yaw_rate_control_core.h"
#endif

#ifdef __cplusplus
extern "C" {
#endif

#if CHASSIS_CONTROL_REVISION >= 203
bool ChassisRuntime_GetYawControl(YawRateControlState *state);
#endif

#define CHASSIS_RUNTIME_COMMAND_TIMEOUT_MS       250U
#define CHASSIS_RUNTIME_MAX_LINEAR_M_S           0.20f
#define CHASSIS_RUNTIME_MAX_ANGULAR_RAD_S         1.50f

typedef enum
{
  CHASSIS_RUNTIME_STATE_INIT = 0,
  CHASSIS_RUNTIME_STATE_IDLE,
  CHASSIS_RUNTIME_STATE_ACTIVE,
  CHASSIS_RUNTIME_STATE_FAULT,
  CHASSIS_RUNTIME_STATE_ESTOP
} ChassisRuntimeState;

typedef enum
{
  CHASSIS_RUNTIME_FAULT_NONE = 0,
  CHASSIS_RUNTIME_FAULT_INVALID_COMMAND,
  CHASSIS_RUNTIME_FAULT_DIAGNOSTIC_CONFLICT,
  CHASSIS_RUNTIME_FAULT_CONTROLLER_START,
  CHASSIS_RUNTIME_FAULT_CONTROLLER_RUNTIME,
  CHASSIS_RUNTIME_FAULT_HEALTH_MANAGER
} ChassisRuntimeFault;

typedef enum
{
  CHASSIS_RUNTIME_STOP_NONE = 0,
  CHASSIS_RUNTIME_STOP_ZERO_COMMAND,
  CHASSIS_RUNTIME_STOP_EXPLICIT,
  CHASSIS_RUNTIME_STOP_COMMAND_TIMEOUT,
  CHASSIS_RUNTIME_STOP_DIAGNOSTIC,
  CHASSIS_RUNTIME_STOP_ESTOP,
  CHASSIS_RUNTIME_STOP_FAULT,
  CHASSIS_RUNTIME_STOP_TRANSPORT_LOST
} ChassisRuntimeStopReason;

typedef struct
{
  ChassisRuntimeState state;
  ChassisRuntimeFault fault;
  ChassisRuntimeStopReason last_stop_reason;
  ChassisControllerFault controller_fault;
  ChassisVelocityCommand requested_command;
  /** Internal shaped target; existing USB STATUS still reports requested_command. */
  ChassisVelocityCommand applied_command;
  bool command_valid;
  bool diagnostics_active;
  uint32_t last_command_ms;
  uint32_t command_sequence;
  uint32_t accepted_commands;
  uint32_t rejected_commands;
} ChassisRuntimeStatus;

/* Bounded request/start history, independently retained from wheel samples. */
#define CHASSIS_RUNTIME_EVENT_CAPACITY 16U
typedef enum {
  CHASSIS_EVENT_START = 1, CHASSIS_EVENT_STOP_REQUEST,
  CHASSIS_EVENT_TRANSPORT_LOSS, CHASSIS_EVENT_HEALTH_FAULT,
  CHASSIS_EVENT_ZERO_COMMAND, CHASSIS_EVENT_COMMAND_TIMEOUT
} ChassisRuntimeEventKind;
typedef struct {
  uint32_t sequence, time_ms, generation, command_sequence;
  ChassisRuntimeEventKind kind;
  ChassisRuntimeState state;
} ChassisRuntimeEvent;
/* Bounded read-only history, independent of the 20ms wheel trace.
 * Returns oldest-first entries; overwritten count is reported explicitly. */
uint32_t ChassisRuntime_GetEvents(ChassisRuntimeEvent events[CHASSIS_RUNTIME_EVENT_CAPACITY],
                                 uint32_t *overwritten);

/** Initialize in motor-disabled IDLE, or ESTOP when the latch is active. */
bool ChassisRuntime_Init(uint32_t now_ms);

/**
 * Submit the latest velocity command and source timestamp.
 * Commands are rejected outside configured bounds or while diagnostics/faults
 * own the motor path. A non-zero command must be refreshed within 250 ms.
 */
bool ChassisRuntime_SetVelocity(float linear_x_m_s,
                                float angular_z_rad_s,
                                uint32_t now_ms);

/** Immediately remove motor drive and return to IDLE when not E-stopped. */
void ChassisRuntime_Stop(uint32_t now_ms);

/** Immediately stop and retain a distinct communication-link-loss reason. */
void ChassisRuntime_StopForTransportLoss(uint32_t now_ms);

/** Latch a central health fault and immediately remove all motor drive. */
void ChassisRuntime_LatchHealthFault(uint32_t now_ms);

/** Clear a software runtime fault only while stopped and E-stop is inactive. */
bool ChassisRuntime_ClearFault(uint32_t now_ms);

/** Run command ownership and closed-loop processing from motorTask. */
void ChassisRuntime_Process(uint32_t now_ms, bool diagnostics_active);

/** Enforce E-stop and source-command timeout from safetyTask. */
void ChassisRuntime_SafetyProcess(uint32_t now_ms);

bool ChassisRuntime_IsActive(void);
bool ChassisRuntime_CanRunDiagnostics(void);
bool ChassisRuntime_GetStatus(ChassisRuntimeStatus *status);
const char *ChassisRuntime_GetStateName(ChassisRuntimeState state);
const char *ChassisRuntime_GetFaultName(ChassisRuntimeFault fault);
const char *ChassisRuntime_GetStopReasonName(ChassisRuntimeStopReason reason);

#ifdef __cplusplus
}
#endif

#endif /* APPLICATION_CHASSIS_CHASSIS_RUNTIME_H_ */
