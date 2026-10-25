/**
 * @file chassis_controller.h
 * @brief Chassis velocity command to four closed-loop wheel outputs.
 */

#ifndef APPLICATION_CHASSIS_CHASSIS_CONTROLLER_H_
#define APPLICATION_CHASSIS_CHASSIS_CONTROLLER_H_

#include <stdbool.h>
#include <stdint.h>

#include "chassis_kinematics.h"
#include "wheel_speed_controller.h"
#if CHASSIS_CONTROL_V2
#include "control_run_metrics.h"
#endif

#ifdef __cplusplus
extern "C" {
#endif

#if CHASSIS_CONTROL_V2
bool ChassisController_GetRunMetrics(ChassisRunMetrics *metrics);
#endif

typedef enum
{
  CHASSIS_CONTROLLER_FAULT_NONE = 0,
  CHASSIS_CONTROLLER_FAULT_INVALID_COMMAND,
  CHASSIS_CONTROLLER_FAULT_ESTOP,
  CHASSIS_CONTROLLER_FAULT_SPEED_SNAPSHOT,
  CHASSIS_CONTROLLER_FAULT_CONTROL_UPDATE,
  CHASSIS_CONTROLLER_FAULT_OUTPUT,
  CHASSIS_CONTROLLER_FAULT_OVERSPEED,
  CHASSIS_CONTROLLER_FAULT_COMMAND_TIMEOUT,
  CHASSIS_CONTROLLER_FAULT_CONTROL_TIMING
} ChassisControllerFault;

typedef struct
{
  bool active;
  ChassisControllerFault fault;
  ChassisVelocityCommand command;
  ChassisWheelTargets targets;
  float measured_counts_per_second[MOTOR_DRIVER_WHEEL_COUNT];
  int16_t output_permille[MOTOR_DRIVER_WHEEL_COUNT];
  uint32_t limited_mask;
  uint32_t last_command_ms;
  uint32_t last_control_ms;
  uint32_t feedback_time_ms;
  uint32_t feedback_sequence;
  uint32_t motion_generation;
  uint32_t pivot_mask;
  uint32_t output_limit_reasons[MOTOR_DRIVER_WHEEL_COUNT];
} ChassisControllerStatus;

/** Minimal coherent snapshot for nested safety checks (not a wire format). */
typedef struct
{
  bool active;
  ChassisControllerFault fault;
} ChassisControllerSafetyState;

void ChassisController_Init(void);

/** Start from a disabled state using a valid non-zero velocity command. */
bool ChassisController_Start(const ChassisVelocityCommand *command,
                             uint32_t now_ms);

/**
 * Start closed-loop control while adopting already-enabled supervised motor
 * outputs. This is reserved for a bounded startup-assist handoff.
 */
bool ChassisController_StartFromActiveOutputs(
  const ChassisVelocityCommand *command,
  uint32_t now_ms);

/** Refresh/change the command and its timeout heartbeat while active. */
bool ChassisController_SetCommand(const ChassisVelocityCommand *command,
                                  uint32_t now_ms);

/** Run one controller step from the configured motor sample task. */
void ChassisController_Process(uint32_t now_ms);

/** Enforce E-stop and command-heartbeat timeout from the safety task. */
void ChassisController_SafetyProcess(uint32_t now_ms);

/** Immediately disable STBY, clear PWM, and reset all wheel controllers. */
void ChassisController_Stop(void);

bool ChassisController_IsActive(void);
bool ChassisController_GetStatus(ChassisControllerStatus *status);
bool ChassisController_GetSafetyState(ChassisControllerSafetyState *state);
const char *ChassisController_GetFaultName(ChassisControllerFault fault);

#ifdef __cplusplus
}
#endif

#endif /* APPLICATION_CHASSIS_CHASSIS_CONTROLLER_H_ */
