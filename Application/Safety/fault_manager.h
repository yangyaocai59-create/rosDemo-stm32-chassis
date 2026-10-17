/**
 * @file fault_manager.h
 * @brief Central task-health and production wheel-stall supervision.
 */

#ifndef APPLICATION_SAFETY_FAULT_MANAGER_H_
#define APPLICATION_SAFETY_FAULT_MANAGER_H_

#include <stdbool.h>
#include <stdint.h>

#include "motor_driver.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
  FAULT_MANAGER_TASK_MOTOR = 0,
  FAULT_MANAGER_TASK_SAFETY,
  FAULT_MANAGER_TASK_USB_RX,
  FAULT_MANAGER_TASK_TELEMETRY,
  FAULT_MANAGER_TASK_IMU,
  FAULT_MANAGER_TASK_BATTERY,
  FAULT_MANAGER_TASK_COUNT
} FaultManagerTask;

typedef uint32_t FaultManagerFaultFlags;

enum
{
  FAULT_MANAGER_FAULT_NONE = 0U,
  FAULT_MANAGER_FAULT_TASK_HEARTBEAT = (1UL << 0),
  FAULT_MANAGER_FAULT_SYSTEM_INIT = (1UL << 1),
  FAULT_MANAGER_FAULT_WHEEL_STALL = (1UL << 2)
};

typedef struct
{
  bool initialized;
  bool system_init_complete;
  FaultManagerFaultFlags latched_faults;
  uint32_t seen_task_mask;
  uint32_t stale_task_mask;
  uint32_t critical_stale_task_mask;
  uint32_t latched_stalled_wheel_mask;
  uint32_t current_stalled_wheel_mask;
  uint32_t first_fault_ms;
  uint32_t last_fault_ms;
  uint32_t trip_count;
} FaultManagerStatus;

/** Initialize health supervision. This does not enable or feed IWDG. */
void FaultManager_Init(uint32_t now_ms);

/** Record one successful task loop iteration. */
void FaultManager_Heartbeat(FaultManagerTask task, uint32_t now_ms);

/** Mark the CubeMX USB/system one-shot initialization as completed. */
void FaultManager_MarkSystemInitComplete(uint32_t now_ms);

/**
 * Observe the production controller only. Diagnostics and IDLE never generate
 * a wheel-stall fault.
 */
void FaultManager_ObserveMotion(
  uint32_t now_ms,
  bool runtime_active,
  const float target_counts_per_second[MOTOR_DRIVER_WHEEL_COUNT],
  const float measured_counts_per_second[MOTOR_DRIVER_WHEEL_COUNT],
  const int16_t output_permille[MOTOR_DRIVER_WHEEL_COUNT]);

/** Refresh task-health state and latch any new critical health fault. */
void FaultManager_Process(uint32_t now_ms);

/** True while a latched critical fault must inhibit production motion. */
bool FaultManager_HasCriticalFault(void);

/** True only when current health is valid and no fault remains latched. */
bool FaultManager_CanEnableMotion(void);

/** Clear recoverable latched faults only after their live causes disappear. */
bool FaultManager_Clear(uint32_t now_ms);

/** Copy a coherent health/fault snapshot. */
bool FaultManager_GetStatus(FaultManagerStatus *status);

#ifdef __cplusplus
}
#endif

#endif /* APPLICATION_SAFETY_FAULT_MANAGER_H_ */
