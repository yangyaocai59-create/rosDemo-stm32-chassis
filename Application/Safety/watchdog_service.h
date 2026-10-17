/**
 * @file watchdog_service.h
 * @brief IWDG refresh arbitration based on live system/task health.
 */

#ifndef APPLICATION_SAFETY_WATCHDOG_SERVICE_H_
#define APPLICATION_SAFETY_WATCHDOG_SERVICE_H_

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
  bool initialized;
  bool armed;
  bool health_eligible;
  bool refresh_failed;
  uint32_t last_refresh_ms;
  uint32_t refresh_count;
  uint32_t withhold_event_count;
} WatchdogServiceStatus;

/** Initialize arbitration state without performing an unconditional refresh. */
void WatchdogService_Init(uint32_t now_ms);

/**
 * Evaluate live health and refresh IWDG when eligible. This must be called
 * only by safetyTask so loss of safetyTask execution also stops refreshes.
 */
void WatchdogService_Process(uint32_t now_ms);

/** Copy a coherent diagnostic snapshot. */
bool WatchdogService_GetStatus(WatchdogServiceStatus *status);

#ifdef __cplusplus
}
#endif

#endif /* APPLICATION_SAFETY_WATCHDOG_SERVICE_H_ */
