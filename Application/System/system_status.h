/**
 * @file system_status.h
 * @brief Persistent runtime system status captured during early startup.
 */

#ifndef APPLICATION_SYSTEM_SYSTEM_STATUS_H_
#define APPLICATION_SYSTEM_SYSTEM_STATUS_H_

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef uint32_t SystemResetCause;

enum
{
  SYSTEM_RESET_CAUSE_NONE       = 0U,
  SYSTEM_RESET_CAUSE_POR_PDR    = (1UL << 0),
  SYSTEM_RESET_CAUSE_PIN        = (1UL << 1),
  SYSTEM_RESET_CAUSE_SOFTWARE   = (1UL << 2),
  SYSTEM_RESET_CAUSE_IWDG       = (1UL << 3),
  SYSTEM_RESET_CAUSE_WWDG       = (1UL << 4),
  SYSTEM_RESET_CAUSE_BOR        = (1UL << 5),
  SYSTEM_RESET_CAUSE_LOW_POWER  = (1UL << 6),
  SYSTEM_RESET_CAUSE_UNKNOWN    = (1UL << 31)
};

/** Capture RCC reset flags once and then clear the hardware flags. */
void SystemStatus_CaptureResetCause(void);

/** Return the reset-cause bitmask retained for the current boot. */
SystemResetCause SystemStatus_GetResetCause(void);

/** Convert a reset-cause bitmask into a bounded, human-readable string. */
void SystemStatus_ResetCauseToString(SystemResetCause cause,
                                     char *buffer,
                                     size_t buffer_size);

#ifdef __cplusplus
}
#endif

#endif /* APPLICATION_SYSTEM_SYSTEM_STATUS_H_ */
