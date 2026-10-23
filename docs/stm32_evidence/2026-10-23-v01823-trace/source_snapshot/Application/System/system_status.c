/**
 * @file system_status.c
 * @brief Early reset-cause capture for diagnostics and the future fault manager.
 */

#include "system_status.h"

#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include "stm32f4xx_hal.h"

static SystemResetCause s_reset_cause = SYSTEM_RESET_CAUSE_UNKNOWN;

static void SystemStatus_AppendCause(char *buffer,
                                     size_t buffer_size,
                                     const char *text,
                                     bool *is_first)
{
  const size_t used = strlen(buffer);

  if (used >= buffer_size)
  {
    return;
  }

  (void)snprintf(&buffer[used],
                 buffer_size - used,
                 "%s%s",
                 *is_first ? "" : " | ",
                 text);
  *is_first = false;
}

void SystemStatus_CaptureResetCause(void)
{
  SystemResetCause cause = SYSTEM_RESET_CAUSE_NONE;

  if (__HAL_RCC_GET_FLAG(RCC_FLAG_PORRST) != RESET)
  {
    cause |= SYSTEM_RESET_CAUSE_POR_PDR;
  }
  if (__HAL_RCC_GET_FLAG(RCC_FLAG_PINRST) != RESET)
  {
    cause |= SYSTEM_RESET_CAUSE_PIN;
  }
  if (__HAL_RCC_GET_FLAG(RCC_FLAG_SFTRST) != RESET)
  {
    cause |= SYSTEM_RESET_CAUSE_SOFTWARE;
  }
  if (__HAL_RCC_GET_FLAG(RCC_FLAG_IWDGRST) != RESET)
  {
    cause |= SYSTEM_RESET_CAUSE_IWDG;
  }
  if (__HAL_RCC_GET_FLAG(RCC_FLAG_WWDGRST) != RESET)
  {
    cause |= SYSTEM_RESET_CAUSE_WWDG;
  }
  if (__HAL_RCC_GET_FLAG(RCC_FLAG_BORRST) != RESET)
  {
    cause |= SYSTEM_RESET_CAUSE_BOR;
  }
  if (__HAL_RCC_GET_FLAG(RCC_FLAG_LPWRRST) != RESET)
  {
    cause |= SYSTEM_RESET_CAUSE_LOW_POWER;
  }

  if (cause == SYSTEM_RESET_CAUSE_NONE)
  {
    cause = SYSTEM_RESET_CAUSE_UNKNOWN;
  }

  s_reset_cause = cause;
  __HAL_RCC_CLEAR_RESET_FLAGS();
}

SystemResetCause SystemStatus_GetResetCause(void)
{
  return s_reset_cause;
}

void SystemStatus_ResetCauseToString(SystemResetCause cause,
                                     char *buffer,
                                     size_t buffer_size)
{
  bool is_first = true;

  if ((buffer == NULL) || (buffer_size == 0U))
  {
    return;
  }

  buffer[0] = '\0';

  if ((cause & SYSTEM_RESET_CAUSE_POR_PDR) != 0U)
  {
    SystemStatus_AppendCause(buffer, buffer_size, "POR/PDR", &is_first);
  }
  if ((cause & SYSTEM_RESET_CAUSE_PIN) != 0U)
  {
    SystemStatus_AppendCause(buffer, buffer_size, "PIN RESET", &is_first);
  }
  if ((cause & SYSTEM_RESET_CAUSE_SOFTWARE) != 0U)
  {
    SystemStatus_AppendCause(buffer, buffer_size, "SOFTWARE RESET", &is_first);
  }
  if ((cause & SYSTEM_RESET_CAUSE_IWDG) != 0U)
  {
    SystemStatus_AppendCause(buffer, buffer_size, "IWDG RESET", &is_first);
  }
  if ((cause & SYSTEM_RESET_CAUSE_WWDG) != 0U)
  {
    SystemStatus_AppendCause(buffer, buffer_size, "WWDG RESET", &is_first);
  }
  if ((cause & SYSTEM_RESET_CAUSE_BOR) != 0U)
  {
    SystemStatus_AppendCause(buffer, buffer_size, "BOR", &is_first);
  }
  if ((cause & SYSTEM_RESET_CAUSE_LOW_POWER) != 0U)
  {
    SystemStatus_AppendCause(buffer, buffer_size, "LOW-POWER RESET", &is_first);
  }
  if ((cause & SYSTEM_RESET_CAUSE_UNKNOWN) != 0U)
  {
    SystemStatus_AppendCause(buffer, buffer_size, "UNKNOWN", &is_first);
  }

  if (is_first)
  {
    SystemStatus_AppendCause(buffer, buffer_size, "NONE", &is_first);
  }
}
