/**
 * @file debug_log.c
 * @brief Central USART1 logging implementation.
 */

#include "debug_log.h"

#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "FreeRTOS.h"
#include "board_config.h"
#include "chassis_config.h"
#include "cmsis_os2.h"
#include "firmware_info.h"
#include "imu_config.h"
#include "stm32f4xx_hal.h"
#include "system_status.h"
#include "task.h"
#include "usart.h"

#define DEBUG_LOG_BUFFER_SIZE          256U
#define DEBUG_LOG_UART_TIMEOUT_MS      100U
#define DEBUG_LOG_MUTEX_TIMEOUT_TICKS  100U
#define DEBUG_LOG_DEFERRED_DEPTH       32U
#define DEBUG_LOG_NONBLOCKING_TASKS     3U

static char s_log_buffer[DEBUG_LOG_BUFFER_SIZE];
static StaticSemaphore_t s_log_mutex_control_block;
static osMutexId_t s_log_mutex;
static bool s_log_initialized;
static char s_deferred_records[DEBUG_LOG_DEFERRED_DEPTH][DEBUG_LOG_BUFFER_SIZE];
static uint16_t s_deferred_lengths[DEBUG_LOG_DEFERRED_DEPTH];
static uint8_t s_deferred_head;
static uint8_t s_deferred_tail;
static osThreadId_t s_deferred_owner;
static uint32_t s_deferred_nesting;
static uint32_t s_deferred_dropped;
static osThreadId_t s_nonblocking_tasks[DEBUG_LOG_NONBLOCKING_TASKS];
/* Per registered producer, not per call: safetyTask has a 1 KiB stack.
 * Formatting is outside the queue critical section; ISR logging is rejected. */
static char s_nonblocking_scratch[DEBUG_LOG_NONBLOCKING_TASKS][DEBUG_LOG_BUFFER_SIZE];

static const char *const s_level_names[DEBUG_LOG_LEVEL_COUNT] = {
  "DEBUG", "INFO", "WARN", "ERROR"
};

static bool DebugLog_Lock(bool *locked)
{
  *locked = false;

  if (osKernelGetState() == osKernelRunning)
  {
    if (osMutexAcquire(s_log_mutex, DEBUG_LOG_MUTEX_TIMEOUT_TICKS) != osOK)
    {
      return false;
    }
    *locked = true;
  }

  return true;
}

static void DebugLog_Unlock(bool locked)
{
  if (locked)
  {
    (void)osMutexRelease(s_log_mutex);
  }
}

bool DebugLog_Init(void)
{
  static const osMutexAttr_t mutex_attributes = {
    .name = "debugLogMutex",
    .cb_mem = &s_log_mutex_control_block,
    .cb_size = sizeof(s_log_mutex_control_block)
  };

  if (s_log_initialized)
  {
    return true;
  }

  s_log_mutex = osMutexNew(&mutex_attributes);
  s_log_initialized = (s_log_mutex != NULL);
  return s_log_initialized;
}

bool DebugLog_RegisterCurrentTaskNonBlocking(void)
{
  if ((!s_log_initialized) || (__get_IPSR() != 0U) ||
      (osKernelGetState() != osKernelRunning)) { return false; }
  const osThreadId_t current = osThreadGetId();
  if (current == NULL) { return false; }
  bool registered = false;
  taskENTER_CRITICAL();
  for (uint32_t i = 0U; i < DEBUG_LOG_NONBLOCKING_TASKS; ++i)
  {
    if ((s_nonblocking_tasks[i] == current) ||
        (s_nonblocking_tasks[i] == NULL))
    {
      s_nonblocking_tasks[i] = current;
      registered = true;
      break;
    }
  }
  taskEXIT_CRITICAL();
  return registered;
}

bool DebugLog_BeginDeferred(void)
{
  const osThreadId_t current = osThreadGetId();
  bool accepted = false;
  if ((!s_log_initialized) || (current == NULL) || (__get_IPSR() != 0U) ||
      (osKernelGetState() != osKernelRunning))
  { return false; }
  taskENTER_CRITICAL();
  if ((s_deferred_owner == NULL) || (s_deferred_owner == current))
  {
    s_deferred_owner = current;
    ++s_deferred_nesting;
    accepted = true;
  }
  taskEXIT_CRITICAL();
  return accepted;
}

void DebugLog_EndDeferred(void)
{
  if (osKernelGetState() != osKernelRunning) { return; }
  const osThreadId_t current = osThreadGetId();
  taskENTER_CRITICAL();
  if ((s_deferred_owner == current) && (s_deferred_nesting != 0U))
  {
    if (--s_deferred_nesting == 0U) { s_deferred_owner = NULL; }
  }
  taskEXIT_CRITICAL();
}

uint32_t DebugLog_GetDeferredDroppedCount(void)
{
  if (osKernelGetState() != osKernelRunning) { return s_deferred_dropped; }
  uint32_t dropped;
  taskENTER_CRITICAL(); dropped = s_deferred_dropped; taskEXIT_CRITICAL();
  return dropped;
}

static bool DebugLog_DeferToBuffer(char *record, DebugLog_Level level,
                                  const char *format, va_list arguments)
{
  int prefix = snprintf(record, DEBUG_LOG_BUFFER_SIZE, "[%010lu] [%-5s] ",
    (unsigned long)HAL_GetTick(), s_level_names[level]);
  if ((prefix < 0) || ((size_t)prefix >= DEBUG_LOG_BUFFER_SIZE - 3U)) { return false; }
  int body = vsnprintf(&record[prefix], DEBUG_LOG_BUFFER_SIZE - (size_t)prefix,
                       format, arguments);
  if (body < 0) { return false; }
  size_t used = (size_t)prefix + (size_t)body;
  if (used > DEBUG_LOG_BUFFER_SIZE - 3U) { used = DEBUG_LOG_BUFFER_SIZE - 3U; }
  record[used++] = '\r'; record[used++] = '\n'; record[used] = '\0';
  bool queued = false;
  taskENTER_CRITICAL();
  const uint8_t next = (uint8_t)((s_deferred_head + 1U) % DEBUG_LOG_DEFERRED_DEPTH);
  if (next != s_deferred_tail)
  {
    (void)memcpy(s_deferred_records[s_deferred_head], record, used);
    s_deferred_lengths[s_deferred_head] = (uint16_t)used;
    s_deferred_head = next;
    queued = true;
  }
  else { ++s_deferred_dropped; }
  taskEXIT_CRITICAL();
  return queued;
}

static bool DebugLog_DeferRecord(DebugLog_Level level, const char *format,
                                 va_list arguments)
{
  char record[DEBUG_LOG_BUFFER_SIZE];
  return DebugLog_DeferToBuffer(record, level, format, arguments);
}

void DebugLog_ProcessDeferred(uint32_t max_records)
{
  char record[DEBUG_LOG_BUFFER_SIZE];
  if ((!s_log_initialized) || (__get_IPSR() != 0U) ||
      (osKernelGetState() != osKernelRunning)) { return; }
  for (uint32_t index = 0U; index < max_records; ++index)
  {
    uint16_t length = 0U;
    bool locked;
    /* Single consumer. Never hold a critical section across UART/mutex waits. */
    if (!DebugLog_Lock(&locked)) { return; }
    taskENTER_CRITICAL();
    if (s_deferred_head != s_deferred_tail)
    {
      length = s_deferred_lengths[s_deferred_tail];
      (void)memcpy(record, s_deferred_records[s_deferred_tail], length);
      s_deferred_tail = (uint8_t)((s_deferred_tail + 1U) % DEBUG_LOG_DEFERRED_DEPTH);
    }
    taskEXIT_CRITICAL();
    if (length != 0U && HAL_UART_Transmit(&huart1, (uint8_t *)record, length,
                                         DEBUG_LOG_UART_TIMEOUT_MS) != HAL_OK)
    {
      taskENTER_CRITICAL(); ++s_deferred_dropped; taskEXIT_CRITICAL();
    }
    DebugLog_Unlock(locked);
    if (length == 0U) { return; }
  }
}

bool DebugLog_Printf(DebugLog_Level level, const char *format, ...)
{
  bool locked;
  int result;
  size_t used;
  va_list arguments;

  if ((!s_log_initialized) || (format == NULL) ||
      (level >= DEBUG_LOG_LEVEL_COUNT) ||
      (level < (DebugLog_Level)DEBUG_LOG_MIN_LEVEL) ||
      (__get_IPSR() != 0U))
  {
    return false;
  }

  bool deferred = false;
  int32_t producer = -1;
  if (osKernelGetState() == osKernelRunning)
  {
    const osThreadId_t current = osThreadGetId();
    taskENTER_CRITICAL();
    deferred = (s_deferred_owner != NULL) && (s_deferred_owner == current);
    for (uint32_t i = 0U; i < DEBUG_LOG_NONBLOCKING_TASKS; ++i)
    {
      if ((current != NULL) && (s_nonblocking_tasks[i] == current))
      { deferred = true; producer = (int32_t)i; break; }
    }
    taskEXIT_CRITICAL();
  }
  if (deferred)
  {
    va_start(arguments, format);
    const bool queued = (producer >= 0) ?
      DebugLog_DeferToBuffer(s_nonblocking_scratch[producer], level, format, arguments) :
      DebugLog_DeferRecord(level, format, arguments);
    va_end(arguments);
    return queued;
  }

  if (!DebugLog_Lock(&locked))
  {
    return false;
  }

  result = snprintf(s_log_buffer,
                    sizeof(s_log_buffer),
                    "[%010lu] [%-5s] ",
                    (unsigned long)HAL_GetTick(),
                    s_level_names[level]);
  if (result < 0)
  {
    DebugLog_Unlock(locked);
    return false;
  }

  used = (size_t)result;
  if (used >= sizeof(s_log_buffer))
  {
    used = sizeof(s_log_buffer) - 1U;
  }

  va_start(arguments, format);
  result = vsnprintf(&s_log_buffer[used],
                     sizeof(s_log_buffer) - used,
                     format,
                     arguments);
  va_end(arguments);

  if (result < 0)
  {
    DebugLog_Unlock(locked);
    return false;
  }

  used += (size_t)result;
  if (used > (sizeof(s_log_buffer) - 3U))
  {
    used = sizeof(s_log_buffer) - 3U;
  }

  s_log_buffer[used++] = '\r';
  s_log_buffer[used++] = '\n';
  s_log_buffer[used] = '\0';

  result = (HAL_UART_Transmit(&huart1,
                              (uint8_t *)s_log_buffer,
                              (uint16_t)used,
                              DEBUG_LOG_UART_TIMEOUT_MS) == HAL_OK);

  DebugLog_Unlock(locked);
  return (result != 0);
}

void DebugLog_FormatInt64(int64_t value,
                          char output[DEBUG_LOG_INT64_TEXT_SIZE])
{
  char reversed_digits[20];
  uint64_t magnitude;
  uint32_t digit_count = 0U;
  uint32_t output_index = 0U;
  const bool is_negative = (value < 0);

  if (is_negative)
  {
    /* This form also handles INT64_MIN without signed overflow. */
    magnitude = (uint64_t)(-(value + 1)) + 1U;
  }
  else
  {
    magnitude = (uint64_t)value;
  }

  do
  {
    reversed_digits[digit_count++] = (char)('0' + (magnitude % 10U));
    magnitude /= 10U;
  } while (magnitude != 0U);

  if (is_negative)
  {
    output[output_index++] = '-';
  }

  while (digit_count > 0U)
  {
    output[output_index++] = reversed_digits[--digit_count];
  }
  output[output_index] = '\0';
}

void DebugLog_PrintBootBanner(void)
{
  char reset_cause_text[112];
  const SystemResetCause reset_cause = SystemStatus_GetResetCause();

  SystemStatus_ResetCauseToString(reset_cause,
                                  reset_cause_text,
                                  sizeof(reset_cause_text));

  DEBUG_LOG_INFO("================================================");
  DEBUG_LOG_INFO("Firmware name = %s", FIRMWARE_NAME);
  DEBUG_LOG_INFO("Firmware version = %s", FIRMWARE_VERSION);
  DEBUG_LOG_INFO("MCU = %s", FIRMWARE_MCU);
  DEBUG_LOG_INFO("System clock = %lu MHz",
                 (unsigned long)(HAL_RCC_GetSysClockFreq() / 1000000U));
  DEBUG_LOG_INFO("RTOS = FreeRTOS %s / CMSIS-RTOS2 (kernel initialized)",
                 tskKERNEL_VERSION_NUMBER);
  DEBUG_LOG_INFO("Debug UART = USART1 / PA9-PA10 / 115200 8-N-1");
  DEBUG_LOG_INFO("USB CDC init status = pending");
  DEBUG_LOG_INFO("Build = %s %s", __DATE__, __TIME__);
  DEBUG_LOG_INFO("Reset cause = 0x%08lX (%s)",
                 (unsigned long)reset_cause,
                 reset_cause_text);
  DEBUG_LOG_INFO("ENC: %s", BOARD_ENCODER_MAPPING_TEXT);
  DEBUG_LOG_INFO("PWM: %s", BOARD_PWM_MAPPING_TEXT);
  DEBUG_LOG_INFO("BAT: %s", BOARD_BATTERY_ADC_MAPPING_TEXT);
  DEBUG_LOG_INFO("IMU: %s", BOARD_IMU_MAPPING_TEXT);
  DEBUG_LOG_INFO("IMU frame: %s", IMU_FRAME_MAPPING_TEXT);
  DEBUG_LOG_INFO("Motor forward polarity: %s", BOARD_MOTOR_POLARITY_TEXT);
  DEBUG_LOG_INFO("Encoder forward polarity: %s", BOARD_ENCODER_POLARITY_TEXT);
  DEBUG_LOG_INFO("Encoder = %lu PPR x%lu x gear %lu = %lu counts/wheel-rev",
                 (unsigned long)CHASSIS_ENCODER_PPR,
                 (unsigned long)CHASSIS_ENCODER_QUADRATURE_MULTIPLIER,
                 (unsigned long)CHASSIS_MOTOR_GEAR_RATIO,
                 (unsigned long)CHASSIS_ENCODER_COUNTS_PER_WHEEL_REV);
  DEBUG_LOG_INFO("Geometry = wheel D%lu mm, track %lu mm, wheelbase %lu mm",
                 (unsigned long)CHASSIS_WHEEL_DIAMETER_MM,
                 (unsigned long)CHASSIS_WHEEL_TRACK_MM,
                 (unsigned long)CHASSIS_WHEELBASE_MM);
  DEBUG_LOG_INFO("ESTOP = PC13 active-low, reset-latched motor shutdown");
  DEBUG_LOG_INFO("Motor safety = D24A STBY/PD10 LOW, PD11 reserved LOW, TIM8 outputs disabled");
  DEBUG_LOG_INFO("================================================");
}
