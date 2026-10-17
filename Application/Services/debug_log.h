/**
 * @file debug_log.h
 * @brief Thread-safe, low-frequency debug logging over the board USART1 port.
 */

#ifndef APPLICATION_SERVICES_DEBUG_LOG_H_
#define APPLICATION_SERVICES_DEBUG_LOG_H_

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
  DEBUG_LOG_LEVEL_DEBUG = 0,
  DEBUG_LOG_LEVEL_INFO,
  DEBUG_LOG_LEVEL_WARN,
  DEBUG_LOG_LEVEL_ERROR,
  DEBUG_LOG_LEVEL_COUNT
} DebugLog_Level;

/* May be overridden from the build system for production builds. */
#ifndef DEBUG_LOG_ENABLE_DEBUG
#define DEBUG_LOG_ENABLE_DEBUG 1
#endif

#ifndef DEBUG_LOG_MIN_LEVEL
#define DEBUG_LOG_MIN_LEVEL DEBUG_LOG_LEVEL_DEBUG
#endif

/* Sign + 19 decimal digits + terminator for the complete int64_t range. */
#define DEBUG_LOG_INT64_TEXT_SIZE 21U

/** Initialize the USART1 logger and its RTOS mutex. Call after osKernelInitialize. */
bool DebugLog_Init(void);

/** Permanently queue this task's records without UART/mutex waits.
 * Register once at task entry, before its first log. Bounded queue overflow
 * drops a log and increments the existing counter; it never blocks control.
 */
bool DebugLog_RegisterCurrentTaskNonBlocking(void);

/** Print one bounded log record. CRLF is appended automatically. */
bool DebugLog_Printf(DebugLog_Level level, const char *format, ...);
/* A task-local scope copies bounded records without UART/mutex waits.
 * Only telemetryTask drains these records; other tasks keep normal logging. */
bool DebugLog_BeginDeferred(void);
void DebugLog_EndDeferred(void);
void DebugLog_ProcessDeferred(uint32_t max_records);
uint32_t DebugLog_GetDeferredDroppedCount(void);

/**
 * Convert an int64_t to decimal without relying on optional newlib-nano
 * long-long printf support.
 */
void DebugLog_FormatInt64(int64_t value,
                          char output[DEBUG_LOG_INT64_TEXT_SIZE]);

/** Print firmware, clock, RTOS, reset-cause, and board-mapping information. */
void DebugLog_PrintBootBanner(void);

#if DEBUG_LOG_ENABLE_DEBUG
#define DEBUG_LOG_DEBUG(...) \
  ((void)DebugLog_Printf(DEBUG_LOG_LEVEL_DEBUG, __VA_ARGS__))
#else
#define DEBUG_LOG_DEBUG(...) ((void)0)
#endif

#define DEBUG_LOG_INFO(...) \
  ((void)DebugLog_Printf(DEBUG_LOG_LEVEL_INFO, __VA_ARGS__))
#define DEBUG_LOG_WARN(...) \
  ((void)DebugLog_Printf(DEBUG_LOG_LEVEL_WARN, __VA_ARGS__))
#define DEBUG_LOG_ERROR(...) \
  ((void)DebugLog_Printf(DEBUG_LOG_LEVEL_ERROR, __VA_ARGS__))

#ifdef __cplusplus
}
#endif

#endif /* APPLICATION_SERVICES_DEBUG_LOG_H_ */
