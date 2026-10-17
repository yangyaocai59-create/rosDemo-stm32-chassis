/**
 * @file battery_monitor.h
 * @brief Polled ADC battery acquisition, filtering and coherent telemetry.
 */

#ifndef APPLICATION_POWER_BATTERY_MONITOR_H_
#define APPLICATION_POWER_BATTERY_MONITOR_H_

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
  BATTERY_MONITOR_STATE_NOT_READY = 0,
  BATTERY_MONITOR_STATE_MONITOR_ONLY,
  BATTERY_MONITOR_STATE_NORMAL,
  BATTERY_MONITOR_STATE_LOW,
  BATTERY_MONITOR_STATE_CRITICAL,
  BATTERY_MONITOR_STATE_ADC_FAULT
} BatteryMonitorState;

typedef struct
{
  bool initialized;
  bool valid;
  bool protection_enabled;
  BatteryMonitorState state;
  uint32_t sample_time_ms;
  uint32_t sample_count;
  uint32_t adc_error_count;
  uint16_t consecutive_adc_errors;
  uint16_t raw_average;
  uint16_t filtered_raw;
  uint16_t adc_input_mv;
  uint16_t supply_mv;
} BatteryMonitorSnapshot;

/** Initialize application-owned state; ADC hardware remains CubeMX-owned. */
bool BatteryMonitor_Init(uint32_t now_ms);

/** Acquire and filter one oversampled ADC measurement using polling only. */
bool BatteryMonitor_Update(uint32_t now_ms);

/** Copy one coherent snapshot for logs or USB telemetry. */
bool BatteryMonitor_GetSnapshot(BatteryMonitorSnapshot *snapshot);

const char *BatteryMonitor_GetStateName(BatteryMonitorState state);

#ifdef __cplusplus
}
#endif

#endif /* APPLICATION_POWER_BATTERY_MONITOR_H_ */
