/**
 * @file battery_monitor.c
 * @brief D24A supply-voltage monitoring through the CubeMX-owned ADC input.
 */

#include "battery_monitor.h"

#include <stddef.h>
#include <stdio.h>

#include "FreeRTOS.h"
#include "adc.h"
#include "battery_config.h"
#include "debug_log.h"
#include "stm32f4xx_ll_adc.h"
#include "task.h"

#define BATTERY_FILTER_FRACTION_BITS 8U

static BatteryMonitorSnapshot s_snapshot;
static uint32_t s_filtered_raw_q8;

static bool BatteryMonitor_AcquireAverageDetailed(uint16_t *raw_average,
                                                  uint16_t *raw_min,
                                                  uint16_t *raw_max,
                                                  uint16_t *raw_samples);

static void BatteryMonitor_RunGroundPadSelfTest(void)
{
#if BATTERY_GROUND_PAD_SELF_TEST_ENABLED
  GPIO_InitTypeDef gpio = { 0 };
  GPIO_PinState pin_state;

  /* Temporary PC1 diagnostic only; restore CubeMX analog mode immediately. */
  gpio.Pin = GPIO_PIN_1;
  gpio.Mode = GPIO_MODE_INPUT;
  gpio.Pull = GPIO_PULLUP;
  gpio.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &gpio);
  for (volatile uint32_t settle = 0U; settle < 1000U; ++settle)
  {
    __NOP();
  }
  pin_state = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_1);

  gpio.Mode = GPIO_MODE_ANALOG;
  gpio.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &gpio);
  /* STM32F4 HAL intentionally skips PUPDR writes in analog mode. Clear the
   * temporary diagnostic pull-up explicitly before any ADC conversion. */
  CLEAR_BIT(GPIOC->PUPDR, GPIO_PUPDR_PUPDR1);

  if (pin_state == GPIO_PIN_RESET)
  {
    DEBUG_LOG_INFO(
      "Battery PC1 pad self-test: LOW with internal pull-up; external GND detected");
  }
  else
  {
    DEBUG_LOG_ERROR(
      "Battery PC1 pad self-test: HIGH with internal pull-up; PC1 is not externally grounded");
  }
#endif
}

#if BATTERY_ADC_STARTUP_DIAGNOSTIC_ENABLED
static bool BatteryMonitor_ConfigChannel(uint32_t channel,
                                         uint32_t sampling_time)
{
  ADC_ChannelConfTypeDef config = { 0 };

  config.Channel = channel;
  config.Rank = 1U;
  config.SamplingTime = sampling_time;
  return HAL_ADC_ConfigChannel(&hadc1, &config) == HAL_OK;
}
#endif

static void BatteryMonitor_RunAdcStartupDiagnostic(void)
{
#if BATTERY_ADC_STARTUP_DIAGNOSTIC_ENABLED
  uint16_t external_samples[BATTERY_ADC_SAMPLES_PER_UPDATE];
  uint16_t external_average = 0U;
  uint16_t external_min = 0U;
  uint16_t external_max = 0U;
  uint16_t vref_average = 0U;
  uint16_t vref_min = 0U;
  uint16_t vref_max = 0U;
  const uint16_t vref_cal = *VREFINT_CAL_ADDR;
  uint32_t estimated_vdda_mv = 0U;
  char sample_line[160];
  int used;

  DEBUG_LOG_INFO(
    "ADC diag regs: PC1 mode=%lu pull=%lu CR1=%08lX CR2=%08lX SQR3=%08lX SMPR1=%08lX CCR=%08lX",
    (unsigned long)((GPIOC->MODER >> (1U * 2U)) & 0x3U),
    (unsigned long)((GPIOC->PUPDR >> (1U * 2U)) & 0x3U),
    (unsigned long)ADC1->CR1,
    (unsigned long)ADC1->CR2,
    (unsigned long)ADC1->SQR3,
    (unsigned long)ADC1->SMPR1,
    (unsigned long)ADC->CCR);

  if (BatteryMonitor_AcquireAverageDetailed(&external_average,
                                             &external_min,
                                             &external_max,
                                             external_samples))
  {
    used = snprintf(sample_line, sizeof(sample_line),
                    "ADC diag PC1 samples:");
    for (uint32_t index = 0U;
         (index < BATTERY_ADC_SAMPLES_PER_UPDATE) &&
         (used > 0) && ((size_t)used < sizeof(sample_line));
         ++index)
    {
      const int appended = snprintf(&sample_line[used],
                                    sizeof(sample_line) - (size_t)used,
                                    " %u",
                                    (unsigned int)external_samples[index]);
      if (appended < 0)
      {
        break;
      }
      used += appended;
    }
    DEBUG_LOG_INFO("%s", sample_line);
    DEBUG_LOG_INFO("ADC diag PC1: avg=%u min=%u max=%u",
                   (unsigned int)external_average,
                   (unsigned int)external_min,
                   (unsigned int)external_max);
  }
  else
  {
    DEBUG_LOG_ERROR("ADC diag PC1 acquisition failed");
  }

  if (BatteryMonitor_ConfigChannel(ADC_CHANNEL_VREFINT,
                                   ADC_SAMPLETIME_480CYCLES))
  {
    /* VREFINT requires at least 10 us after enabling its internal path. */
    for (volatile uint32_t settle = 0U; settle < 4000U; ++settle)
    {
      __NOP();
    }

    if (BatteryMonitor_AcquireAverageDetailed(&vref_average,
                                               &vref_min,
                                               &vref_max,
                                               NULL))
    {
      if (vref_average != 0U)
      {
        estimated_vdda_mv =
          ((uint32_t)vref_cal * VREFINT_CAL_VREF) / vref_average;
      }
      DEBUG_LOG_INFO(
        "ADC diag VREFINT: avg=%u min=%u max=%u factory_cal=%u estimated_VDDA=%lumV",
        (unsigned int)vref_average,
        (unsigned int)vref_min,
        (unsigned int)vref_max,
        (unsigned int)vref_cal,
        (unsigned long)estimated_vdda_mv);
    }
    else
    {
      DEBUG_LOG_ERROR("ADC diag VREFINT acquisition failed");
    }
  }
  else
  {
    DEBUG_LOG_ERROR("ADC diag VREFINT channel configuration failed");
  }

  if (!BatteryMonitor_ConfigChannel(ADC_CHANNEL_11,
                                    ADC_SAMPLETIME_144CYCLES))
  {
    DEBUG_LOG_ERROR("ADC diag failed to restore ADC1_IN11");
  }
  ADC->CCR &= ~ADC_CCR_TSVREFE;
#endif
}

static uint16_t BatteryMonitor_SaturateU16(uint32_t value)
{
  return (value > UINT16_MAX) ? UINT16_MAX : (uint16_t)value;
}

static uint16_t BatteryMonitor_RawToAdcMillivolts(uint16_t raw)
{
  const uint32_t scaled =
    ((uint32_t)raw * BATTERY_ADC_REFERENCE_MV) +
    (BATTERY_ADC_FULL_SCALE_COUNTS / 2U);
  return BatteryMonitor_SaturateU16(
    scaled / BATTERY_ADC_FULL_SCALE_COUNTS);
}

static uint16_t BatteryMonitor_AdcToSupplyMillivolts(uint16_t adc_mv)
{
  const uint64_t numerator =
    (uint64_t)adc_mv * BATTERY_DIVIDER_NUMERATOR *
    BATTERY_CALIBRATION_NUMERATOR;
  const uint32_t denominator =
    BATTERY_DIVIDER_DENOMINATOR * BATTERY_CALIBRATION_DENOMINATOR;
  return BatteryMonitor_SaturateU16(
    (uint32_t)((numerator + (denominator / 2U)) / denominator));
}

static BatteryMonitorState BatteryMonitor_Classify(uint16_t supply_mv)
{
#if BATTERY_PROTECTION_ENABLED
  if (supply_mv <= BATTERY_CRITICAL_THRESHOLD_MV)
  {
    return BATTERY_MONITOR_STATE_CRITICAL;
  }
  if (supply_mv <= BATTERY_LOW_THRESHOLD_MV)
  {
    return BATTERY_MONITOR_STATE_LOW;
  }
  return BATTERY_MONITOR_STATE_NORMAL;
#else
  (void)supply_mv;
  return BATTERY_MONITOR_STATE_MONITOR_ONLY;
#endif
}

static bool BatteryMonitor_AcquireAverageDetailed(uint16_t *raw_average,
                                                  uint16_t *raw_min,
                                                  uint16_t *raw_max,
                                                  uint16_t *raw_samples)
{
  uint32_t sum = 0U;
  uint16_t minimum = UINT16_MAX;
  uint16_t maximum = 0U;

  if (raw_average == NULL)
  {
    return false;
  }

  for (uint32_t sample = 0U;
       sample < BATTERY_ADC_SAMPLES_PER_UPDATE;
       ++sample)
  {
    if (HAL_ADC_Start(&hadc1) != HAL_OK)
    {
      (void)HAL_ADC_Stop(&hadc1);
      return false;
    }
    if (HAL_ADC_PollForConversion(&hadc1,
                                  BATTERY_ADC_POLL_TIMEOUT_MS) != HAL_OK)
    {
      (void)HAL_ADC_Stop(&hadc1);
      return false;
    }
    const uint16_t value = (uint16_t)HAL_ADC_GetValue(&hadc1);
    sum += value;
    if (value < minimum)
    {
      minimum = value;
    }
    if (value > maximum)
    {
      maximum = value;
    }
    if (raw_samples != NULL)
    {
      raw_samples[sample] = value;
    }
    if (HAL_ADC_Stop(&hadc1) != HAL_OK)
    {
      return false;
    }
  }

  *raw_average = (uint16_t)(
    (sum + (BATTERY_ADC_SAMPLES_PER_UPDATE / 2U)) /
    BATTERY_ADC_SAMPLES_PER_UPDATE);
  if (raw_min != NULL)
  {
    *raw_min = minimum;
  }
  if (raw_max != NULL)
  {
    *raw_max = maximum;
  }
  return true;
}

static bool BatteryMonitor_AcquireAverage(uint16_t *raw_average)
{
  return BatteryMonitor_AcquireAverageDetailed(raw_average,
                                                NULL,
                                                NULL,
                                                NULL);
}

bool BatteryMonitor_Init(uint32_t now_ms)
{
  taskENTER_CRITICAL();
  s_snapshot.initialized = (hadc1.Instance == ADC1);
  s_snapshot.valid = false;
  s_snapshot.protection_enabled =
    (BATTERY_PROTECTION_ENABLED != 0U);
  s_snapshot.state = BATTERY_MONITOR_STATE_NOT_READY;
  s_snapshot.sample_time_ms = now_ms;
  s_snapshot.sample_count = 0U;
  s_snapshot.adc_error_count = 0U;
  s_snapshot.consecutive_adc_errors = 0U;
  s_snapshot.raw_average = 0U;
  s_snapshot.filtered_raw = 0U;
  s_snapshot.adc_input_mv = 0U;
  s_snapshot.supply_mv = 0U;
  s_filtered_raw_q8 = 0U;
  taskEXIT_CRITICAL();

  if (!s_snapshot.initialized)
  {
    DEBUG_LOG_ERROR("Battery monitor initialization failed: ADC1 unavailable");
    return false;
  }

  BatteryMonitor_RunGroundPadSelfTest();
  BatteryMonitor_RunAdcStartupDiagnostic();

  DEBUG_LOG_INFO(
    "Battery monitor ready: %s polling, %lu samples/%lums, D24A divider=1/%lu",
    BATTERY_ADC_SOURCE_TEXT,
    (unsigned long)BATTERY_ADC_SAMPLES_PER_UPDATE,
    (unsigned long)BATTERY_SAMPLE_PERIOD_MS,
    (unsigned long)BATTERY_DIVIDER_NUMERATOR);
  DEBUG_LOG_WARN(
    "Battery protection disabled: monitor-only until 3S low/critical thresholds are validated");
  return true;
}

bool BatteryMonitor_Update(uint32_t now_ms)
{
  BatteryMonitorState previous_state;
  BatteryMonitorState next_state;
  uint16_t raw_average;
  uint16_t filtered_raw;
  uint16_t adc_input_mv;
  uint16_t supply_mv;
  bool previously_valid;

  if (!s_snapshot.initialized)
  {
    return false;
  }

  if (!BatteryMonitor_AcquireAverage(&raw_average))
  {
    bool report_fault = false;

    taskENTER_CRITICAL();
    previous_state = s_snapshot.state;
    ++s_snapshot.adc_error_count;
    if (s_snapshot.consecutive_adc_errors < UINT16_MAX)
    {
      ++s_snapshot.consecutive_adc_errors;
    }
    if (s_snapshot.consecutive_adc_errors >=
        BATTERY_ADC_FAULT_CONSECUTIVE_LIMIT)
    {
      s_snapshot.valid = false;
      s_snapshot.state = BATTERY_MONITOR_STATE_ADC_FAULT;
      report_fault = (previous_state != BATTERY_MONITOR_STATE_ADC_FAULT);
    }
    taskEXIT_CRITICAL();

    if (report_fault)
    {
      DEBUG_LOG_ERROR(
        "Battery ADC fault: %u consecutive acquisition failures",
        (unsigned int)BATTERY_ADC_FAULT_CONSECUTIVE_LIMIT);
    }
    return false;
  }

  taskENTER_CRITICAL();
  previously_valid = s_snapshot.valid;
  previous_state = s_snapshot.state;
  taskEXIT_CRITICAL();

  if (!previously_valid)
  {
    s_filtered_raw_q8 = (uint32_t)raw_average <<
      BATTERY_FILTER_FRACTION_BITS;
  }
  else
  {
    const int32_t target_q8 =
      (int32_t)((uint32_t)raw_average << BATTERY_FILTER_FRACTION_BITS);
    const int32_t filtered_q8 = (int32_t)s_filtered_raw_q8;
    s_filtered_raw_q8 = (uint32_t)(
      filtered_q8 +
      ((target_q8 - filtered_q8) / (1L << BATTERY_FILTER_SHIFT)));
  }

  filtered_raw = (uint16_t)(
    (s_filtered_raw_q8 + (1UL << (BATTERY_FILTER_FRACTION_BITS - 1U))) >>
    BATTERY_FILTER_FRACTION_BITS);
  adc_input_mv = BatteryMonitor_RawToAdcMillivolts(filtered_raw);
  supply_mv = BatteryMonitor_AdcToSupplyMillivolts(adc_input_mv);
  next_state = BatteryMonitor_Classify(supply_mv);

  taskENTER_CRITICAL();
  s_snapshot.valid = true;
  s_snapshot.state = next_state;
  s_snapshot.sample_time_ms = now_ms;
  ++s_snapshot.sample_count;
  s_snapshot.consecutive_adc_errors = 0U;
  s_snapshot.raw_average = raw_average;
  s_snapshot.filtered_raw = filtered_raw;
  s_snapshot.adc_input_mv = adc_input_mv;
  s_snapshot.supply_mv = supply_mv;
  taskEXIT_CRITICAL();

  if ((previous_state == BATTERY_MONITOR_STATE_ADC_FAULT) &&
      (next_state != BATTERY_MONITOR_STATE_ADC_FAULT))
  {
    DEBUG_LOG_INFO("Battery ADC acquisition recovered");
  }
  return true;
}

bool BatteryMonitor_GetSnapshot(BatteryMonitorSnapshot *snapshot)
{
  if (snapshot == NULL)
  {
    return false;
  }

  taskENTER_CRITICAL();
  *snapshot = s_snapshot;
  taskEXIT_CRITICAL();
  return snapshot->initialized;
}

const char *BatteryMonitor_GetStateName(BatteryMonitorState state)
{
  switch (state)
  {
    case BATTERY_MONITOR_STATE_NOT_READY:
      return "NOT_READY";
    case BATTERY_MONITOR_STATE_MONITOR_ONLY:
      return "MONITOR_ONLY";
    case BATTERY_MONITOR_STATE_NORMAL:
      return "NORMAL";
    case BATTERY_MONITOR_STATE_LOW:
      return "LOW";
    case BATTERY_MONITOR_STATE_CRITICAL:
      return "CRITICAL";
    case BATTERY_MONITOR_STATE_ADC_FAULT:
      return "ADC_FAULT";
    default:
      return "UNKNOWN";
  }
}
