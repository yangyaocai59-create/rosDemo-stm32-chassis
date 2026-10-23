/**
 * @file battery_config.h
 * @brief D24A battery-voltage acquisition configuration.
 */

#ifndef APPLICATION_POWER_BATTERY_CONFIG_H_
#define APPLICATION_POWER_BATTERY_CONFIG_H_

/* Permanent D24A battery-divider input, validated against a multimeter. */
#define BATTERY_MONITOR_ENABLED                        1U
#define BATTERY_ADC_SOURCE_TEXT "ADC1_IN11/PC1"
#define BATTERY_GROUND_PAD_SELF_TEST_ENABLED           0U
#define BATTERY_ADC_STARTUP_DIAGNOSTIC_ENABLED         0U
#define BATTERY_SAMPLE_PERIOD_MS                 100U
#define BATTERY_ADC_SAMPLES_PER_UPDATE            16U
#define BATTERY_ADC_POLL_TIMEOUT_MS                 2U
#define BATTERY_ADC_FULL_SCALE_COUNTS            4095U
#define BATTERY_ADC_REFERENCE_MV                 3300U

/* D24A nominal divider: 10 kOhm high side and 1 kOhm low side => 1/11. */
#define BATTERY_DIVIDER_NUMERATOR                  11U
#define BATTERY_DIVIDER_DENOMINATOR                 1U

/*
 * 8.547 V firmware versus 8.60 V handheld-meter measurement is within 0.62%.
 * Keep unity calibration until a higher-resolution, multi-point calibration.
 */
#define BATTERY_CALIBRATION_NUMERATOR            1000U
#define BATTERY_CALIBRATION_DENOMINATOR          1000U

/* First-order IIR: filtered += (sample - filtered) / 8. */
#define BATTERY_FILTER_SHIFT                         3U
#define BATTERY_ADC_FAULT_CONSECUTIVE_LIMIT           3U
#define BATTERY_DEBUG_LOG_PERIOD_MS                5000U

/*
 * Pack is identified as 3S1P lithium-ion. Protection intentionally remains
 * disabled until the low/critical thresholds are validated with a charged
 * pack; monitoring and USB telemetry remain active.
 */
#define BATTERY_PROTECTION_ENABLED                    0U
#define BATTERY_LOW_THRESHOLD_MV                      0U
#define BATTERY_CRITICAL_THRESHOLD_MV                 0U
#define BATTERY_RECOVERY_THRESHOLD_MV                 0U

#if BATTERY_PROTECTION_ENABLED
#if (BATTERY_CRITICAL_THRESHOLD_MV == 0U) || \
    (BATTERY_LOW_THRESHOLD_MV <= BATTERY_CRITICAL_THRESHOLD_MV) || \
    (BATTERY_RECOVERY_THRESHOLD_MV <= BATTERY_LOW_THRESHOLD_MV)
#error "Battery protection thresholds must satisfy critical < low < recovery"
#endif
#endif

#endif /* APPLICATION_POWER_BATTERY_CONFIG_H_ */
