/**
 * @file wheel_speed_estimator.h
 * @brief Four-wheel velocity estimation from normalized encoder counts.
 */

#ifndef APPLICATION_CHASSIS_WHEEL_SPEED_ESTIMATOR_H_
#define APPLICATION_CHASSIS_WHEEL_SPEED_ESTIMATOR_H_

#include <stdbool.h>
#include <stdint.h>

#include "encoder_service.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
  /** MCU tick at which this position/velocity sample was produced. */
  uint32_t sample_time_ms;
  /** Forward-normalized cumulative position captured for this sample. */
  int64_t accumulated_count;
  /** Encoder increment since the preceding velocity sample. */
  int32_t delta_counts;
  /** Actual interval used for this calculation. */
  uint32_t sample_period_ms;
  /** Unfiltered, forward-positive encoder count rate. */
  float raw_counts_per_second;
  /** First-order low-pass-filtered count rate. */
  float filtered_counts_per_second;
  /** Filtered wheel revolutions per minute. */
  float wheel_rpm;
  /** Filtered wheel angular velocity in radians per second. */
  float wheel_angular_velocity_rad_s;
  /** Filtered wheel tread linear velocity in metres per second. */
  float wheel_linear_velocity_m_s;
  /** True after no encoder movement has been seen for the stop timeout. */
  bool stopped;
  /** False until the first sample with a non-zero time interval is produced. */
  bool valid;
  uint32_t sample_sequence;
  /** Revision 200: interval or implausible count-rate rejection. */
  uint32_t quality_flags;
} WheelSpeedState;

enum { WHEEL_SPEED_QUALITY_INTERVAL=1U, WHEEL_SPEED_QUALITY_COUNT_RATE=2U };

/** Capture the initial encoder positions and reset all velocity estimates. */
bool WheelSpeedEstimator_Init(uint32_t now_ms);

/** Update all four velocity estimates from the latest encoder snapshot. */
void WheelSpeedEstimator_Update(uint32_t now_ms);

/** Copy a coherent LF/LR/RF/RR velocity snapshot. */
bool WheelSpeedEstimator_GetSnapshot(
  WheelSpeedState states[ENCODER_WHEEL_COUNT]);

#ifdef __cplusplus
}
#endif

#endif /* APPLICATION_CHASSIS_WHEEL_SPEED_ESTIMATOR_H_ */
