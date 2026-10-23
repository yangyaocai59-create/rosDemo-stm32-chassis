/** Canonical gyro baseline: signed milli-RAW (1 unit = 0.001 sensor LSB).
 * Integer sums, not floating-point means, define calibration and persistence. */
#ifndef APPLICATION_SENSORS_IMU_BIAS_UNITS_H_
#define APPLICATION_SENSORS_IMU_BIAS_UNITS_H_

#include <stdbool.h>
#include <stdint.h>

#define IMU_BIAS_MILLI_RAW_PER_RAW 1000L

/** Round an int16 sample mean to nearest milli-RAW, ties away from zero.
 * Bound the sum before multiplication; even UINT32_MAX int16 samples fit. */
static inline bool ImuBias_MeanMilliRaw(int64_t sum, uint32_t count,
                                       int32_t *result)
{
  if ((result == 0) || (count == 0U) ||
      (sum < -32768LL * count) || (sum > 32767LL * count))
  {
    return false;
  }
  const int64_t scaled = sum * IMU_BIAS_MILLI_RAW_PER_RAW;
  const int64_t half = (int64_t)count / 2;
  *result = (int32_t)((scaled + ((scaled >= 0) ? half : -half)) / count);
  return true;
}

#endif /* APPLICATION_SENSORS_IMU_BIAS_UNITS_H_ */
