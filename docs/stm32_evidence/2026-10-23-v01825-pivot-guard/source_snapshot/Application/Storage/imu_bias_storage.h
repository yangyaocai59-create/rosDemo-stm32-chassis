/**
 * @file imu_bias_storage.h
 * @brief Power-loss-safe persistence for the current IMU gyro bias.
 */

#ifndef APPLICATION_STORAGE_IMU_BIAS_STORAGE_H_
#define APPLICATION_STORAGE_IMU_BIAS_STORAGE_H_

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Scan the two reserved Flash journals. This function never erases Flash. */
void ImuBiasStorage_Init(void);

/** Compatibility view, truncated to integer RAW. Do not use for boot restore. */
bool ImuBiasStorage_Load(int32_t *x_raw,
                         int32_t *y_raw,
                         int32_t *z_raw,
                         int16_t *calibration_temp_centi_c);

/** Exact canonical values from format 3. Old integer format records are
 * rejected, never reinterpreted in different units or silently migrated. */
bool ImuBiasStorage_LoadMilliRaw(int32_t *x_milli_raw,
                                  int32_t *y_milli_raw,
                                  int32_t *z_milli_raw,
                                  int16_t *calibration_temp_centi_c);

/**
 * Append one committed record and verify it by readback.
 *
 * The previous record remains valid if programming or verification fails.
 * Runtime sector erasure is deliberately not performed; the two journals
 * provide more than 6500 manual calibration records before maintenance is
 * required.
 */
bool ImuBiasStorage_Save(int32_t x_raw,
                         int32_t y_raw,
                         int32_t z_raw,
                         int16_t calibration_temp_centi_c);

/** Append/verify exact milli-RAW correction, commit word last; no erase. */
bool ImuBiasStorage_SaveMilliRaw(int32_t x_milli_raw,
                                  int32_t y_milli_raw,
                                  int32_t z_milli_raw,
                                  int16_t calibration_temp_centi_c);

#ifdef __cplusplus
}
#endif

#endif /* APPLICATION_STORAGE_IMU_BIAS_STORAGE_H_ */
