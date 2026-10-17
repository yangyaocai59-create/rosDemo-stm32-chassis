/**
 * @file imu_service.h
 * @brief Sensor-independent IMU lifecycle and coherent raw-data snapshot.
 */

#ifndef APPLICATION_SENSORS_IMU_SERVICE_H_
#define APPLICATION_SENSORS_IMU_SERVICE_H_

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
  IMU_SERVICE_STATE_NOT_READY = 0,
  IMU_SERVICE_STATE_INITIALIZING,
  IMU_SERVICE_STATE_READY,
  IMU_SERVICE_STATE_DISCONNECTED,
  IMU_SERVICE_STATE_IO_FAULT
} ImuServiceState;

typedef struct
{
  bool initialized;
  bool valid;
  ImuServiceState state;
  uint8_t who_am_i;
  int16_t accel_x_raw;
  int16_t accel_y_raw;
  int16_t accel_z_raw;
  int16_t temperature_raw;
  int16_t gyro_x_raw;
  int16_t gyro_y_raw;
  int16_t gyro_z_raw;
  bool gyro_bias_valid;
  int32_t gyro_bias_x_raw;
  int32_t gyro_bias_y_raw;
  int32_t gyro_bias_z_raw;
  /** Canonical applied correction. *_raw above is a truncated compatibility
   * view only; use these milli-RAW values for precise RAW/SI reconstruction. */
  int32_t gyro_bias_x_milli_raw;
  int32_t gyro_bias_y_milli_raw;
  int32_t gyro_bias_z_milli_raw;
  float accel_x_m_s2;
  float accel_y_m_s2;
  float accel_z_m_s2;
  float angular_velocity_x_rad_s;
  float angular_velocity_y_rad_s;
  float angular_velocity_z_rad_s;
  float temperature_c;
  uint32_t sample_time_ms;
  uint32_t sample_count;
  uint32_t read_error_count;
  uint32_t consecutive_errors;
  /** Internal diagnostics; not appended to the legacy/RAW USB layouts. */
  uint32_t acquisition_epoch;
  uint32_t data_ready_count;
  uint32_t missed_samples;
  uint32_t read_races;
  uint32_t data_ready_timeouts;
} ImuServiceSnapshot;

/** Begin non-blocking detection/reset of the configured IMU. */
void ImuService_Init(uint32_t now_ms);

/** PB0 ISR only: timestamp/mailbox + task notification, no SPI/logging. */
void ImuService_OnDataReadyFromISR(uint32_t timestamp_ms);

/** Advance initialization/recovery or acquire one raw sample. */
void ImuService_Update(uint32_t now_ms);

/** Copy a coherent snapshot for telemetry/logging consumers. */
bool ImuService_GetSnapshot(ImuServiceSnapshot *snapshot);

/**
 * Compatibility integer-RAW setter. Apply a validated body-frame gyro bias.
 * Persistence, when requested by an
 * explicit successful calibration, is owned by the storage service.
 * Returns false when calibration is disabled (e.g. identity-only probe).
 */
bool ImuService_SetGyroBiasRaw(int32_t x_raw,
                               int32_t y_raw,
                               int32_t z_raw);

/** Apply exact fixed-point correction; no Flash write or automatic learning. */
bool ImuService_SetGyroBiasMilliRaw(int32_t x_milli_raw,
                                    int32_t y_milli_raw,
                                    int32_t z_milli_raw);

/** Remove the current runtime gyro correction. */
void ImuService_ClearGyroBias(void);

/** Backend-specific die temperature conversion, also used for bias records. */
float ImuService_TemperatureRawToC(int16_t raw);

const char *ImuService_StateToString(ImuServiceState state);

#ifdef __cplusplus
}
#endif

#endif /* APPLICATION_SENSORS_IMU_SERVICE_H_ */
