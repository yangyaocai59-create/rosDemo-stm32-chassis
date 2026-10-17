/**
 * @file imu_config.h
 * @brief IMU service configuration independent of CubeMX generated files.
 */

#ifndef APPLICATION_SENSORS_IMU_CONFIG_H_
#define APPLICATION_SENSORS_IMU_CONFIG_H_

/* Mutually exclusive backends. Current stage: MPU6500 SPI with calibration.
 * Override both to 0 only when deliberately compiling the legacy backend. */
#ifndef IMU_SPI_ID_PROBE_ONLY
#define IMU_SPI_ID_PROBE_ONLY             0
#endif
#ifndef IMU_USE_MPU6500_SPI
#define IMU_USE_MPU6500_SPI               (!IMU_SPI_ID_PROBE_ONLY)
#endif
#if (IMU_SPI_ID_PROBE_ONLY != 0) && (IMU_SPI_ID_PROBE_ONLY != 1)
#error "IMU_SPI_ID_PROBE_ONLY must be 0 or 1"
#endif
#if (IMU_USE_MPU6500_SPI != 0) && (IMU_USE_MPU6500_SPI != 1)
#error "IMU_USE_MPU6500_SPI must be 0 or 1"
#endif
#if IMU_SPI_ID_PROBE_ONLY && IMU_USE_MPU6500_SPI
#error "Identity-only and MPU6500 polling modes are mutually exclusive"
#endif
#ifndef IMU_CALIBRATION_ENABLED
#define IMU_CALIBRATION_ENABLED (!IMU_SPI_ID_PROBE_ONLY)
#endif
#if (IMU_CALIBRATION_ENABLED != 0) && (IMU_CALIBRATION_ENABLED != 1)
#error "IMU_CALIBRATION_ENABLED must be 0 or 1"
#endif
#if IMU_SPI_ID_PROBE_ONLY && IMU_CALIBRATION_ENABLED
#error "Identity-only probe must not calibrate or write Flash"
#endif

/* Preserved legacy backend: GY-521/MPU6050 with AD0 tied low. */
#define IMU_MPU6050_I2C_ADDRESS_7BIT       0x68U

/*
 * Installed sensor frame is identical to ROS base_link:
 * +X forward, +Y left, +Z up. Source indices are X=0, Y=1, Z=2.
 */
#define IMU_FRAME_MAPPING_TEXT              \
  "sensor +X=forward +Y=left +Z=up (ROS base_link identity)"
#define IMU_BODY_X_SOURCE_AXIS              0U
#define IMU_BODY_Y_SOURCE_AXIS              1U
#define IMU_BODY_Z_SOURCE_AXIS              2U
#define IMU_BODY_X_SOURCE_SIGN              (+1)
#define IMU_BODY_Y_SOURCE_SIGN              (+1)
#define IMU_BODY_Z_SOURCE_SIGN              (+1)

/* Both configured backends use +/-4g and +/-500dps. */
#define IMU_ACCEL_LSB_PER_G                  8192.0f
#define IMU_GYRO_LSB_PER_DEGREE_PER_SECOND  65.5f
#define IMU_STANDARD_GRAVITY_M_S2            9.80665f
#define IMU_DEGREES_TO_RADIANS               0.01745329252f
#define IMU_GYRO_BIAS_ABS_MAX_RAW           1000L

/* MPU6500: PB0 DRDY wakes imuTask; SPI transfers remain blocking, no DMA.
 * Legacy MPU6050 still uses periodic polling. All limits are milliseconds. */
#define IMU_SAMPLE_PERIOD_MS               10U
#define IMU_DRDY_TIMEOUT_MS                35U
#define IMU_DRDY_MAX_AGE_MS                 15U
#define IMU_VALID_LOG_PERIOD_MS            1000U
#define IMU_INVALID_LOG_PERIOD_MS          5000U
#define IMU_RESET_SETTLE_MS                100U
#if IMU_USE_MPU6500_SPI
#define IMU_STARTUP_SETTLE_MS             100U
#define IMU_WAKE_SETTLE_MS                100U
#else
#define IMU_STARTUP_SETTLE_MS               0U
#define IMU_WAKE_SETTLE_MS                 10U
#endif
#define IMU_CONFIGURATION_CHECK_MS        1000U
#define IMU_RECOVERY_RETRY_MS              1000U
#define IMU_CONSECUTIVE_ERROR_LIMIT         3U

/* Debug/USB stationary baseline collection. Successful values are persisted. */
#define IMU_STATIC_TEST_SAMPLE_COUNT       500U
#define IMU_STATIC_TEST_MAX_GAP_MS          25U
#define IMU_STATIC_TEST_MIN_INTERVAL_MS      5U
#define IMU_STATIC_TEST_MAX_INTERVAL_MS     15U
#define IMU_STATIC_TEST_MAX_DURATION_MS   6000U
#define IMU_STATIC_TEST_GYRO_STD_RAW        12.0f
#define IMU_STATIC_TEST_ACCEL_STD_RAW       64.0f

#endif /* APPLICATION_SENSORS_IMU_CONFIG_H_ */
