/**
 * @file imu_service.c
 * @brief Configured IMU backend startup, polling and automatic recovery.
 */

#include "imu_service.h"

#include <string.h>

#include "FreeRTOS.h"
#include "debug_log.h"
#include "imu_config.h"
#include "imu_bias_units.h"
#include "task.h"

#if IMU_USE_MPU6500_SPI
#include "mpu6500.h"
#include "stm32f4xx_hal.h"
typedef Mpu6500RawSample ImuDeviceRawSample;
#define ImuDevice_ReadWhoAmI Mpu6500_ReadWhoAmI
#define ImuDevice_BeginReset Mpu6500_BeginReset
#define ImuDevice_Configure Mpu6500_Configure
#define ImuDevice_VerifyConfiguration Mpu6500_VerifyConfiguration
#define ImuDevice_ReadRaw Mpu6500_ReadRaw
#define IMU_DEVICE_EXPECTED_ID MPU6500_WHO_AM_I_EXPECTED
#define IMU_DEVICE_TEXT "MPU6500 SPI1"
static uint32_t s_configuration_deadline_ms;
static TaskHandle_t volatile s_imu_task;
static volatile uint32_t s_data_ready_count;
static volatile uint32_t s_data_ready_ms;
static uint32_t s_consumed_ready_count;
static uint32_t s_last_ready_ms;
#else
#include "mpu6050.h"
typedef Mpu6050RawSample ImuDeviceRawSample;
#define ImuDevice_ReadWhoAmI Mpu6050_ReadWhoAmI
#define ImuDevice_BeginReset Mpu6050_BeginReset
#define ImuDevice_Configure Mpu6050_Configure
#define ImuDevice_VerifyConfiguration Mpu6050_VerifyConfiguration
#define ImuDevice_ReadRaw Mpu6050_ReadRaw
#define IMU_DEVICE_EXPECTED_ID MPU6050_WHO_AM_I_EXPECTED
#define IMU_DEVICE_TEXT "MPU6050 I2C1"
#endif

typedef enum
{
  IMU_PHASE_RETRY_WAIT = 0,
  IMU_PHASE_WAIT_RESET,
  IMU_PHASE_WAIT_WAKE,
  IMU_PHASE_READY
} ImuPrivatePhase;

static ImuServiceSnapshot s_snapshot;
static ImuPrivatePhase s_phase;
static uint32_t s_deadline_ms;
static int32_t s_gyro_bias_milli_raw[3];
static bool s_gyro_bias_valid;

void ImuService_OnDataReadyFromISR(uint32_t timestamp_ms)
{
#if IMU_USE_MPU6500_SPI
  /* Single writer (EXTI0). Readers copy both words with EXTI0 masked by the
   * FreeRTOS task critical section; EXTI0 priority is already 6 in CubeMX. */
  s_data_ready_ms = timestamp_ms;
  ++s_data_ready_count;
  if (s_imu_task != NULL)
  {
    BaseType_t higher_priority_woken = pdFALSE;
    vTaskNotifyGiveFromISR(s_imu_task, &higher_priority_woken);
    portYIELD_FROM_ISR(higher_priority_woken);
  }
#else
  (void)timestamp_ms;
#endif
}

static int16_t ImuService_MapAxis(const int16_t source[3],
                                  uint32_t source_axis,
                                  int32_t source_sign)
{
  return (int16_t)((int32_t)source[source_axis] * source_sign);
}

#if IMU_CALIBRATION_ENABLED
static bool ImuService_IsBiasValid(int32_t value)
{
  return (value >= -IMU_GYRO_BIAS_ABS_MAX_RAW * IMU_BIAS_MILLI_RAW_PER_RAW) &&
         (value <= IMU_GYRO_BIAS_ABS_MAX_RAW * IMU_BIAS_MILLI_RAW_PER_RAW);
}
#endif

static float ImuService_GyroRawToRadS(int16_t raw, int32_t bias_milli_raw)
{
  /* Subtract in integer milli-RAW before conversion, preserving fine static
   * residuals without float cancellation between large raw/baseline values. */
  return ((float)((int32_t)raw * IMU_BIAS_MILLI_RAW_PER_RAW - bias_milli_raw) /
          (IMU_BIAS_MILLI_RAW_PER_RAW * IMU_GYRO_LSB_PER_DEGREE_PER_SECOND)) *
         IMU_DEGREES_TO_RADIANS;
}

static bool ImuService_TimeReached(uint32_t now_ms, uint32_t deadline_ms)
{
  return ((int32_t)(now_ms - deadline_ms) >= 0);
}

static void ImuService_SetRetry(ImuServiceState state, uint32_t now_ms)
{
  taskENTER_CRITICAL();
  s_snapshot.state = state;
  s_snapshot.valid = false;
  taskEXIT_CRITICAL();

  s_phase = IMU_PHASE_RETRY_WAIT;
  s_deadline_ms = now_ms + IMU_RECOVERY_RETRY_MS;
}

static bool ImuService_StartResetSequence(uint32_t now_ms)
{
  uint8_t who_am_i = 0U;

  if (!ImuDevice_ReadWhoAmI(&who_am_i) ||
      (who_am_i != IMU_DEVICE_EXPECTED_ID))
  {
    taskENTER_CRITICAL();
    s_snapshot.who_am_i = who_am_i;
    taskEXIT_CRITICAL();
    ImuService_SetRetry(IMU_SERVICE_STATE_DISCONNECTED, now_ms);
    return false;
  }

  if (!ImuDevice_BeginReset())
  {
    ImuService_SetRetry(IMU_SERVICE_STATE_IO_FAULT, now_ms);
    return false;
  }

  taskENTER_CRITICAL();
  s_snapshot.who_am_i = who_am_i;
  s_snapshot.state = IMU_SERVICE_STATE_INITIALIZING;
  s_snapshot.valid = false;
  taskEXIT_CRITICAL();

  s_phase = IMU_PHASE_WAIT_RESET;
  s_deadline_ms = now_ms + IMU_RESET_SETTLE_MS;
  return true;
}

void ImuService_Init(uint32_t now_ms)
{
#if IMU_USE_MPU6500_SPI
  taskENTER_CRITICAL();
  s_imu_task = xTaskGetCurrentTaskHandle();
  s_data_ready_count = 0U;
  s_data_ready_ms = now_ms;
  s_consumed_ready_count = 0U;
  s_last_ready_ms = now_ms;
  taskEXIT_CRITICAL();
#endif
  memset(&s_snapshot, 0, sizeof(s_snapshot));
  memset(s_gyro_bias_milli_raw, 0, sizeof(s_gyro_bias_milli_raw));
  s_gyro_bias_valid = false;
  s_snapshot.initialized = true;
  s_snapshot.state = IMU_SERVICE_STATE_NOT_READY;
  s_phase = IMU_PHASE_RETRY_WAIT;
  s_deadline_ms = now_ms + IMU_STARTUP_SETTLE_MS;

#if IMU_USE_MPU6500_SPI
#if IMU_CALIBRATION_ENABLED
  DEBUG_LOG_INFO("MPU6500 SPI startup: 100Hz PB0 DRDY; explicit calibration, Flash restore enabled");
#else
  DEBUG_LOG_INFO("MPU6500 SPI startup: 100Hz PB0 DRDY; calibration/Flash restore disabled");
#endif
#else
  if (!ImuService_StartResetSequence(now_ms))
  {
    DEBUG_LOG_WARN(
      "MPU6050 not detected on I2C1 address 0x%02X; retrying without blocking system",
      (unsigned int)IMU_MPU6050_I2C_ADDRESS_7BIT);
  }
#endif
}

void ImuService_Update(uint32_t now_ms)
{
  ImuDeviceRawSample raw;
  int16_t sensor_accel[3];
  int16_t sensor_gyro[3];
  int16_t body_accel[3];
  int16_t body_gyro[3];
  int32_t gyro_bias[3];
  bool gyro_bias_valid;
  float accel_m_s2[3];
  float angular_velocity_rad_s[3];
  float temperature_c;

  if (!s_snapshot.initialized)
  {
    return;
  }

  if (s_phase == IMU_PHASE_RETRY_WAIT)
  {
    if (ImuService_TimeReached(now_ms, s_deadline_ms))
    {
      (void)ImuService_StartResetSequence(now_ms);
    }
    return;
  }

  if (s_phase == IMU_PHASE_WAIT_RESET)
  {
    if (!ImuService_TimeReached(now_ms, s_deadline_ms))
    {
      return;
    }

    if (!ImuDevice_Configure())
    {
      ImuService_SetRetry(IMU_SERVICE_STATE_IO_FAULT, now_ms);
      return;
    }
    s_phase = IMU_PHASE_WAIT_WAKE;
    s_deadline_ms = now_ms + IMU_WAKE_SETTLE_MS;
    return;
  }

  if (s_phase == IMU_PHASE_WAIT_WAKE)
  {
    if (!ImuService_TimeReached(now_ms, s_deadline_ms))
    {
      return;
    }

    if (!ImuDevice_VerifyConfiguration())
    {
      ImuService_SetRetry(IMU_SERVICE_STATE_IO_FAULT, now_ms);
      return;
    }

    taskENTER_CRITICAL();
    s_snapshot.state = IMU_SERVICE_STATE_READY;
    s_snapshot.consecutive_errors = 0U;
    ++s_snapshot.acquisition_epoch;
#if IMU_USE_MPU6500_SPI
    /* Ignore edges accumulated during reset/configuration settling. */
    s_consumed_ready_count = s_data_ready_count;
    s_last_ready_ms = now_ms;
#endif
    taskEXIT_CRITICAL();
    s_phase = IMU_PHASE_READY;
#if IMU_USE_MPU6500_SPI
    s_configuration_deadline_ms = now_ms + IMU_CONFIGURATION_CHECK_MS;
    DEBUG_LOG_INFO("MPU6500 ready: WHO_AM_I=0x%02X SPI1 100Hz +/-4g +/-500dps DLPF=41Hz PB0 DRDY enabled",
                   (unsigned int)s_snapshot.who_am_i);
#else
    DEBUG_LOG_INFO(
      "MPU6050 ready: WHO_AM_I=0x%02X I2C1/0x%02X 100Hz +/-4g +/-500dps polling",
      (unsigned int)s_snapshot.who_am_i,
      (unsigned int)IMU_MPU6050_I2C_ADDRESS_7BIT);
#endif
    return;
  }

#if IMU_USE_MPU6500_SPI
  if (ImuService_TimeReached(now_ms, s_configuration_deadline_ms))
  {
    s_configuration_deadline_ms = now_ms + IMU_CONFIGURATION_CHECK_MS;
    if (!Mpu6500_VerifyConfiguration())
    {
      taskENTER_CRITICAL();
      ++s_snapshot.read_error_count;
      ++s_snapshot.consecutive_errors;
      taskEXIT_CRITICAL();
      ImuService_SetRetry(IMU_SERVICE_STATE_IO_FAULT, now_ms);
      DEBUG_LOG_ERROR("MPU6500 configuration/identity check failed; sample invalid, retrying");
      return;
    }
  }
#endif
#if IMU_USE_MPU6500_SPI
  uint32_t ready_count, ready_ms;
  taskENTER_CRITICAL();
  ready_count = s_data_ready_count;
  ready_ms = s_data_ready_ms;
  taskEXIT_CRITICAL();
  /* Sample the clock AFTER copying the ISR mailbox. Caller now_ms can be
   * older than a just-arrived edge (the same race fixed in USB SOF handling). */
  const uint32_t observed_ms = HAL_GetTick();
  if (ready_count == s_consumed_ready_count)
  {
    if ((uint32_t)(observed_ms - s_last_ready_ms) >= IMU_DRDY_TIMEOUT_MS)
    {
      taskENTER_CRITICAL();
      /* An edge can arrive after the mailbox copy but before this timeout
       * decision. Leave it pending instead of falsely entering recovery. */
      if (s_data_ready_count != ready_count)
      {
        taskEXIT_CRITICAL();
        return;
      }
      ++s_snapshot.data_ready_timeouts;
      ++s_snapshot.read_error_count;
      ++s_snapshot.consecutive_errors;
      taskEXIT_CRITICAL();
      ImuService_SetRetry(IMU_SERVICE_STATE_IO_FAULT, observed_ms);
      DEBUG_LOG_ERROR("MPU6500 DRDY timeout: check INT -> PB0; sample invalid, retrying");
    }
    return;
  }
  const uint32_t ready_delta = ready_count - s_consumed_ready_count;
  s_consumed_ready_count = ready_count;
  s_last_ready_ms = ready_ms;
  taskENTER_CRITICAL();
  s_snapshot.missed_samples += ready_delta - 1U;
  taskEXIT_CRITICAL();
  if ((uint32_t)(observed_ms - ready_ms) > IMU_DRDY_MAX_AGE_MS)
  {
    taskENTER_CRITICAL();
    ++s_snapshot.missed_samples;
    s_snapshot.valid = false;
    taskEXIT_CRITICAL();
    return;
  }
#endif
  if (ImuDevice_ReadRaw(&raw))
  {
#if IMU_USE_MPU6500_SPI
    taskENTER_CRITICAL();
    const bool raced = (ready_count != s_data_ready_count);
    if (raced)
    {
      ++s_snapshot.read_races;
      s_snapshot.valid = false;
    }
    taskEXIT_CRITICAL();
    /* A register update overlapped the transaction. Do not assign an old
     * timestamp to possibly newer data; the next pending edge is retried. */
    if (raced) { return; }
#endif
    sensor_accel[0] = raw.accel_x;
    sensor_accel[1] = raw.accel_y;
    sensor_accel[2] = raw.accel_z;
    sensor_gyro[0] = raw.gyro_x;
    sensor_gyro[1] = raw.gyro_y;
    sensor_gyro[2] = raw.gyro_z;

    body_accel[0] = ImuService_MapAxis(sensor_accel,
                                       IMU_BODY_X_SOURCE_AXIS,
                                       IMU_BODY_X_SOURCE_SIGN);
    body_accel[1] = ImuService_MapAxis(sensor_accel,
                                       IMU_BODY_Y_SOURCE_AXIS,
                                       IMU_BODY_Y_SOURCE_SIGN);
    body_accel[2] = ImuService_MapAxis(sensor_accel,
                                       IMU_BODY_Z_SOURCE_AXIS,
                                       IMU_BODY_Z_SOURCE_SIGN);
    body_gyro[0] = ImuService_MapAxis(sensor_gyro,
                                      IMU_BODY_X_SOURCE_AXIS,
                                      IMU_BODY_X_SOURCE_SIGN);
    body_gyro[1] = ImuService_MapAxis(sensor_gyro,
                                      IMU_BODY_Y_SOURCE_AXIS,
                                      IMU_BODY_Y_SOURCE_SIGN);
    body_gyro[2] = ImuService_MapAxis(sensor_gyro,
                                      IMU_BODY_Z_SOURCE_AXIS,
                                      IMU_BODY_Z_SOURCE_SIGN);

    taskENTER_CRITICAL();
    gyro_bias_valid = s_gyro_bias_valid;
    gyro_bias[0] = s_gyro_bias_milli_raw[0];
    gyro_bias[1] = s_gyro_bias_milli_raw[1];
    gyro_bias[2] = s_gyro_bias_milli_raw[2];
    taskEXIT_CRITICAL();

    for (uint32_t axis = 0U; axis < 3U; ++axis)
    {
      accel_m_s2[axis] =
        ((float)body_accel[axis] / IMU_ACCEL_LSB_PER_G) *
        IMU_STANDARD_GRAVITY_M_S2;
      angular_velocity_rad_s[axis] = ImuService_GyroRawToRadS(
        body_gyro[axis], gyro_bias_valid ? gyro_bias[axis] : 0L);
    }
    temperature_c = ImuService_TemperatureRawToC(raw.temperature);

    taskENTER_CRITICAL();
    s_snapshot.accel_x_raw = body_accel[0];
    s_snapshot.accel_y_raw = body_accel[1];
    s_snapshot.accel_z_raw = body_accel[2];
    s_snapshot.temperature_raw = raw.temperature;
    s_snapshot.gyro_x_raw = body_gyro[0];
    s_snapshot.gyro_y_raw = body_gyro[1];
    s_snapshot.gyro_z_raw = body_gyro[2];
    s_snapshot.gyro_bias_valid = gyro_bias_valid;
    s_snapshot.gyro_bias_x_raw = gyro_bias[0] / IMU_BIAS_MILLI_RAW_PER_RAW;
    s_snapshot.gyro_bias_y_raw = gyro_bias[1] / IMU_BIAS_MILLI_RAW_PER_RAW;
    s_snapshot.gyro_bias_z_raw = gyro_bias[2] / IMU_BIAS_MILLI_RAW_PER_RAW;
    s_snapshot.gyro_bias_x_milli_raw = gyro_bias[0];
    s_snapshot.gyro_bias_y_milli_raw = gyro_bias[1];
    s_snapshot.gyro_bias_z_milli_raw = gyro_bias[2];
    s_snapshot.accel_x_m_s2 = accel_m_s2[0];
    s_snapshot.accel_y_m_s2 = accel_m_s2[1];
    s_snapshot.accel_z_m_s2 = accel_m_s2[2];
    s_snapshot.angular_velocity_x_rad_s = angular_velocity_rad_s[0];
    s_snapshot.angular_velocity_y_rad_s = angular_velocity_rad_s[1];
    s_snapshot.angular_velocity_z_rad_s = angular_velocity_rad_s[2];
    s_snapshot.temperature_c = temperature_c;
#if IMU_USE_MPU6500_SPI
    s_snapshot.sample_time_ms = ready_ms;
#else
    s_snapshot.sample_time_ms = now_ms;
#endif
    ++s_snapshot.sample_count;
    s_snapshot.consecutive_errors = 0U;
    s_snapshot.valid = true;
    s_snapshot.state = IMU_SERVICE_STATE_READY;
    taskEXIT_CRITICAL();
    return;
  }

  taskENTER_CRITICAL();
  s_snapshot.valid = false;
  ++s_snapshot.read_error_count;
  ++s_snapshot.consecutive_errors;
  const uint32_t consecutive_errors = s_snapshot.consecutive_errors;
  taskEXIT_CRITICAL();

  if (consecutive_errors >= IMU_CONSECUTIVE_ERROR_LIMIT)
  {
    DEBUG_LOG_ERROR(
      IMU_DEVICE_TEXT " read failed %lu consecutive times; entering recovery",
      (unsigned long)consecutive_errors);
    ImuService_SetRetry(IMU_SERVICE_STATE_IO_FAULT, now_ms);
  }
}

float ImuService_TemperatureRawToC(int16_t raw)
{
#if IMU_USE_MPU6500_SPI
  return Mpu6500_TemperatureC(raw);
#else
  return ((float)raw / 340.0f) + 36.53f;
#endif
}

bool ImuService_GetSnapshot(ImuServiceSnapshot *snapshot)
{
  if (snapshot == NULL)
  {
    return false;
  }

  taskENTER_CRITICAL();
  *snapshot = s_snapshot;
#if IMU_USE_MPU6500_SPI
  snapshot->data_ready_count = s_data_ready_count;
#endif
  taskEXIT_CRITICAL();
  return true;
}

bool ImuService_SetGyroBiasRaw(int32_t x_raw,
                               int32_t y_raw,
                               int32_t z_raw)
{
  /* Compatibility API; check before multiplying externally supplied ints. */
  if ((x_raw < -IMU_GYRO_BIAS_ABS_MAX_RAW) || (x_raw > IMU_GYRO_BIAS_ABS_MAX_RAW) ||
      (y_raw < -IMU_GYRO_BIAS_ABS_MAX_RAW) || (y_raw > IMU_GYRO_BIAS_ABS_MAX_RAW) ||
      (z_raw < -IMU_GYRO_BIAS_ABS_MAX_RAW) || (z_raw > IMU_GYRO_BIAS_ABS_MAX_RAW))
  {
    return false;
  }
  return ImuService_SetGyroBiasMilliRaw(x_raw * IMU_BIAS_MILLI_RAW_PER_RAW,
                                        y_raw * IMU_BIAS_MILLI_RAW_PER_RAW,
                                        z_raw * IMU_BIAS_MILLI_RAW_PER_RAW);
}

bool ImuService_SetGyroBiasMilliRaw(int32_t x_milli_raw,
                                    int32_t y_milli_raw,
                                    int32_t z_milli_raw)
{
#if !IMU_CALIBRATION_ENABLED
  (void)x_milli_raw;
  (void)y_milli_raw;
  (void)z_milli_raw;
  return false;
#else
  if ((!ImuService_IsBiasValid(x_milli_raw)) ||
      (!ImuService_IsBiasValid(y_milli_raw)) ||
      (!ImuService_IsBiasValid(z_milli_raw)))
  {
    return false;
  }

  taskENTER_CRITICAL();
  s_gyro_bias_milli_raw[0] = x_milli_raw;
  s_gyro_bias_milli_raw[1] = y_milli_raw;
  s_gyro_bias_milli_raw[2] = z_milli_raw;
  s_gyro_bias_valid = true;
  s_snapshot.gyro_bias_x_raw = x_milli_raw / IMU_BIAS_MILLI_RAW_PER_RAW;
  s_snapshot.gyro_bias_y_raw = y_milli_raw / IMU_BIAS_MILLI_RAW_PER_RAW;
  s_snapshot.gyro_bias_z_raw = z_milli_raw / IMU_BIAS_MILLI_RAW_PER_RAW;
  s_snapshot.gyro_bias_x_milli_raw = x_milli_raw;
  s_snapshot.gyro_bias_y_milli_raw = y_milli_raw;
  s_snapshot.gyro_bias_z_milli_raw = z_milli_raw;
  s_snapshot.gyro_bias_valid = true;
  s_snapshot.angular_velocity_x_rad_s =
    ImuService_GyroRawToRadS(s_snapshot.gyro_x_raw, x_milli_raw);
  s_snapshot.angular_velocity_y_rad_s =
    ImuService_GyroRawToRadS(s_snapshot.gyro_y_raw, y_milli_raw);
  s_snapshot.angular_velocity_z_rad_s =
    ImuService_GyroRawToRadS(s_snapshot.gyro_z_raw, z_milli_raw);
  taskEXIT_CRITICAL();
  return true;
#endif
}

void ImuService_ClearGyroBias(void)
{
  taskENTER_CRITICAL();
  memset(s_gyro_bias_milli_raw, 0, sizeof(s_gyro_bias_milli_raw));
  s_gyro_bias_valid = false;
  s_snapshot.gyro_bias_x_raw = 0L;
  s_snapshot.gyro_bias_y_raw = 0L;
  s_snapshot.gyro_bias_z_raw = 0L;
  s_snapshot.gyro_bias_x_milli_raw = 0L;
  s_snapshot.gyro_bias_y_milli_raw = 0L;
  s_snapshot.gyro_bias_z_milli_raw = 0L;
  s_snapshot.gyro_bias_valid = false;
  s_snapshot.angular_velocity_x_rad_s =
    ImuService_GyroRawToRadS(s_snapshot.gyro_x_raw, 0L);
  s_snapshot.angular_velocity_y_rad_s =
    ImuService_GyroRawToRadS(s_snapshot.gyro_y_raw, 0L);
  s_snapshot.angular_velocity_z_rad_s =
    ImuService_GyroRawToRadS(s_snapshot.gyro_z_raw, 0L);
  taskEXIT_CRITICAL();
}

const char *ImuService_StateToString(ImuServiceState state)
{
  switch (state)
  {
    case IMU_SERVICE_STATE_NOT_READY:
      return "NOT_READY";
    case IMU_SERVICE_STATE_INITIALIZING:
      return "INITIALIZING";
    case IMU_SERVICE_STATE_READY:
      return "READY";
    case IMU_SERVICE_STATE_DISCONNECTED:
      return "DISCONNECTED";
    case IMU_SERVICE_STATE_IO_FAULT:
      return "IO_FAULT";
    default:
      return "UNKNOWN";
  }
}
