/**
 * @file imu_static_test.c
 * @brief Five-second raw IMU stationary statistics and validated bias save.
 */

#include "imu_static_test.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

#include "FreeRTOS.h"
#include "debug_log.h"
#include "imu_bias_storage.h"
#include "imu_config.h"
#include "imu_bias_units.h"
#include "imu_calibration_guard.h"
#include "stm32f4xx_hal.h"
#include "task.h"

#define IMU_STATIC_TEST_AXIS_COUNT           3U
#define IMU_STATIC_TEST_ACCEL_1G_RAW          8192L
#define IMU_STATIC_TEST_ACCEL_RANGE_LIMIT_RAW 500L
#define IMU_STATIC_TEST_GYRO_RANGE_LIMIT_RAW  96L
#define IMU_STATIC_TEST_GRAVITY_MIN_RAW       7373UL
#define IMU_STATIC_TEST_GRAVITY_MAX_RAW       9011UL

typedef struct
{
  uint32_t generation;
  uint32_t count;
  uint32_t start_ms;
  uint32_t last_ms;
  uint32_t last_sample_count;
  uint32_t last_sample_ms;
  uint32_t initial_errors;
  uint32_t initial_epoch;
  uint32_t initial_missed;
  uint32_t initial_races;
  int64_t wheel_counts[4];
  int64_t accel_sum[IMU_STATIC_TEST_AXIS_COUNT];
  int64_t gyro_sum[IMU_STATIC_TEST_AXIS_COUNT];
  int64_t accel_sum_squared[IMU_STATIC_TEST_AXIS_COUNT];
  int64_t gyro_sum_squared[IMU_STATIC_TEST_AXIS_COUNT];
  int64_t temperature_sum;
  int16_t accel_min[IMU_STATIC_TEST_AXIS_COUNT];
  int16_t accel_max[IMU_STATIC_TEST_AXIS_COUNT];
  int16_t gyro_min[IMU_STATIC_TEST_AXIS_COUNT];
  int16_t gyro_max[IMU_STATIC_TEST_AXIS_COUNT];
} ImuStaticAccumulator;

static ImuStaticAccumulator s_accumulator;
static ImuStaticTestStatus s_status;
static bool s_completion_pending;
static bool s_persistence_active;
static uint32_t s_generation;

static void ImuStaticTest_Fail(ImuStaticResult reason, const char *description,
                              uint32_t generation)
{
  bool failed = false;
  taskENTER_CRITICAL();
  if ((s_status.result == IMU_STATIC_RESULT_RUNNING) &&
      (s_accumulator.generation == generation))
  {
    s_status.result = reason;
    s_completion_pending = false;
    failed = true;
  }
  taskEXIT_CRITICAL();
  if (failed)
  {
    DEBUG_LOG_WARN("IMU STATIC aborted: %s; previous bias unchanged", description);
  }
}

static float ImuStaticTest_Std(int64_t sum, int64_t squares, uint32_t count)
{
  /* Exact integer numerator avoids float cancellation near 1g or large DC
   * gyro bias. Bounds: 500 * int16 samples -> numerator safely fits int64. */
  const int64_t numerator = (int64_t)count * squares - sum * sum;
  return sqrtf((float)numerator / ((float)count * (float)count));
}

static int32_t ImuStaticTest_Abs(int32_t value)
{
  return (value < 0) ? -value : value;
}

static int32_t ImuStaticTest_RawGyroToMdps(int32_t raw)
{
  /* +/-500 dps sensitivity is 65.5 LSB/(degree/s). */
  return (raw * 10000L) / 655L;
}

static int32_t ImuStaticTest_RawAccelToMg(int32_t raw)
{
  return (raw * 1000L) / IMU_STATIC_TEST_ACCEL_1G_RAW;
}

static void ImuStaticTest_ResetAccumulator(uint32_t now_ms)
{
  memset(&s_accumulator, 0, sizeof(s_accumulator));
  s_accumulator.start_ms = now_ms;

  for (uint32_t axis = 0U; axis < IMU_STATIC_TEST_AXIS_COUNT; ++axis)
  {
    s_accumulator.accel_min[axis] = INT16_MAX;
    s_accumulator.accel_max[axis] = INT16_MIN;
    s_accumulator.gyro_min[axis] = INT16_MAX;
    s_accumulator.gyro_max[axis] = INT16_MIN;
  }
}

static void ImuStaticTest_UpdateRange(int16_t value,
                                     int16_t *minimum,
                                     int16_t *maximum)
{
  if (value < *minimum)
  {
    *minimum = value;
  }
  if (value > *maximum)
  {
    *maximum = value;
  }
}

static void ImuStaticTest_Report(const ImuStaticAccumulator *result)
{
  int32_t accel_mean[IMU_STATIC_TEST_AXIS_COUNT];
  int32_t gyro_mean[IMU_STATIC_TEST_AXIS_COUNT];
  int32_t gyro_mean_milli_raw[IMU_STATIC_TEST_AXIS_COUNT];
  int32_t accel_range[IMU_STATIC_TEST_AXIS_COUNT];
  int32_t gyro_range[IMU_STATIC_TEST_AXIS_COUNT];
  float accel_std[IMU_STATIC_TEST_AXIS_COUNT];
  float gyro_std[IMU_STATIC_TEST_AXIS_COUNT];
  uint32_t dominant_axis = 0U;
  bool stationary = true;
  bool bias_applied = false;
  bool bias_persisted = false;
  ImuStaticResult final_result = IMU_STATIC_RESULT_CANCELLED;

  for (uint32_t axis = 0U; axis < IMU_STATIC_TEST_AXIS_COUNT; ++axis)
  {
    accel_mean[axis] =
      (int32_t)(result->accel_sum[axis] / (int64_t)result->count);
    gyro_mean[axis] =
      (int32_t)(result->gyro_sum[axis] / (int64_t)result->count);
    if (!ImuBias_MeanMilliRaw(result->gyro_sum[axis], result->count,
                              &gyro_mean_milli_raw[axis]))
    {
      ImuStaticTest_Fail(IMU_STATIC_RESULT_FAIL_DATA, "invalid sample sum/count",
                         result->generation);
      return;
    }
    accel_range[axis] = (int32_t)result->accel_max[axis] -
                        (int32_t)result->accel_min[axis];
    gyro_range[axis] = (int32_t)result->gyro_max[axis] -
                       (int32_t)result->gyro_min[axis];
    accel_std[axis] = ImuStaticTest_Std(result->accel_sum[axis],
      result->accel_sum_squared[axis], result->count);
    gyro_std[axis] = ImuStaticTest_Std(result->gyro_sum[axis],
      result->gyro_sum_squared[axis], result->count);

    if (ImuStaticTest_Abs(accel_mean[axis]) >
        ImuStaticTest_Abs(accel_mean[dominant_axis]))
    {
      dominant_axis = axis;
    }
    if ((accel_range[axis] > IMU_STATIC_TEST_ACCEL_RANGE_LIMIT_RAW) ||
        (gyro_range[axis] > IMU_STATIC_TEST_GYRO_RANGE_LIMIT_RAW) ||
        (accel_std[axis] > IMU_STATIC_TEST_ACCEL_STD_RAW) ||
        (gyro_std[axis] > IMU_STATIC_TEST_GYRO_STD_RAW))
    {
      stationary = false;
    }
  }

  const float norm_squared =
    ((float)accel_mean[0] * (float)accel_mean[0]) +
    ((float)accel_mean[1] * (float)accel_mean[1]) +
    ((float)accel_mean[2] * (float)accel_mean[2]);
  const uint32_t gravity_norm_raw = (uint32_t)(sqrtf(norm_squared) + 0.5f);
  const int32_t temperature_raw =
    (int32_t)(result->temperature_sum / (int64_t)result->count);
  const int32_t temperature_centi_c = (int32_t)lroundf(
    ImuService_TemperatureRawToC((int16_t)temperature_raw) * 100.0f);

  if ((gravity_norm_raw < IMU_STATIC_TEST_GRAVITY_MIN_RAW) ||
      (gravity_norm_raw > IMU_STATIC_TEST_GRAVITY_MAX_RAW))
  {
    stationary = false;
  }

  /* Commit the bias and public result as one logical operation. A concurrent
   * STOP/disconnect can win before this critical section and then no bias is
   * applied. ImuService_SetGyroBiasMilliRaw() uses a nestable FreeRTOS critical
   * section internally. */
  taskENTER_CRITICAL();
  if ((s_status.result == IMU_STATIC_RESULT_RUNNING) &&
      (s_accumulator.generation == result->generation))
  {
    /* Recheck immediately before applying a bias; a higher-priority motor
     * task may have updated the read-only guard since the last sample. */
    int64_t wheel_counts[4];
    if (!ImuCalibrationGuard_Read(wheel_counts, true) ||
        memcmp(wheel_counts, result->wheel_counts, sizeof(wheel_counts)) != 0)
    {
      stationary = false;
    }
    if (stationary)
    {
      bias_applied = ImuService_SetGyroBiasMilliRaw(gyro_mean_milli_raw[0],
                                                    gyro_mean_milli_raw[1],
                                                    gyro_mean_milli_raw[2]);
      if (bias_applied)
      {
        ImuService_SetCalibrationTemperatureCentiC(
          (int16_t)temperature_centi_c);
      }
      final_result = bias_applied ? IMU_STATIC_RESULT_PASS :
        IMU_STATIC_RESULT_FAIL_BIAS_RANGE;
      s_persistence_active = bias_applied;
      if (bias_applied) { s_status.persistence = IMU_STATIC_PERSIST_PENDING; }
    }
    else
    {
      final_result = IMU_STATIC_RESULT_FAIL_MOVED;
    }
    s_status.result = final_result;
  }
  s_completion_pending = false;
  taskEXIT_CRITICAL();

  if (final_result == IMU_STATIC_RESULT_CANCELLED)
  {
    DEBUG_LOG_INFO("IMU STATIC completion discarded: calibration was cancelled");
    return;
  }

  DEBUG_LOG_INFO("IMU STATIC result=%s samples=%lu duration=%lums",
                 (stationary && bias_applied) ? "PASS_BIAS_APPLIED" :
                 (stationary ? "RETRY_BIAS_RANGE" : "RETRY_MOVED"),
                 (unsigned long)result->count,
                 (unsigned long)(result->last_ms - result->start_ms));
  DEBUG_LOG_INFO(
    "IMU STATIC accel mean raw=[%ld,%ld,%ld] mg=[%ld,%ld,%ld] norm=%lu",
    (long)accel_mean[0], (long)accel_mean[1], (long)accel_mean[2],
    (long)ImuStaticTest_RawAccelToMg(accel_mean[0]),
    (long)ImuStaticTest_RawAccelToMg(accel_mean[1]),
    (long)ImuStaticTest_RawAccelToMg(accel_mean[2]),
    (unsigned long)gravity_norm_raw);
  DEBUG_LOG_INFO("IMU STATIC accel p2p raw=[%ld,%ld,%ld]",
                 (long)accel_range[0],
                 (long)accel_range[1],
                 (long)accel_range[2]);
  DEBUG_LOG_INFO("IMU STATIC std RAW x100: gyro=[%lu,%lu,%lu] accel=[%lu,%lu,%lu]",
    (unsigned long)lroundf(gyro_std[0] * 100.0f),
    (unsigned long)lroundf(gyro_std[1] * 100.0f),
    (unsigned long)lroundf(gyro_std[2] * 100.0f),
    (unsigned long)lroundf(accel_std[0] * 100.0f),
    (unsigned long)lroundf(accel_std[1] * 100.0f),
    (unsigned long)lroundf(accel_std[2] * 100.0f));
  DEBUG_LOG_INFO(
    "IMU STATIC coarse bias raw=[%ld,%ld,%ld] mdps=[%ld,%ld,%ld]",
    (long)gyro_mean[0], (long)gyro_mean[1], (long)gyro_mean[2],
    (long)ImuStaticTest_RawGyroToMdps(gyro_mean[0]),
    (long)ImuStaticTest_RawGyroToMdps(gyro_mean[1]),
    (long)ImuStaticTest_RawGyroToMdps(gyro_mean[2]));
  DEBUG_LOG_INFO("IMU STATIC gyro p2p raw=[%ld,%ld,%ld] temp=%ld.%02ldC",
                 (long)gyro_range[0],
                 (long)gyro_range[1],
                 (long)gyro_range[2],
                 (long)(temperature_centi_c / 100L),
                 (long)ImuStaticTest_Abs(temperature_centi_c % 100L));
  DEBUG_LOG_INFO("IMU STATIC precise bias milli_raw=[%ld,%ld,%ld] scale=1000",
                 (long)gyro_mean_milli_raw[0], (long)gyro_mean_milli_raw[1],
                 (long)gyro_mean_milli_raw[2]);
  DEBUG_LOG_INFO("IMU STATIC gravity dominant sensor axis=%c%c",
                 (accel_mean[dominant_axis] >= 0) ? '+' : '-',
                 "XYZ"[dominant_axis]);
  if (bias_applied)
  {
    bias_persisted = ImuBiasStorage_SaveMilliRaw(
      gyro_mean_milli_raw[0],
      gyro_mean_milli_raw[1],
      gyro_mean_milli_raw[2],
      (int16_t)temperature_centi_c);
    if (bias_persisted)
    {
      DEBUG_LOG_INFO(
        "IMU STATIC gyro bias applied and persisted for future boots");
    }
    else
    {
      DEBUG_LOG_ERROR(
        "IMU STATIC gyro bias applied in RAM but persistence failed");
    }
    taskENTER_CRITICAL();
    s_status.persistence = bias_persisted ? IMU_STATIC_PERSIST_SAVED :
      IMU_STATIC_PERSIST_FAILED;
    s_persistence_active = false;
    taskEXIT_CRITICAL();
  }
  else
  {
    DEBUG_LOG_WARN(
      "IMU STATIC gyro bias not changed; keep chassis still and retry");
  }
}

void ImuStaticTest_Init(void)
{
  taskENTER_CRITICAL();
  ImuStaticTest_ResetAccumulator(0U);
  s_accumulator.generation = ++s_generation;
  s_status.result = IMU_STATIC_RESULT_NOT_RUN;
  s_status.source = IMU_STATIC_SOURCE_NONE;
  s_status.persistence = IMU_STATIC_PERSIST_NOT_REQUESTED;
  s_status.collected_samples = 0U;
  s_status.duration_ms = 0U;
  s_completion_pending = false;
  s_persistence_active = false;
  taskEXIT_CRITICAL();
}

bool ImuStaticTest_Start(uint32_t now_ms,
                         bool motion_active,
                         ImuStaticSource source)
{
#if !IMU_CALIBRATION_ENABLED
  (void)now_ms;
  (void)motion_active;
  (void)source;
  DEBUG_LOG_WARN("IMU STATIC rejected: calibration/Flash writes disabled in SPI test firmware");
  return false;
#else
  ImuServiceSnapshot snapshot;
  int64_t wheel_counts[4];
  bool started = false;
  now_ms = HAL_GetTick(); /* A USB caller's timestamp may predate its logging. */

  if (motion_active || (source == IMU_STATIC_SOURCE_NONE))
  {
    DEBUG_LOG_WARN("IMU STATIC rejected: motor/chassis diagnostic is active");
    return false;
  }
  if ((!ImuService_GetSnapshot(&snapshot)) || (!snapshot.valid) ||
      (snapshot.state != IMU_SERVICE_STATE_READY) ||
      ((uint32_t)(now_ms - snapshot.sample_time_ms) > IMU_STATIC_TEST_MAX_GAP_MS) ||
      !ImuCalibrationGuard_Read(wheel_counts, false))
  {
    DEBUG_LOG_WARN("IMU STATIC rejected: IMU sample is not ready");
    return false;
  }

  taskENTER_CRITICAL();
  if ((s_status.result != IMU_STATIC_RESULT_RUNNING) &&
      (!s_persistence_active))
  {
    ImuStaticTest_ResetAccumulator(now_ms);
    s_accumulator.generation = ++s_generation;
    s_accumulator.last_sample_count = snapshot.sample_count;
    s_accumulator.last_sample_ms = snapshot.sample_time_ms;
    s_accumulator.last_ms = now_ms;
    s_accumulator.initial_errors = snapshot.read_error_count;
    s_accumulator.initial_epoch = snapshot.acquisition_epoch;
    s_accumulator.initial_missed = snapshot.missed_samples;
    s_accumulator.initial_races = snapshot.read_races;
    memcpy(s_accumulator.wheel_counts, wheel_counts, sizeof(wheel_counts));
    s_status.result = IMU_STATIC_RESULT_RUNNING;
    s_status.source = source;
    s_status.persistence = IMU_STATIC_PERSIST_NOT_REQUESTED;
    s_status.collected_samples = 0U;
    s_status.duration_ms = 0U;
    s_completion_pending = false;
    started = true;
  }
  taskEXIT_CRITICAL();

  if (!started)
  {
    DEBUG_LOG_WARN("IMU STATIC rejected: calibration already running");
    return false;
  }

  DEBUG_LOG_WARN(
    "IMU STATIC started via %s: keep chassis completely still and level for about 5 seconds",
    (source == IMU_STATIC_SOURCE_USB) ? "USB" : "UART");
  return true;
#endif
}

void ImuStaticTest_ProcessSample(const ImuServiceSnapshot *snapshot,
                                 uint32_t now_ms)
{
  bool report_ready = false;
  /* imuTask is the sole sample consumer. Keep the completed-report copy out
   * of its 2KB stack; concurrent Start/Cancel never touch this buffer. */
  static ImuStaticAccumulator result;
  int64_t wheel_counts[4];
  const int16_t accel[IMU_STATIC_TEST_AXIS_COUNT] = {
    (snapshot != NULL) ? snapshot->accel_x_raw : 0,
    (snapshot != NULL) ? snapshot->accel_y_raw : 0,
    (snapshot != NULL) ? snapshot->accel_z_raw : 0
  };
  const int16_t gyro[IMU_STATIC_TEST_AXIS_COUNT] = {
    (snapshot != NULL) ? snapshot->gyro_x_raw : 0,
    (snapshot != NULL) ? snapshot->gyro_y_raw : 0,
    (snapshot != NULL) ? snapshot->gyro_z_raw : 0
  };

  /* Do not concatenate valid fragments around sensor faults or long gaps.
   * Only imuTask calls this function; Start/Cancel may run in other tasks. */
  now_ms = HAL_GetTick();
  taskENTER_CRITICAL();
  const bool running = (s_status.result == IMU_STATIC_RESULT_RUNNING);
  const ImuStaticAccumulator initial = s_accumulator;
  taskEXIT_CRITICAL();
  if (!running) { return; }
  if (!ImuCalibrationGuard_Read(wheel_counts, true) ||
      memcmp(wheel_counts, initial.wheel_counts, sizeof(wheel_counts)) != 0)
  {
    ImuStaticTest_Fail(IMU_STATIC_RESULT_FAIL_MOVED, "motion/interlock changed", initial.generation);
    return;
  }
  if ((snapshot == NULL) || !snapshot->valid ||
      (snapshot->state != IMU_SERVICE_STATE_READY) ||
      (snapshot->read_error_count != initial.initial_errors) ||
      (snapshot->acquisition_epoch != initial.initial_epoch) ||
      (snapshot->missed_samples != initial.initial_missed) ||
      (snapshot->read_races != initial.initial_races) ||
      ((uint32_t)(now_ms - initial.start_ms) > IMU_STATIC_TEST_MAX_DURATION_MS) ||
      ((uint32_t)(now_ms - initial.last_ms) > IMU_STATIC_TEST_MAX_GAP_MS))
  {
    ImuStaticTest_Fail(IMU_STATIC_RESULT_FAIL_DATA, "invalid/stale/discontinuous data", initial.generation);
    return;
  }
  if (snapshot->sample_count == initial.last_sample_count) { return; }
  if (((uint32_t)(snapshot->sample_count - initial.last_sample_count) != 1U) ||
      ((uint32_t)(snapshot->sample_time_ms - initial.last_sample_ms) >
       IMU_STATIC_TEST_MAX_INTERVAL_MS) ||
      ((uint32_t)(snapshot->sample_time_ms - initial.last_sample_ms) <
       IMU_STATIC_TEST_MIN_INTERVAL_MS) ||
      ((uint32_t)(now_ms - snapshot->sample_time_ms) > IMU_STATIC_TEST_MAX_GAP_MS))
  {
    ImuStaticTest_Fail(IMU_STATIC_RESULT_FAIL_DATA, "sample count/time gap", initial.generation);
    return;
  }

  taskENTER_CRITICAL();
  if ((s_status.result == IMU_STATIC_RESULT_RUNNING) &&
      (!s_completion_pending) &&
      (s_accumulator.generation == initial.generation) &&
      (snapshot->sample_count != s_accumulator.last_sample_count))
  {
    s_accumulator.last_sample_count = snapshot->sample_count;
    s_accumulator.last_sample_ms = snapshot->sample_time_ms;
    s_accumulator.last_ms = now_ms;
    ++s_accumulator.count;
    s_accumulator.temperature_sum += snapshot->temperature_raw;
    s_status.collected_samples = s_accumulator.count;
    s_status.duration_ms = now_ms - s_accumulator.start_ms;

    for (uint32_t axis = 0U; axis < IMU_STATIC_TEST_AXIS_COUNT; ++axis)
    {
      s_accumulator.accel_sum[axis] += accel[axis];
      s_accumulator.gyro_sum[axis] += gyro[axis];
      s_accumulator.accel_sum_squared[axis] += (int64_t)accel[axis] * accel[axis];
      s_accumulator.gyro_sum_squared[axis] += (int64_t)gyro[axis] * gyro[axis];
      ImuStaticTest_UpdateRange(accel[axis],
                                &s_accumulator.accel_min[axis],
                                &s_accumulator.accel_max[axis]);
      ImuStaticTest_UpdateRange(gyro[axis],
                                &s_accumulator.gyro_min[axis],
                                &s_accumulator.gyro_max[axis]);
    }

    if (s_accumulator.count >= IMU_STATIC_TEST_SAMPLE_COUNT)
    {
      result = s_accumulator;
      s_completion_pending = true;
      report_ready = true;
    }
  }
  taskEXIT_CRITICAL();

  if (report_ready)
  {
    ImuStaticTest_Report(&result);
  }
}

void ImuStaticTest_Cancel(void)
{
  bool was_active;

  taskENTER_CRITICAL();
  was_active = (s_status.result == IMU_STATIC_RESULT_RUNNING);
  if (was_active)
  {
    s_status.result = IMU_STATIC_RESULT_CANCELLED;
    s_completion_pending = false;
  }
  taskEXIT_CRITICAL();

  if (was_active)
  {
    DEBUG_LOG_INFO("IMU STATIC cancelled; no calibration was changed");
  }
}

bool ImuStaticTest_CancelIfSource(ImuStaticSource source)
{
  bool cancelled = false;

  taskENTER_CRITICAL();
  if ((s_status.result == IMU_STATIC_RESULT_RUNNING) &&
      (s_status.source == source))
  {
    s_status.result = IMU_STATIC_RESULT_CANCELLED;
    s_completion_pending = false;
    cancelled = true;
  }
  taskEXIT_CRITICAL();

  if (cancelled)
  {
    DEBUG_LOG_INFO("IMU STATIC cancelled after USB session ended; bias unchanged");
  }
  return cancelled;
}

bool ImuStaticTest_IsActive(void)
{
  bool active;

  taskENTER_CRITICAL();
  active = (s_status.result == IMU_STATIC_RESULT_RUNNING) ||
           s_persistence_active;
  taskEXIT_CRITICAL();
  return active;
}

bool ImuStaticTest_GetStatus(ImuStaticTestStatus *status)
{
  if (status == NULL)
  {
    return false;
  }

  taskENTER_CRITICAL();
  *status = s_status;
  taskEXIT_CRITICAL();
  return true;
}
