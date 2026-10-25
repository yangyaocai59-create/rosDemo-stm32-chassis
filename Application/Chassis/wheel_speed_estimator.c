/**
 * @file wheel_speed_estimator.c
 * @brief Time-aware wheel velocity calculation and low-pass filtering.
 */

#include "wheel_speed_estimator.h"

#include <limits.h>
#include <stddef.h>

#include "FreeRTOS.h"
#include "chassis_config.h"
#include "chassis_control_config.h"
#include <math.h>
#include "task.h"

static WheelSpeedState s_states[ENCODER_WHEEL_COUNT];
static int64_t s_previous_totals[ENCODER_WHEEL_COUNT];
static uint32_t s_last_motion_ms[ENCODER_WHEEL_COUNT];
static uint32_t s_previous_sample_ms;
static bool s_initialized;
static uint32_t s_sample_sequence;

static int32_t WheelSpeedEstimator_SaturateDelta(int64_t delta)
{
  if (delta > INT32_MAX)
  {
    return INT32_MAX;
  }
  if (delta < INT32_MIN)
  {
    return INT32_MIN;
  }
  return (int32_t)delta;
}

bool WheelSpeedEstimator_Init(uint32_t now_ms)
{
  EncoderState encoder_states[ENCODER_WHEEL_COUNT];

  if (!EncoderService_GetSnapshot(encoder_states))
  {
    return false;
  }

  taskENTER_CRITICAL();
  for (uint32_t wheel = 0U; wheel < ENCODER_WHEEL_COUNT; ++wheel)
  {
    s_states[wheel].sample_time_ms = now_ms;
    s_states[wheel].accumulated_count =
      encoder_states[wheel].accumulated_count;
    s_states[wheel].delta_counts = 0;
    s_states[wheel].sample_period_ms = 0U;
    s_states[wheel].raw_counts_per_second = 0.0f;
    s_states[wheel].filtered_counts_per_second = 0.0f;
    s_states[wheel].wheel_rpm = 0.0f;
    s_states[wheel].wheel_angular_velocity_rad_s = 0.0f;
    s_states[wheel].wheel_linear_velocity_m_s = 0.0f;
    s_states[wheel].stopped = true;
    s_states[wheel].valid = false;
    s_states[wheel].sample_sequence = 0U;
    s_states[wheel].quality_flags = 0U;
    s_previous_totals[wheel] = encoder_states[wheel].accumulated_count;
    s_last_motion_ms[wheel] = now_ms;
  }
  s_previous_sample_ms = now_ms;
  s_sample_sequence = 0U;
  s_initialized = true;
  taskEXIT_CRITICAL();

  return true;
}

void WheelSpeedEstimator_Update(uint32_t now_ms)
{
  EncoderState encoder_states[ENCODER_WHEEL_COUNT];
  WheelSpeedState next_states[ENCODER_WHEEL_COUNT];
  const uint32_t elapsed_ms = now_ms - s_previous_sample_ms;
  float filter_alpha;

  if ((!s_initialized) || (elapsed_ms == 0U) ||
      (!EncoderService_GetSnapshot(encoder_states)))
  {
    return;
  }

  filter_alpha = (float)elapsed_ms /
    ((float)CHASSIS_SPEED_FILTER_TIME_CONSTANT_MS + (float)elapsed_ms);
  const uint32_t sequence=s_sample_sequence+1U;

  for (uint32_t wheel = 0U; wheel < ENCODER_WHEEL_COUNT; ++wheel)
  {
    const int64_t wide_delta =
      encoder_states[wheel].accumulated_count - s_previous_totals[wheel];
    const int32_t delta = WheelSpeedEstimator_SaturateDelta(wide_delta);
    const float raw_counts_per_second =
      ((float)delta * 1000.0f) / (float)elapsed_ms;
    float filtered_counts_per_second =
      s_states[wheel].filtered_counts_per_second +
      filter_alpha *
        (raw_counts_per_second - s_states[wheel].filtered_counts_per_second);
    bool stopped = s_states[wheel].stopped;
    uint32_t quality=0U;
#if CHASSIS_CONTROL_V2
    if (elapsed_ms>CHASSIS_FEEDBACK_MAX_INTERVAL_MS) quality|=WHEEL_SPEED_QUALITY_INTERVAL;
    if (fabsf(raw_counts_per_second)>CHASSIS_ENCODER_MAX_RATE_CPS ||
        wide_delta>INT32_MAX || wide_delta<INT32_MIN) quality|=WHEEL_SPEED_QUALITY_COUNT_RATE;
    if (quality!=0U) filtered_counts_per_second=0;
#endif

    if (delta != 0)
    {
      s_last_motion_ms[wheel] = now_ms;
      stopped = false;
    }
    else if ((now_ms - s_last_motion_ms[wheel]) >=
             CHASSIS_SPEED_STOP_TIMEOUT_MS)
    {
      filtered_counts_per_second = 0.0f;
      stopped = true;
    }

    next_states[wheel].sample_time_ms = now_ms;
    next_states[wheel].accumulated_count =
      encoder_states[wheel].accumulated_count;
    next_states[wheel].delta_counts = delta;
    next_states[wheel].sample_period_ms = elapsed_ms;
    next_states[wheel].raw_counts_per_second = raw_counts_per_second;
    next_states[wheel].filtered_counts_per_second =
      filtered_counts_per_second;
    next_states[wheel].wheel_rpm =
      filtered_counts_per_second * 60.0f /
      (float)CHASSIS_ENCODER_COUNTS_PER_WHEEL_REV;
    next_states[wheel].wheel_angular_velocity_rad_s =
      filtered_counts_per_second * (2.0f * CHASSIS_PI_F) /
      (float)CHASSIS_ENCODER_COUNTS_PER_WHEEL_REV;
    next_states[wheel].wheel_linear_velocity_m_s =
      filtered_counts_per_second * CHASSIS_WHEEL_CIRCUMFERENCE_M /
      (float)CHASSIS_ENCODER_COUNTS_PER_WHEEL_REV;
    next_states[wheel].stopped = stopped;
    next_states[wheel].valid = quality==0U;
    next_states[wheel].sample_sequence=sequence;
    next_states[wheel].quality_flags=quality;
  }

  taskENTER_CRITICAL();
  for (uint32_t wheel = 0U; wheel < ENCODER_WHEEL_COUNT; ++wheel)
  {
    s_states[wheel] = next_states[wheel];
    s_previous_totals[wheel] = encoder_states[wheel].accumulated_count;
  }
  s_previous_sample_ms = now_ms;
  s_sample_sequence=sequence;
  taskEXIT_CRITICAL();
}

bool WheelSpeedEstimator_GetSnapshot(
  WheelSpeedState states[ENCODER_WHEEL_COUNT])
{
  if ((!s_initialized) || (states == NULL))
  {
    return false;
  }

  taskENTER_CRITICAL();
  for (uint32_t wheel = 0U; wheel < ENCODER_WHEEL_COUNT; ++wheel)
  {
    states[wheel] = s_states[wheel];
  }
  taskEXIT_CRITICAL();

  return true;
}
