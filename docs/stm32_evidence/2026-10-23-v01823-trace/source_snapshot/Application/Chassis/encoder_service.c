/**
 * @file encoder_service.c
 * @brief Four-channel raw encoder acquisition and counter-wrap handling.
 */

#include "encoder_service.h"

#include <limits.h>
#include <stddef.h>

#include "FreeRTOS.h"
#include "board_config.h"
#include "task.h"

typedef struct
{
  TIM_HandleTypeDef *timer;
  uint8_t counter_bits;
  int8_t forward_sign;
  EncoderState state;
} EncoderChannel;

static EncoderChannel s_channels[ENCODER_WHEEL_COUNT] = {
  [ENCODER_WHEEL_LF] = {
    .timer = BOARD_ENCODER_LF_TIMER_HANDLE,
    .forward_sign = BOARD_ENCODER_LF_FORWARD_SIGN
  },
  [ENCODER_WHEEL_LR] = {
    .timer = BOARD_ENCODER_LR_TIMER_HANDLE,
    .forward_sign = BOARD_ENCODER_LR_FORWARD_SIGN
  },
  [ENCODER_WHEEL_RF] = {
    .timer = BOARD_ENCODER_RF_TIMER_HANDLE,
    .forward_sign = BOARD_ENCODER_RF_FORWARD_SIGN
  },
  [ENCODER_WHEEL_RR] = {
    .timer = BOARD_ENCODER_RR_TIMER_HANDLE,
    .forward_sign = BOARD_ENCODER_RR_FORWARD_SIGN
  }
};

static const char *const s_wheel_names[ENCODER_WHEEL_COUNT] = {
  "LF", "LR", "RF", "RR"
};

static bool s_initialized;

static bool EncoderService_DetectCounterWidth(EncoderChannel *channel)
{
  const uint32_t auto_reload = __HAL_TIM_GET_AUTORELOAD(channel->timer);

  if (auto_reload == UINT16_MAX)
  {
    channel->counter_bits = 16U;
    return true;
  }

  if (auto_reload == UINT32_MAX)
  {
    channel->counter_bits = 32U;
    return true;
  }

  return false;
}

static int32_t EncoderService_CalculateDelta(uint32_t current,
                                             uint32_t previous,
                                             uint8_t counter_bits)
{
  uint32_t difference;

  if (counter_bits == 16U)
  {
    difference = (uint16_t)((uint16_t)current - (uint16_t)previous);
    if (difference <= INT16_MAX)
    {
      return (int32_t)difference;
    }
    return -((int32_t)(UINT16_MAX - difference) + 1L);
  }

  difference = current - previous;
  if (difference <= INT32_MAX)
  {
    return (int32_t)difference;
  }
  if (difference == (1UL << 31))
  {
    return INT32_MIN;
  }
  return -((int32_t)(UINT32_MAX - difference) + 1L);
}

static int32_t EncoderService_ApplyForwardPolarity(int32_t raw_delta,
                                                    int8_t forward_sign)
{
  if (forward_sign >= 0)
  {
    return raw_delta;
  }

  /* An exact half-range jump is ambiguous and cannot be negated as int32_t. */
  if (raw_delta == INT32_MIN)
  {
    return INT32_MAX;
  }
  return -raw_delta;
}

bool EncoderService_Init(void)
{
  uint32_t wheel;

  if (s_initialized)
  {
    return true;
  }

  for (wheel = 0U; wheel < ENCODER_WHEEL_COUNT; ++wheel)
  {
    if (!EncoderService_DetectCounterWidth(&s_channels[wheel]))
    {
      return false;
    }
  }

  for (wheel = 0U; wheel < ENCODER_WHEEL_COUNT; ++wheel)
  {
    if (HAL_TIM_Encoder_Start(s_channels[wheel].timer, TIM_CHANNEL_ALL) != HAL_OK)
    {
      while (wheel > 0U)
      {
        --wheel;
        (void)HAL_TIM_Encoder_Stop(s_channels[wheel].timer, TIM_CHANNEL_ALL);
      }
      return false;
    }

    s_channels[wheel].state.raw_count =
      __HAL_TIM_GET_COUNTER(s_channels[wheel].timer);
    s_channels[wheel].state.delta = 0;
    s_channels[wheel].state.accumulated_count = 0;
    s_channels[wheel].state.direction = ENCODER_DIRECTION_STOPPED;
  }

  s_initialized = true;
  return true;
}

void EncoderService_Update(void)
{
  uint32_t current_counts[ENCODER_WHEEL_COUNT];
  uint32_t wheel;

  if (!s_initialized)
  {
    return;
  }

  for (wheel = 0U; wheel < ENCODER_WHEEL_COUNT; ++wheel)
  {
    current_counts[wheel] = __HAL_TIM_GET_COUNTER(s_channels[wheel].timer);
  }

  taskENTER_CRITICAL();
  for (wheel = 0U; wheel < ENCODER_WHEEL_COUNT; ++wheel)
  {
    EncoderState *const state = &s_channels[wheel].state;
    const int32_t raw_delta = EncoderService_CalculateDelta(
      current_counts[wheel], state->raw_count, s_channels[wheel].counter_bits);
    const int32_t forward_delta = EncoderService_ApplyForwardPolarity(
      raw_delta, s_channels[wheel].forward_sign);

    state->raw_count = current_counts[wheel];
    state->delta = forward_delta;
    state->accumulated_count += forward_delta;

    if (forward_delta > 0)
    {
      state->direction = ENCODER_DIRECTION_POSITIVE;
    }
    else if (forward_delta < 0)
    {
      state->direction = ENCODER_DIRECTION_NEGATIVE;
    }
    else
    {
      state->direction = ENCODER_DIRECTION_STOPPED;
    }
  }
  taskEXIT_CRITICAL();
}

bool EncoderService_GetSnapshot(EncoderState states[ENCODER_WHEEL_COUNT])
{
  uint32_t wheel;

  if ((!s_initialized) || (states == NULL))
  {
    return false;
  }

  taskENTER_CRITICAL();
  for (wheel = 0U; wheel < ENCODER_WHEEL_COUNT; ++wheel)
  {
    states[wheel] = s_channels[wheel].state;
  }
  taskEXIT_CRITICAL();

  return true;
}

const char *EncoderService_GetWheelName(EncoderWheel wheel)
{
  if ((uint32_t)wheel >= ENCODER_WHEEL_COUNT)
  {
    return "?";
  }
  return s_wheel_names[wheel];
}

const char *EncoderService_GetDirectionName(EncoderDirection direction)
{
  if (direction == ENCODER_DIRECTION_POSITIVE)
  {
    return "POS";
  }
  if (direction == ENCODER_DIRECTION_NEGATIVE)
  {
    return "NEG";
  }
  return "STOP";
}
