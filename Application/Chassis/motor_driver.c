/**
 * @file motor_driver.c
 * @brief Safe four-channel TB6612 motor output abstraction.
 */

#include "motor_driver.h"

#include <stddef.h>

#include "board_config.h"
#include "emergency_stop.h"
#include "main.h"

typedef struct
{
  TIM_HandleTypeDef *timer;
  uint32_t channel;
  GPIO_TypeDef *in1_port;
  uint16_t in1_pin;
  GPIO_TypeDef *in2_port;
  uint16_t in2_pin;
  int8_t forward_sign;
} MotorDriverChannel;

static const MotorDriverChannel s_channels[MOTOR_DRIVER_WHEEL_COUNT] = {
  [MOTOR_DRIVER_WHEEL_LF] = {
    BOARD_PWM_LF_TIMER_HANDLE, BOARD_PWM_LF_CHANNEL,
    LF_IN1_GPIO_Port, LF_IN1_Pin, LF_IN2_GPIO_Port, LF_IN2_Pin,
    BOARD_MOTOR_LF_FORWARD_SIGN
  },
  [MOTOR_DRIVER_WHEEL_LR] = {
    BOARD_PWM_LR_TIMER_HANDLE, BOARD_PWM_LR_CHANNEL,
    LR_IN1_GPIO_Port, LR_IN1_Pin, LR_IN2_GPIO_Port, LR_IN2_Pin,
    BOARD_MOTOR_LR_FORWARD_SIGN
  },
  [MOTOR_DRIVER_WHEEL_RF] = {
    BOARD_PWM_RF_TIMER_HANDLE, BOARD_PWM_RF_CHANNEL,
    RF_IN1_GPIO_Port, RF_IN1_Pin, RF_IN2_GPIO_Port, RF_IN2_Pin,
    BOARD_MOTOR_RF_FORWARD_SIGN
  },
  [MOTOR_DRIVER_WHEEL_RR] = {
    BOARD_PWM_RR_TIMER_HANDLE, BOARD_PWM_RR_CHANNEL,
    RR_IN1_GPIO_Port, RR_IN1_Pin, RR_IN2_GPIO_Port, RR_IN2_Pin,
    BOARD_MOTOR_RR_FORWARD_SIGN
  }
};

static MotorDriverStatus s_status;
/* >=2 HAL ticks guarantees >=1 ms real coast time despite tick phase,
 * comfortably longer than the 50 us TIM8 period. No busy wait or HAL_Delay. */
#define MOTOR_DRIVER_DIRECTION_GUARD_MS 2U
static int8_t s_electrical_sign[MOTOR_DRIVER_WHEEL_COUNT];
static bool s_zero_pending[MOTOR_DRIVER_WHEEL_COUNT];
static uint32_t s_zero_since_ms[MOTOR_DRIVER_WHEEL_COUNT];
static int16_t s_diagnostic_request[MOTOR_DRIVER_WHEEL_COUNT];

static bool MotorDriver_IsWheelValid(MotorDriverWheel wheel)
{
  return ((uint32_t)wheel < (uint32_t)MOTOR_DRIVER_WHEEL_COUNT);
}

static int16_t MotorDriver_SaturateCommand(int16_t command_permille)
{
  if (command_permille > MOTOR_DRIVER_COMMAND_MAX_PERMILLE)
  {
    return MOTOR_DRIVER_COMMAND_MAX_PERMILLE;
  }
  if (command_permille < -MOTOR_DRIVER_COMMAND_MAX_PERMILLE)
  {
    return -MOTOR_DRIVER_COMMAND_MAX_PERMILLE;
  }
  return command_permille;
}

static uint32_t MotorDriver_CommandToCompare(const MotorDriverChannel *channel,
                                              int16_t command_permille)
{
  const uint32_t period_ticks =
    __HAL_TIM_GET_AUTORELOAD(channel->timer) + 1U;
  const uint32_t magnitude = (uint32_t)((command_permille < 0) ?
                                        -command_permille : command_permille);

  return (uint32_t)(((uint64_t)period_ticks * magnitude) /
                    MOTOR_DRIVER_COMMAND_MAX_PERMILLE);
}

static void MotorDriver_WriteDirection(const MotorDriverChannel *channel,
                                       GPIO_PinState in1,
                                       GPIO_PinState in2)
{
  if ((in1 == GPIO_PIN_RESET) && (in2 == GPIO_PIN_RESET) &&
      (channel->in1_port == channel->in2_port))
  {
    /* One BSRR write: coast is effective regardless of CCR preload. */
    HAL_GPIO_WritePin(channel->in1_port,
                     channel->in1_pin | channel->in2_pin, GPIO_PIN_RESET);
    return;
  }
  HAL_GPIO_WritePin(channel->in1_port, channel->in1_pin, in1);
  HAL_GPIO_WritePin(channel->in2_port, channel->in2_pin, in2);
}

bool MotorDriver_Init(void)
{
  uint32_t started_channels = 0U;

  MotorDriver_ForceSafeState();

  for (uint32_t wheel = 0U; wheel < MOTOR_DRIVER_WHEEL_COUNT; ++wheel)
  {
    const MotorDriverChannel *channel = &s_channels[wheel];

    MotorDriver_WriteDirection(channel, GPIO_PIN_RESET, GPIO_PIN_RESET);
    __HAL_TIM_SET_COMPARE(channel->timer, channel->channel, 0U);

    if (HAL_TIM_PWM_Start(channel->timer, channel->channel) != HAL_OK)
    {
      for (uint32_t stop_index = 0U;
           stop_index < started_channels;
           ++stop_index)
      {
        (void)HAL_TIM_PWM_Stop(s_channels[stop_index].timer,
                               s_channels[stop_index].channel);
      }
      MotorDriver_ForceSafeState();
      return false;
    }
    ++started_channels;
  }

  s_status.initialized = true;
  s_status.standby_enabled = false;
  return true;
}

bool MotorDriver_Enable(void)
{
  uint32_t previous_primask;

  if (!s_status.initialized)
  {
    return false;
  }

  if (!EmergencyStop_CanEnableMotor())
  {
    Board_EmergencyMotorShutdown();
    s_status.standby_enabled = false;
    return false;
  }

  previous_primask = __get_PRIMASK();
  __disable_irq();
  MotorDriver_CoastAll();
  if (EmergencyStop_CanEnableMotor())
  {
    HAL_GPIO_WritePin(MOTOR_STBY_1_GPIO_Port,
                      MOTOR_STBY_1_Pin,
                      GPIO_PIN_SET);
    /* PD11 is reserved/unconnected on the installed D24A and stays inactive. */
    HAL_GPIO_WritePin(MOTOR_STBY_2_GPIO_Port,
                      MOTOR_STBY_2_Pin,
                      GPIO_PIN_RESET);
    s_status.standby_enabled = true;
  }
  if (previous_primask == 0U)
  {
    __enable_irq();
  }

  if (!EmergencyStop_CanEnableMotor())
  {
    Board_EmergencyMotorShutdown();
    s_status.standby_enabled = false;
    return false;
  }
  return s_status.standby_enabled;
}

void MotorDriver_Disable(void)
{
  const uint32_t previous_primask = __get_PRIMASK();
  __disable_irq();
  /* PD10 is the only physical D24A common standby input. */
  HAL_GPIO_WritePin(MOTOR_STBY_1_GPIO_Port,
                    MOTOR_STBY_1_Pin,
                    GPIO_PIN_RESET);
  HAL_GPIO_WritePin(MOTOR_STBY_2_GPIO_Port,
                    MOTOR_STBY_2_Pin,
                    GPIO_PIN_RESET);
  s_status.standby_enabled = false;
  MotorDriver_CoastAll();
  if (previous_primask == 0U) { __enable_irq(); }
}

bool MotorDriver_SetCommand(MotorDriverWheel wheel, int16_t command_permille)
{
  const MotorDriverChannel *channel;
  int16_t saturated_command;
  int16_t electrical_command;
  uint32_t compare;
  uint32_t previous_primask;

  if ((!s_status.initialized) || (!MotorDriver_IsWheelValid(wheel)))
  {
    return false;
  }

  saturated_command = MotorDriver_SaturateCommand(command_permille);
  if ((!s_status.standby_enabled) && (saturated_command != 0))
  {
    return false;
  }

  if ((saturated_command != 0) && (!EmergencyStop_CanEnableMotor()))
  {
    Board_EmergencyMotorShutdown();
    s_status.standby_enabled = false;
    return false;
  }

  channel = &s_channels[wheel];
  electrical_command =
    (int16_t)(saturated_command * channel->forward_sign);

  previous_primask = __get_PRIMASK();
  __disable_irq();

  /* Disable may have run between the optimistic checks above and this lock. */
  if ((!s_status.initialized) ||
      ((!s_status.standby_enabled) && (saturated_command != 0)) ||
      ((saturated_command != 0) && (!EmergencyStop_CanEnableMotor())))
  {
    if (previous_primask == 0U) { __enable_irq(); }
    return false;
  }

  const int8_t requested_sign = (electrical_command > 0) ? 1 :
    ((electrical_command < 0) ? -1 : 0);
  s_diagnostic_request[wheel] = saturated_command;
  const uint32_t now_ms = HAL_GetTick();
  if ((requested_sign == 0) ||
      ((s_electrical_sign[wheel] != 0) &&
       (requested_sign != s_electrical_sign[wheel])))
  {
    if ((s_electrical_sign[wheel] != 0) || (!s_zero_pending[wheel]))
    {
      s_zero_since_ms[wheel] = now_ms;
      s_zero_pending[wheel] = true;
    }
    MotorDriver_WriteDirection(channel, GPIO_PIN_RESET, GPIO_PIN_RESET);
    __HAL_TIM_SET_COMPARE(channel->timer, channel->channel, 0U);
    s_electrical_sign[wheel] = 0;
    saturated_command = 0;
  }
  else if (s_zero_pending[wheel] &&
           ((uint32_t)(now_ms - s_zero_since_ms[wheel]) <
            MOTOR_DRIVER_DIRECTION_GUARD_MS))
  {
    /* Latest command will be retried at the next control iteration. */
    saturated_command = 0;
  }
  else
  {
    if (requested_sign != s_electrical_sign[wheel])
    {
      /* Previous zero CCR has now crossed many hardware update events. */
      MotorDriver_WriteDirection(channel,
        (requested_sign > 0) ? GPIO_PIN_SET : GPIO_PIN_RESET,
        (requested_sign < 0) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    }
    compare = MotorDriver_CommandToCompare(channel, electrical_command);
    __HAL_TIM_SET_COMPARE(channel->timer, channel->channel, compare);
    s_electrical_sign[wheel] = requested_sign;
    s_zero_pending[wheel] = false;
  }
  s_status.command_permille[wheel] = saturated_command;

  if (previous_primask == 0U)
  {
    __enable_irq();
  }

  if ((saturated_command != 0) && (!EmergencyStop_CanEnableMotor()))
  {
    Board_EmergencyMotorShutdown();
    s_status.standby_enabled = false;
    s_status.command_permille[wheel] = 0;
    return false;
  }
  return true;
}

bool MotorDriver_Coast(MotorDriverWheel wheel)
{
  if ((!s_status.initialized) || (!MotorDriver_IsWheelValid(wheel)))
  {
    return false;
  }

  const uint32_t previous_primask = __get_PRIMASK();
  __disable_irq();
  MotorDriver_WriteDirection(&s_channels[wheel],
                             GPIO_PIN_RESET,
                             GPIO_PIN_RESET);
  __HAL_TIM_SET_COMPARE(s_channels[wheel].timer,
                        s_channels[wheel].channel,
                        0U);
  if ((s_electrical_sign[wheel] != 0) || (!s_zero_pending[wheel]))
  {
    s_zero_since_ms[wheel] = HAL_GetTick();
    s_zero_pending[wheel] = true;
  }
  s_electrical_sign[wheel] = 0;
  s_status.command_permille[wheel] = 0;
  s_diagnostic_request[wheel] = 0;
  if (previous_primask == 0U) { __enable_irq(); }
  return true;
}

void MotorDriver_CoastAll(void)
{
  if (!s_status.initialized)
  {
    return;
  }

  for (uint32_t wheel = 0U; wheel < MOTOR_DRIVER_WHEEL_COUNT; ++wheel)
  {
    (void)MotorDriver_Coast((MotorDriverWheel)wheel);
  }
}

void MotorDriver_ForceSafeState(void)
{
  Board_ForceMotorSafeState();

  for (uint32_t wheel = 0U; wheel < MOTOR_DRIVER_WHEEL_COUNT; ++wheel)
  {
    MotorDriver_WriteDirection(&s_channels[wheel],
                               GPIO_PIN_RESET,
                               GPIO_PIN_RESET);
    s_status.command_permille[wheel] = 0;
    s_diagnostic_request[wheel] = 0;
    s_electrical_sign[wheel] = 0;
    s_zero_pending[wheel] = true;
    s_zero_since_ms[wheel] = HAL_GetTick();
  }

  s_status.standby_enabled = false;
  s_status.initialized = false;
}

bool MotorDriver_GetDiagnostics(MotorDriverDiagnostics *diagnostics)
{
  if (diagnostics == NULL) { return false; }
  *diagnostics = (MotorDriverDiagnostics){0};
  for (uint32_t w = 0U; w < MOTOR_DRIVER_WHEEL_COUNT; ++w)
  {
    const MotorDriverChannel *c = &s_channels[w];
    diagnostics->compare[w] = __HAL_TIM_GET_COMPARE(c->timer, c->channel);
    diagnostics->requested_permille[w] = s_diagnostic_request[w];
    if ((c->in1_port->ODR & c->in1_pin) != 0U)
      diagnostics->direction_bits |= (uint16_t)(1U << (2U * w));
    if ((c->in2_port->ODR & c->in2_pin) != 0U)
      diagnostics->direction_bits |= (uint16_t)(2U << (2U * w));
    if (s_zero_pending[w]) diagnostics->guard_mask |= (uint8_t)(1U << w);
  }
  diagnostics->auto_reload = __HAL_TIM_GET_AUTORELOAD(s_channels[0].timer);
  diagnostics->bdtr = s_channels[0].timer->Instance->BDTR;
  diagnostics->ccer = s_channels[0].timer->Instance->CCER;
  diagnostics->standby_pin =
    (MOTOR_STBY_1_GPIO_Port->ODR & MOTOR_STBY_1_Pin) != 0U;
  return true;
}

bool MotorDriver_GetStatus(MotorDriverStatus *status)
{
  if (status == NULL)
  {
    return false;
  }

  *status = s_status;
  if (EmergencyStop_IsLatched() || EmergencyStop_IsInputActive())
  {
    status->standby_enabled = false;
    for (uint32_t wheel = 0U; wheel < MOTOR_DRIVER_WHEEL_COUNT; ++wheel)
    {
      status->command_permille[wheel] = 0;
    }
  }
  return true;
}
