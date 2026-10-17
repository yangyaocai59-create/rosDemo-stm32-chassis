/**
 * @file board_config.c
 * @brief Board-level safety helpers that use CubeMX-generated pin symbols.
 */

#include "board_config.h"

#include "main.h"

void Board_EmergencyMotorShutdown(void)
{
  const uint32_t motor_gpio_pins =
    MOTOR_STBY_1_Pin | MOTOR_STBY_2_Pin |
    LF_IN1_Pin | LF_IN2_Pin | LR_IN1_Pin | LR_IN2_Pin |
    RF_IN1_Pin | RF_IN2_Pin | RR_IN1_Pin | RR_IN2_Pin;

  /* All motor control pins are on GPIOD. BSRR upper half atomically clears. */
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;
  (void)RCC->AHB1ENR;
  GPIOD->BSRR = motor_gpio_pins << 16U;

  if ((RCC->APB2ENR & RCC_APB2ENR_TIM8EN) != 0U)
  {
    TIM8->CCR1 = 0U;
    TIM8->CCR2 = 0U;
    TIM8->CCR3 = 0U;
    TIM8->CCR4 = 0U;
    TIM8->BDTR &= ~TIM_BDTR_MOE;
  }
  __DSB();
}

void Board_ForceMotorSafeState(void)
{
  GPIO_InitTypeDef gpio_init = {0};

  /* Standby must be actively low even if startup later enters Error_Handler. */
  Board_EmergencyMotorShutdown();

  gpio_init.Pin = MOTOR_STBY_1_Pin | MOTOR_STBY_2_Pin;
  gpio_init.Mode = GPIO_MODE_OUTPUT_PP;
  gpio_init.Pull = GPIO_NOPULL;
  gpio_init.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &gpio_init);

  if (__HAL_RCC_TIM8_IS_CLK_ENABLED())
  {
    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, 0U);
    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, 0U);
    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, 0U);
    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_4, 0U);
    __HAL_TIM_MOE_DISABLE_UNCONDITIONALLY(&htim8);
  }
}
