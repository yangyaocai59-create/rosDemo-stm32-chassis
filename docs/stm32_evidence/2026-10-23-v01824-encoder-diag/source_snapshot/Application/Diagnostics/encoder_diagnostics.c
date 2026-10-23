#include "encoder_diagnostics.h"
#include <stddef.h>
#include "board_config.h"

typedef struct
{
  TIM_HandleTypeDef *timer;
  GPIO_TypeDef *gpio;
  uint8_t a, b, af;
  uint32_t arr;
} Channel;
static const Channel channels[4]={
  {BOARD_ENCODER_LF_TIMER_HANDLE,GPIOE,9,11,1,UINT16_MAX},
  {BOARD_ENCODER_LR_TIMER_HANDLE,GPIOA,15,1,1,UINT32_MAX},
  {BOARD_ENCODER_RF_TIMER_HANDLE,GPIOA,6,7,2,UINT16_MAX},
  {BOARD_ENCODER_RR_TIMER_HANDLE,GPIOD,12,13,2,UINT16_MAX}
};
static bool PinValid(const Channel *c,uint32_t pin)
{
  return ((c->gpio->MODER>>(2U*pin))&3U)==2U &&
    ((c->gpio->PUPDR>>(2U*pin))&3U)==0U &&
    ((c->gpio->AFR[pin/8U]>>(4U*(pin%8U)))&15U)==c->af;
}
bool EncoderDiagnostics_Read(EncoderDiagnosticWheel wheels[ENCODER_WHEEL_COUNT])
{
  EncoderState samples[ENCODER_WHEEL_COUNT];
  if (wheels==NULL || !EncoderService_GetSnapshot(samples)) return false;
  const uint32_t apb1=RCC->APB1ENR, apb2=RCC->APB2ENR, ahb=RCC->AHB1ENR;
  for (uint32_t w=0;w<4U;++w)
  {
    const Channel *c=&channels[w];
    const TIM_TypeDef *t=c->timer->Instance;
    uint8_t bad=0U;
    if ((t->CR1&TIM_CR1_CEN)==0U) bad|=1U;
    if ((t->SMCR&TIM_SMCR_SMS)!=3U) bad|=2U;
    if ((t->CCMR1&0xffffU)!=0x6161U) bad|=4U;
    if ((t->CCER&0xffU)!=0x11U) bad|=8U;
    if (t->ARR!=c->arr) bad|=16U;
    if (t->PSC!=0U || (t->CR1&TIM_CR1_CKD)!=0U) bad|=32U;
    if (!PinValid(c,c->a) || !PinValid(c,c->b)) bad|=64U;
    const bool timer_on=w==0U ? (apb2&RCC_APB2ENR_TIM1EN)!=0U :
      w==1U ? (apb1&RCC_APB1ENR_TIM2EN)!=0U :
      w==2U ? (apb1&RCC_APB1ENR_TIM3EN)!=0U : (apb1&RCC_APB1ENR_TIM4EN)!=0U;
    const uint32_t gpio_clock=w==0U ? RCC_AHB1ENR_GPIOEEN :
      w==3U ? RCC_AHB1ENR_GPIODEN : RCC_AHB1ENR_GPIOAEN;
    if (!timer_on || (ahb&gpio_clock)==0U) bad|=128U;
    const uint32_t input=c->gpio->IDR;
    wheels[w]=(EncoderDiagnosticWheel){.sampled_cnt=samples[w].raw_count,
      .live_cnt=t->CNT,.bad=bad,
      .ab=(uint8_t)(((input>>c->a)&1U)|(((input>>c->b)&1U)<<1))};
  }
  return true;
}
