#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "encoder_diagnostics.c"
TIM_TypeDef registers[4];
TIM_HandleTypeDef test_timers[4];
GPIO_TypeDef test_gpio[3];
RCC_TypeDef test_rcc;
static EncoderState source[4];
static bool available=true;
bool EncoderService_GetSnapshot(EncoderState states[4])
{ memcpy(states,source,sizeof source); return available; }
static void Reset(void)
{
  memset(test_gpio,0,sizeof test_gpio);
  test_rcc=(RCC_TypeDef){.APB1ENR=7,.APB2ENR=1,.AHB1ENR=25};
  for(unsigned w=0;w<4;w++)
  {
    test_timers[w].Instance=&registers[w];
    registers[w]=(TIM_TypeDef){.CR1=1,.SMCR=3,.CCMR1=0x6161,.CCER=0x11,
      .ARR=w==1?UINT32_MAX:UINT16_MAX,.CNT=100+w};
    source[w].raw_count=90+w;
    const Channel *c=&channels[w];
    const unsigned pins[2]={c->a,c->b};
    for(unsigned k=0;k<2;k++)
    { c->gpio->MODER |= 2U<<(2U*pins[k]);
      c->gpio->AFR[pins[k]/8] |= c->af<<(4U*(pins[k]%8)); }
    c->gpio->IDR |= 1U<<c->a;
  }
}
int main(void)
{
  EncoderDiagnosticWheel d[4]; Reset();
  TIM_TypeDef before[4]; GPIO_TypeDef gpio_before[3]; EncoderState counts_before[4];
  memcpy(before,registers,sizeof before); memcpy(gpio_before,test_gpio,sizeof gpio_before);
  memcpy(counts_before,source,sizeof source);
  assert(EncoderDiagnostics_Read(d));
  for(unsigned w=0;w<4;w++)
  { assert(!d[w].bad && d[w].ab==1 && d[w].sampled_cnt==90+w && d[w].live_cnt==100+w); }
  assert(!memcmp(before,registers,sizeof before) && !memcmp(gpio_before,test_gpio,sizeof gpio_before));
  assert(!memcmp(counts_before,source,sizeof source));
  puts("PASS: four mappings, AF/pin order, independent sampled/live counts, no register/source writes");
  for(unsigned w=0;w<4;w++) for(unsigned bit=0;bit<8;bit++)
  {
    Reset(); const Channel *c=&channels[w]; TIM_TypeDef *t=&registers[w];
    switch(bit) {
      case 0:t->CR1=0;break; case 1:t->SMCR=0;break;
      case 2:t->CCMR1^=0x10;break;case 3:t->CCER=0;break;
      case 4:t->ARR=42;break;case 5:t->PSC=1;break;
      case 6:c->gpio->AFR[c->a/8]^=1U<<(4U*(c->a%8));break;
      case 7:if(w==0) test_rcc.APB2ENR=0;else test_rcc.APB1ENR&=~(1U<<(w-1));break;
    }
    assert(EncoderDiagnostics_Read(d)); assert(d[w].bad==(1U<<bit));
  }
  Reset(); registers[0].CR1|=TIM_CR1_CKD;
  assert(EncoderDiagnostics_Read(d) && d[0].bad==32);
  Reset(); test_gpio[1].PUPDR|=1U<<2;
  assert(EncoderDiagnostics_Read(d) && d[1].bad==64);
  Reset(); test_rcc.AHB1ENR &= ~RCC_AHB1ENR_GPIODEN;
  assert(EncoderDiagnostics_Read(d) && d[3].bad==128);
  Reset(); registers[0].CR1|=0x10; /* DIR dynamically changes normally. */
  registers[0].CNT=0xffff;source[0].raw_count=0;
  assert(EncoderDiagnostics_Read(d) && !d[0].bad && d[0].live_cnt==0xffff);
  assert(!EncoderDiagnostics_Read(NULL));available=false;assert(!EncoderDiagnostics_Read(d));
  puts("PASS: every configuration category, clock/pull/CKD, DIR and counter wrap, invalid arguments");
}
