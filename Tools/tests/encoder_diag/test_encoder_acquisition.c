#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "encoder_service.c"
#include "encoder_diagnostics.c"
TIM_TypeDef registers[4];
TIM_HandleTypeDef test_timers[4];
GPIO_TypeDef test_gpio[3];
RCC_TypeDef test_rcc;
static unsigned depth;
void Test_EnterCritical(void) { ++depth; }
void Test_ExitCritical(void) { assert(depth); --depth; }
int HAL_TIM_Encoder_Start(TIM_HandleTypeDef *t,uint32_t ch)
{ (void)ch;t->Instance->CR1|=1; return 0; }
int HAL_TIM_Encoder_Stop(TIM_HandleTypeDef *t,uint32_t ch)
{ (void)ch;t->Instance->CR1&=~1U; return 0; }
int main(void)
{
  int64_t expected[4]={0};
  for(unsigned w=0;w<4;w++)
  {test_timers[w].Instance=&registers[w];registers[w].ARR=w==1?UINT32_MAX:UINT16_MAX;
   registers[w].CNT=w==1 ? 0xfffffffeU:0xfffeU;}
  assert(EncoderService_Init());
  for(unsigned n=0;n<10000;n++)
  {
    int32_t increments[4];
    for(unsigned w=0;w<4;w++)
    {
      increments[w]=(int32_t)((n*(w+3)+w)%41)-20;
      registers[w].CNT=(registers[w].CNT+(uint32_t)increments[w])&registers[w].ARR;
      expected[w]+=(w<2?1:-1)*increments[w];
    }
    EncoderService_Update();
    EncoderState states[4]; assert(EncoderService_GetSnapshot(states));
    EncoderDiagnosticWheel diag[4]; assert(EncoderDiagnostics_Read(diag));
    for(unsigned w=0;w<4;w++)
    {
      assert(states[w].raw_count==registers[w].CNT && states[w].accumulated_count==expected[w]);
      assert(states[w].delta==(w<2?1:-1)*increments[w]);
      assert(diag[w].sampled_cnt==states[w].raw_count && diag[w].live_cnt==states[w].raw_count);
    }
  }
  EncoderState before[4],after[4];assert(EncoderService_GetSnapshot(before));
  registers[0].CNT=(registers[0].CNT+3U)&0xffffU;
  EncoderDiagnosticWheel diag[4];assert(EncoderDiagnostics_Read(diag));
  assert(diag[0].sampled_cnt==before[0].raw_count);
  assert(diag[0].live_cnt==registers[0].CNT && diag[0].live_cnt!=diag[0].sampled_cnt);
  assert(EncoderService_GetSnapshot(after));assert(!memcmp(before,after,sizeof before));
  assert(!depth);
  puts("PASS: real acquisition, 40000 independent wheel updates, signed normalization, 16/32-bit wraps; diagnostics do not consume new counts");
}
