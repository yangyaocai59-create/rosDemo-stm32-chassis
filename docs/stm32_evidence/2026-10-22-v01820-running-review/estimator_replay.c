/* Frozen REAL estimator + synthetic accumulated encoder counts; no hardware. */
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "wheel_speed_estimator.h"
#include "source_snapshot/chassis_config.h"
static EncoderState encoder[4];
static unsigned depth;
void Test_EnterCritical(void) { depth++; }
void Test_ExitCritical(void) { assert(depth); depth--; }
bool EncoderService_GetSnapshot(EncoderState states[ENCODER_WHEEL_COUNT])
{ memcpy(states, encoder, sizeof(encoder)); return true; }
#include "source_snapshot/wheel_speed_estimator.c"
int main(void)
{
  unsigned t, dt, count=0;
  int dc[4], cps[4], ff[4], ix[4], pwm[4];
  puts("t,wheel,raw,measured,stopped");
  while (scanf("%u %u",&t,&dt)==2)
  {
    assert(dt==20);
    for (unsigned w=0; w<4; w++)
      assert(scanf("%d %d %d %d %d",&dc[w],&cps[w],&ff[w],&ix[w],&pwm[w])==5);
    if (count==0)
    {
      assert(WheelSpeedEstimator_Init(t));
      for (unsigned w=0; w<4; w++)
      {
        s_states[w].filtered_counts_per_second=(float)cps[w]+(cps[w]>0 ? .5f : cps[w]<0 ? -.5f : 0);
        s_states[w].raw_counts_per_second=dc[w]*1000.0f/dt;
        s_states[w].stopped=false;
        s_states[w].valid=true;
        s_last_motion_ms[w]=t-(w==3 ? 140 : 0);
      }
    }
    else
    {
      for (unsigned w=0; w<4; w++) encoder[w].accumulated_count+=dc[w];
      WheelSpeedEstimator_Update(t);
    }
    for (unsigned w=0; w<4; w++)
      printf("%u,%u,%.9f,%.9f,%d\n",t,w,
        s_states[w].raw_counts_per_second,s_states[w].filtered_counts_per_second,
        s_states[w].stopped);
    assert(!depth);
    count++;
  }
  assert(count==127);
  return 0;
}
