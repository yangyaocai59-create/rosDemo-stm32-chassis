#include <assert.h>
#include <math.h>
#include <stdio.h>
#include "yaw_rate_control_core.h"
#include "chassis_yaw_feedback.h"

static unsigned depth;
void Test_EnterCritical(void) { ++depth; }
void Test_ExitCritical(void) { assert(depth); --depth; }

static void Baseline(uint32_t base,uint32_t epoch)
{
  ChassisYawFeedback_Init();
  for (unsigned k=0;k<=160;k++)
    ChassisYawFeedback_Observe(base+k*20,true,true,.041f,base+k*20,k*2,epoch);
  ChassisYawFeedback s;assert(ChassisYawFeedback_Get(&s));
  assert(s.ready && s.valid && fabsf(s.bias_rad_s-.041f)<.000001f);
  assert(s.baseline_samples>=100 && s.baseline_std_rad_s<.000001f);
}

int main(void)
{
  for(unsigned wrap=0;wrap<2;wrap++) {
    uint32_t base=wrap?0xfffff800U:1000U;
    Baseline(base,1);
    uint32_t now=base+3220;
    ChassisYawFeedback_Observe(now,false,true,.361f,now,322,1);
    ChassisYawFeedback s;assert(ChassisYawFeedback_Get(&s));
    assert(s.ready && fabsf(s.rate_rad_s-.32f)<.000001f);
    ChassisYawFeedback_Observe(now+20,false,true,.361f,now-50,324,1);
    assert(ChassisYawFeedback_Get(&s) && !s.ready && !s.valid);
    ChassisYawFeedback_Observe(now+40,false,true,.361f,now+40,326,1);
    assert(ChassisYawFeedback_Get(&s) && !s.ready);
    Baseline(base,1);
    ChassisYawFeedback_Observe(now,false,true,.041f,now,322,2);
    assert(ChassisYawFeedback_Get(&s) && !s.ready);
    Baseline(base,1);
    ChassisYawFeedback_Observe(now,false,true,NAN,now,322,1);
    assert(ChassisYawFeedback_Get(&s) && !s.ready && !s.valid);
    Baseline(base,1);
    ChassisYawFeedback_Observe(base+3200,true,true,.07f,base+3200,320,1);
    assert(ChassisYawFeedback_Get(&s) && !s.ready && !s.valid);
    Baseline(base,1);
    ChassisYawFeedback_Observe(now,false,true,.041f,now,9999,1);
    assert(ChassisYawFeedback_Get(&s) && !s.ready && !s.valid);
  }
  ChassisYawFeedback_Init();
  for(unsigned k=0;k<1000;k++)
    ChassisYawFeedback_Observe(k*20,true,true,k%2?.05f:-.05f,k*20,k*2,1);
  ChassisYawFeedback feedback;assert(ChassisYawFeedback_Get(&feedback) && !feedback.ready);
  ChassisYawFeedback_Init();
  for(unsigned k=0;k<1000;k++)
    ChassisYawFeedback_Observe(k*20,false,true,.041f,k*20,k*2,1);
  assert(ChassisYawFeedback_Get(&feedback) && !feedback.ready);
  puts("PASS: stationary-only RAM bias, motion/noise rejection, freshness, epoch/sequence changes, NaN and tick wrap");

  for(int sign=-1;sign<=1;sign+=2) {
    YawRateControlState state={0};
    float previous=0;
    for(unsigned k=0;k<2000;k++) {
      const float correction=YawRateControl_Step(&state,0,sign*.4f,sign*.32f,true,false,20);
      assert(isfinite(correction) && fabsf(correction)<=.100001f);
      assert(fabsf(correction-previous)<=.002401f);
      previous=correction;
    }
    assert(sign*state.correction>.09f && fabsf(state.integral)<=.100001f);
    (void)YawRateControl_Step(&state,0,sign*.4f,sign*.1f,true,true,20);
    assert(fabsf(state.correction)<fabsf(previous) && state.integral==0 && state.inhibited);
    for(unsigned k=0;k<100;k++) {
      float c=YawRateControl_Step(&state,0,sign*.4f,0,false,false,20);
      assert(fabsf(c)<=fabsf(previous)+.000001f); previous=c;
    }
    assert(fabsf(state.correction)<.000001f && state.integral==0);
    (void)YawRateControl_Step(&state,.02f,sign*.4f,sign*.3f,true,false,20);
    assert(state.correction==0 && !state.active);
    (void)YawRateControl_Step(&state,0,sign*.4f,sign*.3f,true,false,101);
    assert(state.correction==0 && !state.active);
    for(unsigned k=0;k<1000;k++)
      (void)YawRateControl_Step(&state,0,sign*.4f,sign*.3f,true,true,20);
    assert(state.integral==0); /* wheel-output saturation forbids winding up */
    (void)YawRateControl_Step(&state,0,-sign*.4f,-sign*.3f,true,false,20);
    assert(state.correction==0 && state.integral==0);
    (void)YawRateControl_Step(&state,0,0,0,true,false,20);
    assert(state.correction==0 && !state.active);
  }
  puts("PASS: both directions, correction/rate bounds, invalid-feedback fade, no integral windup, reversal and zero command reset");

#if CHASSIS_CONTROL_REVISION >= 204
  for(int sign=-1;sign<=1;sign+=2) {
    YawRateControlState state={0};float previous=0;
    for(unsigned k=0;k<200;k++) {
      const float c=YawRateControl_Step(&state,0,sign*.4f,sign*.5f,true,true,20);
      assert(sign*c<=.000001f && fabsf(c)<=.100001f);
      assert(fabsf(c-previous)<=.002401f && state.integral==0);
      if(k<30) assert(c==0); /* no early intervention during warm-up */
      previous=c;
    }
    assert(sign*state.correction<-.05f && state.inhibited);
    for(unsigned k=0;k<200;k++) {
      const float c=YawRateControl_Step(&state,0,sign*.4f,sign*.2f,true,true,20);
      assert(sign*c<=.000001f && fabsf(c-previous)<=.002401f);
      assert(state.integral==0);previous=c;
    }
    assert(fabsf(state.correction)<.000001f);
    /* Even a pre-existing boost must fade through zero at the slew limit. */
    state.correction=sign*.1f;previous=state.correction;
    for(unsigned k=0;k<200;k++) {
      const float c=YawRateControl_Step(&state,0,sign*.4f,sign*.5f,true,true,20);
      assert(sign*c<=sign*previous+.000001f);
      assert(fabsf(c-previous)<=.002401f && state.integral==0);previous=c;
    }
    assert(sign*state.correction<-.05f);
  }
  puts("PASS: wheel-authority loss permits bounded overspeed reduction, forbids extra drive and integral in both directions, including warm-up and prior boost");
#endif

  /* A stated synthetic first-order plant, not a hardware prediction. */
  for(unsigned g=0;g<3;g++) {
    const float gain=.8f+.2f*g;
    YawRateControlState state={0}; float rate=0,open_rate=0;
    double closed_square=0,open_square=0;
    for(unsigned k=0;k<1500;k++) {
      float correction=YawRateControl_Step(&state,0,.4f,rate,true,false,20);
      rate+=(gain*(.4f+correction)-rate)*(.02f/.3f);
      open_rate+=(gain*.4f-open_rate)*(.02f/.3f);
      if(k>500) {closed_square+=(.4f-rate)*(.4f-rate);open_square+=(.4f-open_rate)*(.4f-open_rate);}
    }
    if(gain!=1.0f) assert(closed_square<open_square*.1);
    assert(fabsf(rate-.4f)<.005f);
  }
  assert(!depth);
  puts("PASS: synthetic gain 0.8/1.0/1.2 plants converge inside correction authority; not an on-vehicle acceptance test");
  return 0;
}
