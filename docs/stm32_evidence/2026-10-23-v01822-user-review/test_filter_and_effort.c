/* Prescribed encoder input, actual estimator and separately linked controller.
 * No physical motor model, fault manager, serial ports or board operations. */
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "wheel_speed_estimator.c"
#include "wheel_speed_controller.h"

static EncoderState inputs[ENCODER_WHEEL_COUNT];
static unsigned depth;
void Test_EnterCritical(void) { ++depth; }
void Test_ExitCritical(void) { assert(depth); --depth; }
bool EncoderService_GetSnapshot(EncoderState states[ENCODER_WHEEL_COUNT])
{ memcpy(states,inputs,sizeof inputs); return true; }

static WheelSpeedControllerState Control(unsigned wheel,float target,float speed)
{
  int16_t pwm;
  assert(WheelSpeedController_UpdateWithMode((MotorDriverWheel)wheel,target,speed,0,
    WHEEL_SPEED_CONTROL_MODE_PIVOT,20,&pwm));
  WheelSpeedControllerState state;
  assert(WheelSpeedController_GetState((MotorDriverWheel)wheel,&state));
  return state;
}

int main(void)
{
  for(unsigned wheel=0;wheel<4;wheel+=2) for(int sign=-1;sign<=1;sign+=2)
  {
    memset(inputs,0,sizeof inputs);
    assert(WheelSpeedEstimator_Init(0));
    WheelSpeedController_Init();
    for(unsigned k=0;k<50;k++) Control(wheel,sign*240,sign*240);
    s_states[wheel].filtered_counts_per_second=sign*240;
    s_states[wheel].stopped=false;
    unsigned low_at=0, recovery_at=0, zero_at=0;
    for(unsigned t=20;t<=300;t+=20)
    {
      EncoderState original[ENCODER_WHEEL_COUNT]; memcpy(original,inputs,sizeof inputs);
      WheelSpeedEstimator_Update(t);
      assert(memcmp(original,inputs,sizeof inputs)==0);
      WheelSpeedState speeds[ENCODER_WHEEL_COUNT];
      assert(WheelSpeedEstimator_GetSnapshot(speeds));
      float v=speeds[wheel].filtered_counts_per_second;
      assert(speeds[wheel].delta_counts==0 && speeds[wheel].accumulated_count==0);
      if(fabsf(v)<=20 && !low_at) low_at=t;
      if(v==0 && !zero_at) zero_at=t;
      WheelSpeedControllerState s=Control(wheel,sign*240,v);
      WheelSpeedFrontPivotState front;
      assert(WheelSpeedController_GetFrontPivotState((MotorDriverWheel)wheel,&front));
      if(front.recovering && !recovery_at) recovery_at=t;
      printf("FILTER,%u,%d,%u,%.9g,%.9g,%.9g,%.9g,%d,%u\n",wheel,sign,t,
        v,s.feedforward_permille,s.proportional_permille,s.integrator_permille,
        s.command_permille,front.recovering?1U:0U);
    }
    assert(low_at==160 && zero_at==200);
#if REVIEW_LEGACY
    assert(recovery_at==200);
#else
    assert(recovery_at==0);
#endif
    printf("SUMMARY,%u,%d,%u,%u,%u\n",wheel,sign,low_at,recovery_at,zero_at);
  }
  for(unsigned wheel=0;wheel<4;wheel++) for(int sign=-1;sign<=1;sign+=2)
  {
    WheelSpeedController_Init();
    for(unsigned k=0;k<50;k++) Control(wheel,sign*240,sign*240);
    WheelSpeedControllerState under=Control(wheel,sign*240,sign*140);
    WheelSpeedController_Init();
    for(unsigned k=0;k<50;k++) Control(wheel,sign*240,sign*240);
    WheelSpeedControllerState over=Control(wheel,sign*240,sign*340);
    assert(fabsf(under.proportional_permille*sign-5)<.0001f);
#if REVIEW_LEGACY
    assert(fabsf(over.proportional_permille*sign+15)<.0001f);
#else
    assert(fabsf(over.proportional_permille*sign+5)<.0001f);
    WheelSpeedController_Init();
    WheelSpeedControllerState s={0};
    /* Effort envelope only: do not run a real stalled motor for this period.
     * Actual runtime fault guards are tested separately. */
    for(unsigned k=0;k<150;k++) s=Control(wheel,sign*240,0);
    assert(fabsf(s.integrator_permille*sign-50)<.0001f);
    float bound=s.feedforward_permille*sign+12+50;
    assert(fabsf(s.command_permille*sign-bound)<=.501f && bound<300);
    printf("AUTHORITY,%u,%d,%.9g,%.9g,%.9g,%d\n",wheel,sign,
      s.feedforward_permille*sign,s.proportional_permille*sign,
      s.integrator_permille*sign,s.command_permille*sign);
#endif
    printf("P_GAIN,%u,%d,%.9g,%.9g\n",wheel,sign,
      under.proportional_permille*sign,over.proportional_permille*sign);
  }
  assert(!depth);
  puts("PASS: real estimator decay/timeout and controller gain/effort assertions, prescribed input only");
  return 0;
}
