/* Link frozen .17 with prefixed public symbols and the current controller. */
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "wheel_speed_controller.h"

void Baseline_WheelSpeedController_Init(void);
bool Baseline_WheelSpeedController_Reset(MotorDriverWheel);
bool Baseline_WheelSpeedController_PrimeOutput(MotorDriverWheel,int16_t);
float Baseline_WheelSpeedController_ReleaseFeedforwardBias(MotorDriverWheel,float);
bool Baseline_WheelSpeedController_UpdateWithMode(MotorDriverWheel,float,float,float,WheelSpeedControlMode,uint32_t,int16_t*);
bool Baseline_WheelSpeedController_GetState(MotorDriverWheel,WheelSpeedControllerState*);
bool Baseline_WheelSpeedController_GetFrontPivotState(MotorDriverWheel,WheelSpeedFrontPivotState*);

static void same_state(MotorDriverWheel w)
{
  WheelSpeedControllerState a,b;
  assert(WheelSpeedController_GetState(w,&a));
  assert(Baseline_WheelSpeedController_GetState(w,&b));
  assert(memcmp(&a.target_counts_per_second,&b.target_counts_per_second,6*sizeof(float))==0);
  assert(a.command_permille==b.command_permille && a.limited==b.limited);
  if(w==MOTOR_DRIVER_WHEEL_RF || w==MOTOR_DRIVER_WHEEL_LF)
  {
    WheelSpeedFrontPivotState fa,fb;
    assert(WheelSpeedController_GetFrontPivotState(w,&fa));
    assert(Baseline_WheelSpeedController_GetFrontPivotState(w,&fb));
    assert(memcmp(&fa.load_permille,&fb.load_permille,2*sizeof(float))==0);
    assert(memcmp(&fa.still_ms,&fb.still_ms,5*sizeof(uint32_t))==0);
    assert(fa.sign==fb.sign && fa.active==fb.active && fa.recovering==fb.recovering &&
      fa.exhausted==fb.exhausted && fa.recovery_failed==fb.recovery_failed);
  }
}
int main(void)
{
  const float target[]={0,.5f,1,19.99f,20,40,80,160,280,1600,2000};
  const float feedback[]={0,4,19,21,49,50,100,181,250,500,2000,-2000};
  const unsigned periods[]={0,1,10,20,59,60,100,101};
  WheelSpeedController_Init(); Baseline_WheelSpeedController_Init();
  for(unsigned w=0;w<4;w++) for(unsigned k=0;k<20000;k++)
  {
    const MotorDriverWheel wheel=(MotorDriverWheel)w;
    float sign=k/121%2 ? -1.0f : 1.0f;
    WheelSpeedControlMode mode=(w==0 && sign<0) || k/811%2 ?
      WHEEL_SPEED_CONTROL_MODE_NORMAL : WHEEL_SPEED_CONTROL_MODE_PIVOT;
    if(k%311==0)
      assert(WheelSpeedController_Reset(wheel)==Baseline_WheelSpeedController_Reset(wheel));
    if(k%127==0)
      assert(WheelSpeedController_PrimeOutput(wheel,(int16_t)(sign*120))==
        Baseline_WheelSpeedController_PrimeOutput(wheel,(int16_t)(sign*120)));
    if(k%293==0)
      assert(WheelSpeedController_ReleaseFeedforwardBias(wheel,sign*23.7f)==
        Baseline_WheelSpeedController_ReleaseFeedforwardBias(wheel,sign*23.7f));
    int16_t a=1234,b=1234;
    const float t=sign*target[k/17%11], m=sign*feedback[k/37%12];
    const float bias=(k/101%3-1.0f)*20;
    const bool ok=WheelSpeedController_UpdateWithMode(wheel,t,m,bias,mode,periods[k%8],&a);
    assert(ok==Baseline_WheelSpeedController_UpdateWithMode(wheel,t,m,bias,mode,periods[k%8],&b));
    assert(a==b);
    same_state(wheel);
  }
  puts("PASS: 80000 updates: RF/rear both modes, LF NORMAL/forward PIVOT bit-identical to frozen .17; reset/prime/bias/invalid dt included");
  for(int sign=-1;sign<=1;sign+=2)
  {
    const MotorDriverWheel w=MOTOR_DRIVER_WHEEL_LF;
    WheelSpeedController_Reset(w); Baseline_WheelSpeedController_Reset(w);
    for(unsigned k=0;k<20005;k++)
    {
      int16_t a,b;
      const float t=sign*(k<5 ? 160 : target[4+k/17%7]);
      const float m=sign*(k<5 ? 160 : feedback[k/37%12]);
      assert(WheelSpeedController_UpdateWithMode(w,t,m,0,WHEEL_SPEED_CONTROL_MODE_PIVOT,20,&a));
      assert(Baseline_WheelSpeedController_UpdateWithMode(w,t,m,0,WHEEL_SPEED_CONTROL_MODE_PIVOT,20,&b));
      assert(a==b); same_state(w);
    }
  }
  puts("PASS: 40010 LF bidirectional PIVOT updates after confirmed motion bit-identical to frozen .17, including later stall/recovery/overspeed");
  return 0;
}
