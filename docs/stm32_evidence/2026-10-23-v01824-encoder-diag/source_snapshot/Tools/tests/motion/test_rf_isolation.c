/* Link frozen .16 with prefixed public symbols. No serial/RTOS/GPIO. */
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

static void rf_moving_gap(float sign)
{
  const MotorDriverWheel w=MOTOR_DRIVER_WHEEL_RF;
  assert(WheelSpeedController_Reset(w) && Baseline_WheelSpeedController_Reset(w));
  int16_t a,b;
  for(unsigned k=0;k<200;k++)
  {
    assert(WheelSpeedController_UpdateWithMode(w,sign*240,sign*500,0,
      WHEEL_SPEED_CONTROL_MODE_PIVOT,20,&a));
    assert(Baseline_WheelSpeedController_UpdateWithMode(w,sign*240,sign*500,0,
      WHEEL_SPEED_CONTROL_MODE_PIVOT,20,&b));
  }
  WheelSpeedControllerState ca,cb;
  WheelSpeedFrontPivotState fa,fb;
  assert(WheelSpeedController_GetState(w,&ca));
  assert(Baseline_WheelSpeedController_GetState(w,&cb));
  assert(WheelSpeedController_GetFrontPivotState(w,&fa));
  assert(Baseline_WheelSpeedController_GetFrontPivotState(w,&fb));
  assert(fa.load_permille==0 && fb.load_permille==0);
  assert(fabsf(WheelSpeedController_ReleaseFeedforwardBias(w,-ca.integrator_permille))<0.001f);
  assert(fabsf(Baseline_WheelSpeedController_ReleaseFeedforwardBias(w,-cb.integrator_permille))<0.001f);
  assert(WheelSpeedController_PrimeOutput(w,(int16_t)(sign*100)));
  assert(Baseline_WheelSpeedController_PrimeOutput(w,(int16_t)(sign*100)));
  assert(WheelSpeedController_UpdateWithMode(w,sign*240,sign*180,0,
    WHEEL_SPEED_CONTROL_MODE_PIVOT,20,&a));
  assert(Baseline_WheelSpeedController_UpdateWithMode(w,sign*240,sign*180,0,
    WHEEL_SPEED_CONTROL_MODE_PIVOT,20,&b));
  assert(WheelSpeedController_GetState(w,&ca));
  assert(Baseline_WheelSpeedController_GetState(w,&cb));
  assert(WheelSpeedController_GetFrontPivotState(w,&fa));
  assert(Baseline_WheelSpeedController_GetFrontPivotState(w,&fb));
  assert(fabsf(fa.load_permille-0.72f)<0.001f && ca.integrator_permille==0);
  assert(fb.load_permille==0 && fabsf(cb.integrator_permille*sign-0.3f)<0.001f);
  assert(!fa.recovering && fa.still_ms==0 && !fa.recovery_failed);
}

int main(void)
{
  const float target[]={0,.5f,1,19.99f,20,40,80,160,240,280,1600,2000};
  const float feedback[]={0,4,19,21,49,50,100,181,250,500,2000,-2000};
  const unsigned periods[]={0,1,10,20,59,60,100,101};
  WheelSpeedController_Init(); Baseline_WheelSpeedController_Init();
  for(unsigned w=0;w<4;w++) for(unsigned k=0;k<20000;k++)
  {
    const MotorDriverWheel wheel=(MotorDriverWheel)w;
    float sign=k/121%2 ? -1.0f : 1.0f;
    /* RF PIVOT is intentionally changed; unaffected histories stay equal.
     * LF/rear alternate modes, both signs and below/above policy thresholds. */
    WheelSpeedControlMode mode=w==MOTOR_DRIVER_WHEEL_RF || k/811%2 ?
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
    const float t=sign*target[k/17%12], m=sign*feedback[k/37%12];
    const float bias=(k/101%3-1.0f)*20;
    const bool ok=WheelSpeedController_UpdateWithMode(wheel,t,m,bias,mode,periods[k%8],&a);
    assert(ok==Baseline_WheelSpeedController_UpdateWithMode(wheel,t,m,bias,mode,periods[k%8],&b));
    assert(a==b);
    same_state(wheel);
    if(k%701==0)
    {
      assert(!WheelSpeedController_UpdateWithMode(wheel,NAN,m,bias,mode,20,&a));
      assert(!Baseline_WheelSpeedController_UpdateWithMode(wheel,NAN,m,bias,mode,20,&b));
      assert(!WheelSpeedController_UpdateWithMode(wheel,t,INFINITY,bias,mode,20,&a));
      assert(!Baseline_WheelSpeedController_UpdateWithMode(wheel,t,INFINITY,bias,mode,20,&b));
      same_state(wheel);
    }
  }
  puts("PASS: 80000 adversarial updates: LF/rear NORMAL+PIVOT and RF NORMAL bit-identical to frozen .16; reset/prime/bias/invalid inputs included");
  rf_moving_gap(1); rf_moving_gap(-1);
  puts("PASS: RF both directions, 240 target/180 feedback/zero I: .17 restores load +0.72 without positive I; frozen .16 leaves load zero and integrates +0.3");
  return 0;
}
