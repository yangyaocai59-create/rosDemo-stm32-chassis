/* Prescribed feedback only: proves accounting and guards, NOT motor stability. */
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include "wheel_speed_controller.h"
#include "wheel_speed_control_config.h"

static WheelSpeedControllerState step(MotorDriverWheel w,float t,float m,unsigned dt)
{
  int16_t pwm;
  WheelSpeedControllerState s;
  WheelSpeedFrontPivotState f;
  assert(WheelSpeedController_UpdateWithMode(w,t,m,0,
    WHEEL_SPEED_CONTROL_MODE_PIVOT,dt,&pwm));
  assert(WheelSpeedController_GetState(w,&s));
  assert(WheelSpeedController_GetFrontPivotState(w,&f));
  assert(abs(pwm)<=300 && fabsf(s.integrator_permille)<=50.001f);
  assert(s.integrator_permille*f.sign+f.recovery_permille<=50.001f);
  assert(f.recovery_permille>=0 && f.recovery_permille<=
    50*fminf(fabsf(t)/160,1)+0.001f);
  return s;
}

static float effort(WheelSpeedControllerState s,float sign)
{ return (s.feedforward_permille+s.integrator_permille)*sign; }

static void test_handoff(MotorDriverWheel w,float sign,float magnitude,unsigned dt)
{
  const float t=sign*magnitude;
  assert(WheelSpeedController_Reset(w));
  for(unsigned k=0;k<1000/dt+1;k++) (void)step(w,t,t,dt);
  assert(fabsf(WheelSpeedController_ReleaseFeedforwardBias(w,sign*20))<0.001f);
  WheelSpeedControllerState s;
  WheelSpeedFrontPivotState f;
  float previous_i=20;
  for(unsigned elapsed=0;elapsed<200;elapsed+=dt)
  {
    s=step(w,t,0,dt);
    assert(s.integrator_permille*sign>=previous_i-0.001f);
    previous_i=s.integrator_permille*sign;
  }
  assert(WheelSpeedController_GetFrontPivotState(w,&f));
  assert(f.recovering && f.recovery_permille>0);
  /* >=50 cps movement and still underspeed. No gap even on the last
   * transfer, and no new PI increment on that final transfer's update. */
  const float moving=fmaxf(50,magnitude*0.625f);
  float previous_effort=effort(s,sign);
  unsigned transfers=0;
  for(unsigned elapsed=0;elapsed<2500;elapsed+=dt)
  {
    const float old_rec=f.recovery_permille;
    s=step(w,t,sign*moving,dt);
    assert(WheelSpeedController_GetFrontPivotState(w,&f));
    const float current=effort(s,sign);
    assert(current>=previous_effort-0.002f);
    if(!f.recovering && old_rec>0)
    {
      assert(fabsf(current-previous_effort)<0.002f);
      assert(old_rec-f.recovery_permille<=40*dt/1000.0f+0.002f);
      ++transfers;
    }
    previous_effort=current;
  }
  if(!(transfers>0 && f.recovery_permille==0 && !f.recovering))
    fprintf(stderr,"handoff w=%d sign=%.0f t=%.0f dt=%u transfers=%u rec=%.9f I=%.9f\n",
      w,sign,magnitude,dt,transfers,f.recovery_permille,s.integrator_permille*sign);
  assert(transfers>0 && f.recovery_permille==0 && !f.recovering);
  /* Same positive I must also be withdrawable, not an unload floor. */
  for(unsigned elapsed=0;elapsed<2500;elapsed+=dt)
    s=step(w,t,sign*(magnitude+fmaxf(100,magnitude*0.2f)),dt);
  assert(WheelSpeedController_GetFrontPivotState(w,&f));
  assert(f.load_permille==0 && f.recovery_permille==0);
  assert(s.integrator_permille*sign<=0);
}

static void test_timeout(MotorDriverWheel w,float sign,unsigned dt)
{
  assert(WheelSpeedController_Reset(w));
  for(unsigned elapsed=0;elapsed<2000;elapsed+=dt) (void)step(w,sign*160,0,dt);
  WheelSpeedFrontPivotState f;
  assert(WheelSpeedController_GetFrontPivotState(w,&f));
  assert(f.recovery_failed && !f.recovering && f.recovery_permille==0);
  assert(WheelSpeedController_Reset(w));
  for(unsigned elapsed=0;elapsed<200;elapsed+=dt) (void)step(w,sign*160,0,dt);
  /* Unconfirmed motion must not turn the short pulse into indefinite support. */
  for(unsigned elapsed=0;elapsed<1000;elapsed+=dt) (void)step(w,sign*160,sign*30,dt);
  assert(WheelSpeedController_GetFrontPivotState(w,&f));
  assert(!f.recovering && f.recovery_permille==0);
}

static void test_withdraw_budget(MotorDriverWheel w,float sign)
{
  assert(WheelSpeedController_Reset(w));
  WheelSpeedFrontPivotState f;
  WheelSpeedControllerState s;
  const float target=sign*160;
  const float budget=w==MOTOR_DRIVER_WHEEL_LF ? 0.4f : 4.0f;
  for(unsigned k=0;k<20;k++) (void)step(w,target,target,20);
  /* Approach the last partial (or zero) load bucket, then seed positive I
   * through the public bumpless API. One policy-specific budget spans BOTH. */
  do {
    s=step(w,target,sign*200,20);
    assert(WheelSpeedController_GetFrontPivotState(w,&f));
  } while(f.load_permille>=budget);
  const float load=f.load_permille;
  assert(fabsf(WheelSpeedController_ReleaseFeedforwardBias(w,
    sign*20-s.integrator_permille))<0.001f);
  assert(WheelSpeedController_PrimeOutput(w,(int16_t)(sign*90)));
  s=step(w,target,sign*200,20);
  assert(WheelSpeedController_GetFrontPivotState(w,&f));
  assert(f.load_permille==0);
  /* Plus the unchanged ordinary negative-error I correction: -0.2. */
  assert(fabsf(s.integrator_permille*sign-(20-(budget-load)-0.2f))<0.002f);
}

int main(void)
{
  const MotorDriverWheel wheels[]={MOTOR_DRIVER_WHEEL_LF,MOTOR_DRIVER_WHEEL_RF};
  const unsigned periods[]={1,10,20,25,59,60,100};
  const float magnitudes[]={80,160,280,1600};
  WheelSpeedController_Init();
  for(unsigned w=0;w<2;w++) for(int sign=-1;sign<=1;sign+=2)
    test_withdraw_budget(wheels[w],(float)sign);
  for(unsigned w=0;w<2;w++) for(int sign=-1;sign<=1;sign+=2)
    for(unsigned d=0;d<sizeof(periods)/sizeof(periods[0]);d++)
    {
      for(unsigned m=0;m<sizeof(magnitudes)/sizeof(magnitudes[0]);m++)
        test_handoff(wheels[w],(float)sign,magnitudes[m],periods[d]);
      test_timeout(wheels[w],(float)sign,periods[d]);
    }
  puts("PASS: front handoff preserves positive steady I, conserves FF+I during bounded transfer, shares 50-permille authority, no last-transfer double integration");
  puts("PASS: inherited steady I and temporary support withdraw on sustained overspeed; unconfirmed movement fades; continuous rest still fails; both fronts/signs, variable dt");
  return 0;
}
