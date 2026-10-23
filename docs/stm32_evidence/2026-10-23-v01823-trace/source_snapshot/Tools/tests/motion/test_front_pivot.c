/* Real front policy + PI, prescribed feedback, no hardware/physical plant. */
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include "wheel_speed_controller.h"
#include "wheel_speed_control_config.h"

static WheelSpeedControllerState Step(MotorDriverWheel w,float target,
  float measured,uint32_t dt)
{
  WheelSpeedControllerState before,after;
  WheelSpeedFrontPivotState f;
  int16_t pwm;
  assert(WheelSpeedController_GetState(w,&before));
  assert(WheelSpeedController_UpdateWithMode(w,target,measured,0,
    WHEEL_SPEED_CONTROL_MODE_PIVOT,dt,&pwm));
  assert(WheelSpeedController_GetState(w,&after));
  assert(WheelSpeedController_GetFrontPivotState(w,&f));
  assert(abs(pwm)<=300 && fabsf(after.integrator_permille)<=50.001f);
  assert(abs(pwm-before.command_permille)<=(int)dt+1);
  if(f.active)
  {
    const float t=fmaxf(-1600.0f,fminf(target,1600.0f));
    const float cap=fabsf(WheelSpeedController_GetPivotLoadFeedforward(w,t));
    assert(f.load_permille>=0 && f.load_permille<=cap+0.001f);
    assert(f.recovery_permille>=0 && f.recovery_permille<=50.0f*fminf(fabsf(t)/160.0f,1.0f)+0.001f);
    assert(after.integrator_permille*(float)f.sign+f.recovery_permille<=50.001f);
    assert(f.still_ms<=60 && f.moving_ms<=40 && f.recovery_ms<=300);
    assert(f.no_motion_ms<=WHEEL_SPEED_FRONT_NO_MOTION_LIMIT_MS);
    assert(f.load_overspeed_ms<=WHEEL_SPEED_FRONT_LOAD_OVERSPEED_CONFIRM_MS);
    if(f.recovery_failed)
      assert(f.no_motion_ms==WHEEL_SPEED_FRONT_NO_MOTION_LIMIT_MS &&
             !f.recovering && f.recovery_permille==0);
  }
  return after;
}

static void TestUnloadAndRebase(MotorDriverWheel w,float sign)
{
  const float target=sign*280.0f;
  assert(WheelSpeedController_Reset(w));
  for(unsigned k=0;k<20;k++) (void)Step(w,target,target,20);
  assert(fabsf(WheelSpeedController_ReleaseFeedforwardBias(w,-sign*30))<0.001f);
  assert(WheelSpeedController_PrimeOutput(w,(int16_t)(sign*100)));
  WheelSpeedControllerState before,after;
  WheelSpeedFrontPivotState f;
  assert(WheelSpeedController_GetState(w,&before));
  after=Step(w,target,sign*461.0f,20);
  /* During qualification there is no load withdrawal, only normal error I. */
  assert(fabsf((after.feedforward_permille+after.integrator_permille)-
    (before.feedforward_permille+before.integrator_permille)-sign*(-0.905f))<0.002f);
  assert(WheelSpeedController_GetFrontPivotState(w,&f));
  const float ceiling=fabsf(WheelSpeedController_GetPivotLoadFeedforward(w,target));
  assert(fabsf(f.load_permille-ceiling)<0.001f && f.load_overspeed_ms==20);
  (void)Step(w,target,sign*461.0f,20);
  assert(WheelSpeedController_GetFrontPivotState(w,&f));
  assert(fabsf(f.load_permille-ceiling)<0.001f && f.load_overspeed_ms==40);
  assert(WheelSpeedController_GetState(w,&before));
  after=Step(w,target,sign*461.0f,20);
  assert(WheelSpeedController_GetFrontPivotState(w,&f));
  const float withdraw=3.06f;
  assert(fabsf(f.load_permille-(ceiling-withdraw))<0.001f && f.load_overspeed_ms==60);
  /* Once qualified, withdrawing load and consuming opposing I cancels;
   * only the usual error I increment changes their combined support. */
  assert(fabsf((after.feedforward_permille+after.integrator_permille)-
    (before.feedforward_permille+before.integrator_permille)-sign*(-0.905f))<0.002f);
  for(unsigned k=0;k<100;k++) (void)Step(w,target,sign*461.0f,20);
  assert(WheelSpeedController_GetFrontPivotState(w,&f));
  assert(f.load_permille==0 && f.recovery_permille==0 && !f.recovering);

  /* Both fronts restore bounded load (+0.8) and hold I on this update. */
  WheelSpeedControllerState state;
  assert(WheelSpeedController_GetState(w,&state));
  assert(fabsf(WheelSpeedController_ReleaseFeedforwardBias(w,
    sign*20-state.integrator_permille))<0.001f);
  assert(WheelSpeedController_PrimeOutput(w,(int16_t)(sign*130)));
  assert(WheelSpeedController_GetState(w,&before));
  after=Step(w,target,sign*100,20);
  assert(WheelSpeedController_GetFrontPivotState(w,&f));
  assert(fabsf(f.load_permille-0.8f)<0.001f);
  assert(fabsf(after.integrator_permille*sign-20.0f)<0.002f);
  assert(fabsf((after.feedforward_permille+after.integrator_permille)-
    (before.feedforward_permille+before.integrator_permille)-
    sign*0.8f)<0.002f);
}

static void TestRecovery(MotorDriverWheel w,float sign,float seed)
{
  const float target=sign*280;
  assert(WheelSpeedController_Reset(w));
  for(unsigned k=0;k<20;k++) (void)Step(w,target,target,20);
  assert(fabsf(WheelSpeedController_ReleaseFeedforwardBias(w,sign*seed))<0.001f);
  WheelSpeedFrontPivotState f;
  (void)Step(w,target,0,20);
  assert(WheelSpeedController_GetFrontPivotState(w,&f));
  assert(!f.recovering && f.still_ms==20 && f.recovery_permille==0);
  (void)Step(w,target,0,20);
  assert(WheelSpeedController_GetFrontPivotState(w,&f));
  assert(!f.recovering && f.still_ms==40);
  WheelSpeedControllerState before_entry;
  assert(WheelSpeedController_GetState(w,&before_entry));
  const float retained=fmaxf(0,before_entry.integrator_permille*sign);
  WheelSpeedControllerState state=Step(w,target,0,20);
  assert(WheelSpeedController_GetFrontPivotState(w,&f));
  assert(f.recovering && f.exhausted && f.still_ms==60);
  assert(fabsf(state.integrator_permille*sign-retained)<0.001f);
  assert(f.recovery_permille>0 || retained==50);
  for(unsigned k=0;k<5;k++)
  {
    state=Step(w,target,0,20);
    assert(fabsf(state.integrator_permille*sign-retained)<0.001f);
  }
  (void)Step(w,target,target,20);
  assert(WheelSpeedController_GetFrontPivotState(w,&f));
  assert(f.moving_ms==20 && f.recovering);
  state=Step(w,target,target,20);
  assert(WheelSpeedController_GetFrontPivotState(w,&f));
  assert(f.moving_ms==40 && !f.recovering && !f.exhausted);
  const float support=(state.feedforward_permille+state.integrator_permille)*sign;
  for(unsigned k=0;k<80;k++)
  {
    state=Step(w,target,target,20);
    assert(fabsf((state.feedforward_permille+state.integrator_permille)*sign-support)<0.002f);
  }
  assert(WheelSpeedController_GetFrontPivotState(w,&f));
  assert(f.recovery_permille==0);
  /* Actual motion re-arms one later episode, never stacks a second full kick. */
  for(unsigned k=0;k<3;k++) (void)Step(w,target,0,20);
  assert(WheelSpeedController_GetFrontPivotState(w,&f) && f.recovering);
}

static void TestTimeoutAndQualification(MotorDriverWheel w,float sign)
{
  const float target=sign*160;
  assert(WheelSpeedController_Reset(w));
  (void)Step(w,target,0,20); (void)Step(w,target,0,20);
  (void)Step(w,target,sign*21,20); /* resets the consecutive near-rest timer */
  (void)Step(w,target,0,20);
  WheelSpeedFrontPivotState f;
  assert(WheelSpeedController_GetFrontPivotState(w,&f));
  assert(f.still_ms==20 && !f.recovering);
  /* Opposite rotation is not confirmed near-rest, even though signed speed<0. */
  for(unsigned k=0;k<20;k++) (void)Step(w,target,-sign*100,20);
  assert(WheelSpeedController_GetFrontPivotState(w,&f));
  assert(f.still_ms==0 && !f.recovering);
  for(unsigned k=0;k<3;k++) (void)Step(w,target,0,20);
  assert(WheelSpeedController_GetFrontPivotState(w,&f) && f.recovering);
  /* 100ms real elapsed steps saturate the timer, no deadline catch-up loop. */
  for(unsigned k=0;k<3;k++) (void)Step(w,target,0,100);
  assert(WheelSpeedController_GetFrontPivotState(w,&f));
  assert(!f.recovering && f.exhausted && f.recovery_ms==300);
  for(unsigned k=0;k<10;k++) (void)Step(w,target,0,20);
  assert(WheelSpeedController_GetFrontPivotState(w,&f));
  assert(f.recovery_permille==0 && f.exhausted);
  WheelSpeedControllerState held;
  assert(WheelSpeedController_GetState(w,&held));
  for(unsigned k=0;k<80;k++)
  {
    WheelSpeedControllerState state=Step(w,target,0,20);
    assert(WheelSpeedController_GetFrontPivotState(w,&f));
    assert(!f.recovering && f.exhausted && f.recovery_permille==0);
    assert(fabsf(state.integrator_permille-held.integrator_permille)<0.001f);
  }
}

static void TestSequenceAndReset(MotorDriverWheel w)
{
  const float magnitudes[]={0,0.5f,1,19.99f,20,40,80,160,280,1600,2000};
  const float feedback[]={0,19,21,49,50,100,461,2000,-2000};
  const uint32_t dt[]={1,10,20,100};
  assert(WheelSpeedController_Reset(w));
  for(unsigned k=0;k<4000;k++)
  {
    float sign=(k/117)%2 ? -1.0f : 1.0f;
    (void)Step(w,sign*magnitudes[(k/31)%11],sign*feedback[(k/13)%9],dt[k%4]);
  }
  WheelSpeedFrontPivotState f;
  WheelSpeedControllerState state;
  int16_t pwm;
  assert(WheelSpeedController_Update(w,280,0,20,&pwm));
  assert(WheelSpeedController_GetFrontPivotState(w,&f) && !f.active);
  for(unsigned k=0;k<20;k++) (void)Step(w,280,0,20);
  (void)Step(w,-280,0,20);
  assert(WheelSpeedController_GetFrontPivotState(w,&f));
  assert(f.sign==-1 && f.still_ms==20 && f.recovery_permille==0);
  (void)Step(w,0,0,20);
  assert(WheelSpeedController_GetFrontPivotState(w,&f) && !f.active);
  assert(WheelSpeedController_Reset(w));
  assert(WheelSpeedController_GetFrontPivotState(w,&f) && !f.active);
  assert(WheelSpeedController_GetState(w,&state) && state.command_permille==0);
  pwm=1234;
  assert(!WheelSpeedController_UpdateWithMode(w,NAN,0,0,WHEEL_SPEED_CONTROL_MODE_PIVOT,20,&pwm) && pwm==1234);
  assert(!WheelSpeedController_UpdateWithMode(w,280,INFINITY,0,WHEEL_SPEED_CONTROL_MODE_PIVOT,20,&pwm));
  assert(!WheelSpeedController_UpdateWithMode(w,280,0,0,WHEEL_SPEED_CONTROL_MODE_PIVOT,0,&pwm));
  assert(!WheelSpeedController_UpdateWithMode(w,280,0,0,WHEEL_SPEED_CONTROL_MODE_PIVOT,101,&pwm));
  assert(WheelSpeedController_GetFrontPivotState(w,&f) && !f.active);
}

static void TestLowTargetWithRetainedIntegral(MotorDriverWheel w,float sign)
{
  const float target=sign*20;
  assert(WheelSpeedController_Reset(w));
  for(unsigned k=0;k<20;k++) (void)Step(w,target,target,20);
  assert(fabsf(WheelSpeedController_ReleaseFeedforwardBias(w,sign*50))<0.001f);
  (void)Step(w,target,0,20); (void)Step(w,target,0,20);
  WheelSpeedControllerState state=Step(w,target,0,20);
  WheelSpeedFrontPivotState f;
  assert(WheelSpeedController_GetFrontPivotState(w,&f));
  assert(f.recovering && f.recovery_permille==0);
  assert(fabsf(state.integrator_permille*sign-50)<0.001f);
  assert(fabsf(state.integrator_permille*sign+f.recovery_permille-50)<0.001f);
}

static void TestFailureDeadline(MotorDriverWheel w,float sign)
{
  const float magnitudes[]={20,40,160,199,200,280};
  const uint32_t periods[]={1,10,20,100};
  for(unsigned m=0;m<sizeof(magnitudes)/sizeof(magnitudes[0]);m++)
    for(unsigned d=0;d<sizeof(periods)/sizeof(periods[0]);d++)
    {
      assert(WheelSpeedController_Reset(w));
      const float target=sign*magnitudes[m];
      WheelSpeedFrontPivotState f;
      const uint32_t dt=periods[d];
      for(uint32_t elapsed=dt;elapsed<=1500;elapsed+=dt)
      {
        (void)Step(w,target,0,dt);
        assert(WheelSpeedController_GetFrontPivotState(w,&f));
        assert(f.recovery_failed==(elapsed==1500));
      }
      /* A later moving sample must not erase an already-qualified failure
       * before the safety task can consume it. No second recovery pulse. */
      for(unsigned k=0;k<20;k++) (void)Step(w,target,target,dt);
      for(unsigned k=0;k<20;k++) (void)Step(w,target,0,dt);
      assert(WheelSpeedController_GetFrontPivotState(w,&f));
      assert(f.recovery_failed && !f.recovering && f.recovery_permille==0);
      (void)Step(w,-target,0,dt);
      assert(WheelSpeedController_GetFrontPivotState(w,&f));
      assert(!f.recovery_failed && f.no_motion_ms==dt);
    }

  /* Interrupting near-rest starts a fresh deadline, not lifetime low-speed
   * accumulation. Measured motion here is prescribed, not a physical plant. */
  assert(WheelSpeedController_Reset(w));
  for(unsigned k=0;k<70;k++) (void)Step(w,sign*160,0,20);
  (void)Step(w,sign*160,sign*21,20);
  WheelSpeedFrontPivotState f;
  assert(WheelSpeedController_GetFrontPivotState(w,&f));
  assert(!f.recovery_failed && f.no_motion_ms==0);
  for(unsigned k=0;k<74;k++) (void)Step(w,sign*160,0,20);
  assert(WheelSpeedController_GetFrontPivotState(w,&f) && !f.recovery_failed);
  (void)Step(w,sign*160,0,20);
  assert(WheelSpeedController_GetFrontPivotState(w,&f) && f.recovery_failed);
  int16_t pwm=0;
  assert(WheelSpeedController_Update(w,sign*160,0,20,&pwm));
  assert(WheelSpeedController_GetFrontPivotState(w,&f));
  assert(!f.active && !f.recovery_failed && f.no_motion_ms==0);

  /* Tiny/normal targets are outside this policy; valid low targets at their
   * actual speed must never be labelled stationary because of quantization. */
  assert(WheelSpeedController_Reset(w));
  for(unsigned k=0;k<200;k++) (void)Step(w,sign*20,sign*20,20);
  assert(WheelSpeedController_GetFrontPivotState(w,&f));
  assert(!f.recovery_failed && f.no_motion_ms==0);
  for(unsigned k=0;k<200;k++) (void)Step(w,sign*19,0,20);
  assert(WheelSpeedController_GetFrontPivotState(w,&f) && !f.active);
}

int main(void)
{
  const MotorDriverWheel wheels[]={MOTOR_DRIVER_WHEEL_LF,MOTOR_DRIVER_WHEEL_RF};
  const float seeds[]={-30,0,29.4f,50};
  WheelSpeedController_Init();
  for(unsigned w=0;w<2;w++)
  {
    for(int sign=-1;sign<=1;sign+=2)
    {
      TestUnloadAndRebase(wheels[w],(float)sign);
      for(unsigned k=0;k<4;k++) TestRecovery(wheels[w],(float)sign,seeds[k]);
      TestTimeoutAndQualification(wheels[w],(float)sign);
      TestLowTargetWithRetainedIntegral(wheels[w],(float)sign);
      TestFailureDeadline(wheels[w],(float)sign);
    }
    TestSequenceAndReset(wheels[w]);
  }
  WheelSpeedFrontPivotState f;
  assert(!WheelSpeedController_GetFrontPivotState(MOTOR_DRIVER_WHEEL_LR,&f));
  assert(!WheelSpeedController_GetFrontPivotState(MOTOR_DRIVER_WHEEL_RR,&f));
  assert(!WheelSpeedController_GetFrontPivotState(MOTOR_DRIVER_WHEEL_COUNT,&f));
  assert(!WheelSpeedController_GetFrontPivotState(MOTOR_DRIVER_WHEEL_LF,NULL));
  puts("PASS: both fronts restore bounded moving load; opposing-I debt and recovery handoff conserve support; no duplicate positive I");
  puts("PASS: 60ms consecutive near-rest, 40ms moving rearm, 300ms timeout+finite fade, no repeated retries at rest, no boost on opposite motion");
  puts("PASS: 8000 adversarial target/feedback/dt steps obey load/transient/I/output/slew bounds; sign/mode/zero/reset and invalid inputs clear/preserve state correctly");
  puts("PASS: both fronts/signs at 20/40/160/199/200/280 cps and 1/10/20/100ms dt qualify failure at 1500ms, sticky until reset/sign/mode exit; real movement interrupts qualification");
  return 0;
}
