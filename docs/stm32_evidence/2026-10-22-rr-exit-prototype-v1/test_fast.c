#include "fixture.h"
static uint32_t tick;
static void seed(uint32_t base)
{
  WheelSpeedController_Init();
  for(unsigned w=0;w<4;w++) ChassisController_ClearPivotWheel((MotorDriverWheel)w);
  tick=base;
  int16_t pwm;
  for(unsigned k=0;k<2;k++) assert(WheelSpeedController_UpdateWithMode(
    MOTOR_DRIVER_WHEEL_RR,240,240,0,WHEEL_SPEED_CONTROL_MODE_PIVOT,20,&pwm));
  assert(WheelSpeedController_SelectRrRunningSupport(true));
  for(unsigned k=0;k<10;k++)
    assert(WheelSpeedController_UpdateWithMode(MOTOR_DRIVER_WHEEL_RR,240,100,0,
      WHEEL_SPEED_CONTROL_MODE_PIVOT,20,&pwm));
  assert(s_rr_running.temporary_permille>20);
}
static void step(int32_t delta,float measured,unsigned dt)
{
  tick+=dt;
  WheelSpeedController_ObserveRrRecoveryCounts(delta,dt,tick,tick);
  const float before=s_states[3].integrator_permille+s_rr_running.temporary_permille;
  const int prior=s_states[3].command_permille;
  int16_t pwm;
  assert(WheelSpeedController_UpdateWithMode(MOTOR_DRIVER_WHEEL_RR,240,measured,0,
    WHEEL_SPEED_CONTROL_MODE_PIVOT,dt,&pwm));
  assert(abs(pwm)<=300 && abs(pwm-prior)<=(int)dt+1);
  assert(fabsf(s_states[3].integrator_permille)<=50.001f);
  assert(s_rr_running.temporary_permille>=0 &&
    fmaxf(0,s_states[3].integrator_permille)+s_rr_running.temporary_permille<=60.001f);
  assert(s_states[3].integrator_permille+s_rr_running.temporary_permille-before
         <=250.0f*dt/1000.0f+.001f);
}
int main(void)
{
  for(unsigned wrap=0;wrap<2;wrap++)
  {
    seed(wrap ? 0xFFFFFFD0U : 0);
    step(5,100,20); /* raw250, margin264: not overspeed evidence */
    step(7,150,20); /* raw350 and weighted mean300: first confirmation20 */
    assert(s_rr_exit.confirm_ms==20 && !s_rr_exit.exiting);
    const float a=s_rr_running.temporary_permille, i=s_states[3].integrator_permille;
    step(6,200,20);
    assert(s_rr_exit.exiting && !s_rr_running.active);
    assert(fabsf(s_rr_running.temporary_permille-fmaxf(0,a-8))<.001f);
    assert(s_states[3].integrator_permille==i);
    for(unsigned k=0;k<20;k++) step(7,220,20);
    assert(s_rr_running.temporary_permille==0 && s_rr_running.spent);
    assert(s_rr_exit.exiting);
    assert(WheelSpeedController_IsRrRunningSupportOwned(240));
    for(unsigned k=0;k<8;k++) step(5,240,20);
    assert(!s_rr_running.spent && !s_rr_exit.exiting);
  }
  puts("PASS: fresh two-sample window +40ms confirmation exits before filtered overspeed, bounded fade/no I copy, finite/no rest retry, ownership retained, settled rearm, tick wrap");
  for(unsigned pattern=0;pattern<5;pattern++)
  {
    seed(0);
    for(unsigned k=0;k<30;k++)
      step(pattern==0 ? (k==2 ? 20 : 0) :
           pattern==1 ? (k==2 ? 20 : 1) :
           pattern==2 ? (k%2 ? 7 : 1) :
           pattern==3 ? (k%2 ? 7 : -1) : 5,100,20);
    assert(!s_rr_exit.exiting);
  }
  puts("PASS: isolated spikes, following positive-low/zero/opposite samples, separated excursions and 250cps quantization do not qualify early exit");
  for(unsigned bad=0;bad<9;bad++)
  {
    seed(1000); step(7,100,20); step(7,100,20);
    assert(s_rr_exit.confirm_ms==20);
    const uint32_t stamp=tick;
    switch(bad)
    {
      case 0: WheelSpeedController_ObserveRrRecoveryCounts(7,20,stamp,stamp); break;
      case 1: WheelSpeedController_ObserveRrRecoveryCounts(7,20,stamp-20,stamp); break;
      case 2: WheelSpeedController_ObserveRrRecoveryCounts(7,20,stamp+20,stamp+41); break;
      case 3: WheelSpeedController_ObserveRrRecoveryCounts(7,20,stamp+20,stamp+19); break;
      case 4: WheelSpeedController_ObserveRrRecoveryCounts(7,0,stamp+20,stamp+20); break;
      case 5: WheelSpeedController_ObserveRrRecoveryCounts(7,100,stamp+20,stamp+20); break;
      case 6: WheelSpeedController_ObserveRrRecoveryCounts(INT32_MAX,20,stamp+20,stamp+20); break;
      case 7: WheelSpeedController_ObserveRrRecoveryCounts(INT32_MIN,20,stamp+20,stamp+20); break;
      default: break; /* no fresh observation */
    }
    int16_t pwm;
    assert(WheelSpeedController_UpdateWithMode(MOTOR_DRIVER_WHEEL_RR,240,100,0,
      WHEEL_SPEED_CONTROL_MODE_PIVOT,20,&pwm));
    assert(!s_rr_exit.exiting && s_rr_exit.confirm_ms==0 && !s_rr_exit.have_previous);
  }
  puts("PASS: duplicate/out-of-order/stale/future/zero-or-long-period/extreme/missing input cannot reuse confirmation; filtered fallback remains valid");
  const unsigned periods[]={1,10,15,20,25,30,31,59,60,100};
  for(unsigned d=0;d<10;d++)
  {
    seed(0);
    unsigned steps=0;
    for(;steps<8 && !s_rr_exit.exiting;steps++) step((int32_t)periods[d],100,periods[d]);
    if(periods[d]>=15 && periods[d]<=30) assert(s_rr_exit.exiting);
    else assert(!s_rr_exit.exiting);
  }
  puts("PASS: early channel accepts 15..30ms samples only; other valid control dt1..100ms safely use original filtered policy");
  seed(0); step(7,100,20); step(7,100,20);
  tick+=20; WheelSpeedController_ObserveRrRecoveryCounts(7,20,tick,tick);
  int16_t pwm;
  assert(WheelSpeedController_UpdateWithMode(MOTOR_DRIVER_WHEEL_RR,280,100,0,
    WHEEL_SPEED_CONTROL_MODE_PIVOT,20,&pwm));
  assert(!s_rr_exit.exiting && s_rr_exit.confirm_ms==0);
  assert(WheelSpeedController_Reset(MOTOR_DRIVER_WHEEL_RR));
  const WheelSpeedRrExitState zero={0};
  assert(memcmp(&zero,&s_rr_exit,sizeof(zero))==0);
  puts("PASS: target change resets evidence without resetting finite episode; RR reset clears all new history");
  seed(0);
  for(unsigned k=0;k<30000;k++)
  {
    if(k%613==0) seed(0xFFFFF000U);
    const int32_t counts[]={0,1,5,7,12,-1,INT32_MIN,INT32_MAX};
    const float speeds[]={0,19,50,100,220,240,400,-100};
    step(counts[k/17%8],speeds[k/37%8],periods[k%10]);
  }
  puts("PASS: 30000 adversarial count/filter/dt/wrap updates preserve shared authority, output/I/slew/rise bounds");
  const float targets[]={20,40,80,160,240,600,1600};
  for(unsigned t=0;t<7;t++) for(unsigned d=15;d<=30;d+=5)
  {
    WheelSpeedController_Init();
    for(unsigned k=0;k<2;k++) assert(WheelSpeedController_UpdateWithMode(
      MOTOR_DRIVER_WHEEL_RR,targets[t],fmaxf(50,targets[t]),0,
      WHEEL_SPEED_CONTROL_MODE_PIVOT,20,&pwm));
    assert(WheelSpeedController_SelectRrRunningSupport(true));
    for(unsigned k=0;k<10;k++) assert(WheelSpeedController_UpdateWithMode(
      MOTOR_DRIVER_WHEEL_RR,targets[t],0,0,WHEEL_SPEED_CONTROL_MODE_PIVOT,20,&pwm));
    assert(s_rr_running.temporary_permille>0);
    uint32_t now=0xFFFFFFE0U;
    const int32_t delta=(int32_t)ceilf((targets[t]+fmaxf(20,targets[t]*.1f)+50)*d/1000);
    for(unsigned k=0;k<8 && !s_rr_exit.exiting;k++)
    {
      now+=d;WheelSpeedController_ObserveRrRecoveryCounts(delta,d,now,now);
      assert(WheelSpeedController_UpdateWithMode(MOTOR_DRIVER_WHEEL_RR,targets[t],0,0,
        WHEEL_SPEED_CONTROL_MODE_PIVOT,d,&pwm));
    }
    assert(s_rr_exit.exiting);
    for(unsigned k=0;k<100;k++)
    {
      now+=20;WheelSpeedController_ObserveRrRecoveryCounts(0,20,now,now);
      assert(WheelSpeedController_UpdateWithMode(MOTOR_DRIVER_WHEEL_RR,targets[t],0,0,
        WHEEL_SPEED_CONTROL_MODE_PIVOT,20,&pwm));
      assert(abs(pwm)<=300 && !s_rr_running.active && s_rr_running.spent);
      if(k>10) assert(s_rr_running.temporary_permille==0);
      assert(WheelSpeedController_IsRrRunningSupportOwned(targets[t]));
    }
  }
  puts("PASS: target20..1600 and sample15..30ms early-exit then renewed complete standstill: finite withdrawal, no temporary retry/rearm, persistent safety owner, PWM bound");
  return 0;
}
