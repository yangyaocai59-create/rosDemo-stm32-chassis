#include "lr_running_fixture.h"

static void ready(void)
{
  WheelSpeedController_Init();
  for(unsigned w=0;w<4;w++) ChassisController_ClearPivotWheel((MotorDriverWheel)w);
  int16_t pwm;
  for(unsigned k=0;k<2;k++) assert(WheelSpeedController_UpdateWithMode(
    MOTOR_DRIVER_WHEEL_LR,-240,-240,0,WHEEL_SPEED_CONTROL_MODE_PIVOT,20,&pwm));
  assert(WheelSpeedController_SelectLrRunningSupport(true));
}

static void step(float target,float measured,unsigned dt)
{
  const float before=-s_states[1].integrator_permille+s_lr_running.temporary_permille;
  const bool controlled=s_lr_running.active || s_lr_running.temporary_permille>0;
  const int previous=s_states[1].command_permille;
  int16_t pwm;
  assert(WheelSpeedController_UpdateWithMode(MOTOR_DRIVER_WHEEL_LR,target,measured,0,
    WHEEL_SPEED_CONTROL_MODE_PIVOT,dt,&pwm));
  const float after=-s_states[1].integrator_permille+s_lr_running.temporary_permille;
  assert(abs(pwm)<=300 && abs(pwm-previous)<=(int)dt+1);
  assert(fabsf(s_states[1].integrator_permille)<=50.001f);
  assert(s_lr_running.temporary_permille>=0 && s_lr_running.temporary_permille<=60.001f);
  assert(fmaxf(0,-s_states[1].integrator_permille)+s_lr_running.temporary_permille<=60.001f);
  if(target<=-20 && (controlled || s_lr_running.active || s_lr_running.temporary_permille>0))
    assert(after-before <= 250.0f*dt/1000.0f+0.001f);
  assert(s_lr_running.episode_ms<=600);
}

int main(void)
{
  const unsigned periods[]={1,10,20,25,59,60,100};
  const float targets[]={20,40,80,160,240,280,1600};
  for(unsigned d=0;d<7;d++) for(unsigned m=0;m<7;m++)
  {
    ready();
    const unsigned dt=periods[d];
    for(unsigned t=dt;t<=1200+dt;t+=dt)
    {
      step(-targets[m],0,dt);
      if(t<60) assert(!s_lr_running.spent && s_lr_running.temporary_permille==0);
      if(t>=900) assert(!s_lr_running.active && s_lr_running.temporary_permille==0);
    }
    assert(s_lr_running.spent);
    for(unsigned k=0;k<20;k++) step(-targets[m],0,dt);
    assert(s_lr_running.temporary_permille==0);
  }
  puts("PASS: LR reverse 60ms confirmation, one bounded episode/no rest retries, 600ms cutoff+finite fade, 20..1600cps dt1..100ms; I+temporary<=60, I/PWM/slew/rise caps");
  ready();
  for(unsigned k=0;k<2;k++) step(-240,-100,20);
  assert(!s_lr_running.spent);
  step(-240,-240,20);
  for(unsigned k=0;k<2;k++) step(-240,-100,20);
  assert(!s_lr_running.spent);
  step(-240,-100,20);
  assert(s_lr_running.active && s_lr_running.temporary_permille>0);
  const float before=s_lr_running.temporary_permille;
  step(-240,-500,20);
  assert(!s_lr_running.active && s_lr_running.temporary_permille<=fmaxf(0,before-8)+.001f);
  for(unsigned k=0;k<60;k++) step(-240,-240,20);
  assert(!s_lr_running.spent && s_lr_running.temporary_permille==0);
  puts("PASS: separated underspeed pulses cannot accumulate; FIRST overspeed stops rise/withdraws temporary; confirmed settled recovery rearms only after zero temporary");
  const float seeds[]={-50,-20,0,20,50};
  for(unsigned n=0;n<5;n++)
  {
    ready(); s_states[1].integrator_permille=seeds[n];
    for(unsigned k=0;k<30;k++) step(-240,-100,20);
    for(unsigned k=0;k<50;k++) step(-240,-400,20);
    assert(s_lr_running.temporary_permille==0);
  }
  puts("PASS: shared budget and conservative A-to-I reclassification cover opposing/zero/positive and saturated I; no full extra kick on retained I, overspeed fully unloads temporary");
  ready(); for(unsigned k=0;k<8;k++) step(-240,-100,20);
  const WheelSpeedLrRunningState saved=s_lr_running;
  int16_t pwm;
  assert(!WheelSpeedController_UpdateWithMode(MOTOR_DRIVER_WHEEL_LR,-240,0,0,WHEEL_SPEED_CONTROL_MODE_PIVOT,0,&pwm));
  assert(!WheelSpeedController_UpdateWithMode(MOTOR_DRIVER_WHEEL_LR,NAN,0,0,WHEEL_SPEED_CONTROL_MODE_PIVOT,20,&pwm));
  assert(!WheelSpeedController_UpdateWithMode(MOTOR_DRIVER_WHEEL_LR,-240,0,0,WHEEL_SPEED_CONTROL_MODE_PIVOT,101,&pwm));
  assert(!WheelSpeedController_UpdateWithMode(MOTOR_DRIVER_WHEEL_LR,-240,0,1,WHEEL_SPEED_CONTROL_MODE_PIVOT,20,&pwm));
  assert(memcmp(&saved,&s_lr_running,sizeof(saved))==0);
  assert(WheelSpeedController_Reset(MOTOR_DRIVER_WHEEL_RF));
  assert(memcmp(&saved,&s_lr_running,sizeof(saved))==0);
  step(-240,100,20); assert(!s_lr_running.active);
  step(240,100,20); assert(!s_lr_running.enabled);
  ready(); step(-19,0,20); assert(!s_lr_running.enabled);
  ready(); assert(WheelSpeedController_UpdateWithMode(MOTOR_DRIVER_WHEEL_LR,-240,0,0,WHEEL_SPEED_CONTROL_MODE_NORMAL,20,&pwm));
  assert(!s_lr_running.enabled);
  ready(); assert(WheelSpeedController_Reset(MOTOR_DRIVER_WHEEL_LR)); assert(!s_lr_running.enabled);
  puts("PASS: invalid dt/NaN/stacked caller bias preserve state; RF reset cannot affect LR; opposite motion ends rise; sign/low-target/mode/Reset clear new history");
  const float adversarial_targets[]={20,40,80,160,240,280,1000,1200,1600};
  const float adversarial_speed[]={0,19,21,49,50,100,181,250,500,1000,2000,-200};
  ready();
  for(unsigned k=0;k<30000;k++)
  {
    if(k%613==0) ready();
    step(-adversarial_targets[k/37%9],-adversarial_speed[k/19%12],periods[k%7]);
  }
  puts("PASS: 30000 adversarial LR running steps: target changes do not reset finite episode; output/I/shared authority/controlled-rise bounds across opposite motion and dt1..100ms");
  return 0;
}
