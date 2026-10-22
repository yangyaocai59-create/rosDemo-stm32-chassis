/* Artificial and recorded inputs test guards, not mechanical effectiveness. */
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "prototype/wheel_speed_controller.c"

static void step(float target,float measured,unsigned dt)
{
  const int old=s_states[0].command_permille;
  int16_t pwm;
  assert(WheelSpeedController_UpdateWithMode(MOTOR_DRIVER_WHEEL_LF,target,measured,0,
    WHEEL_SPEED_CONTROL_MODE_PIVOT,dt,&pwm));
  const WheelSpeedFrontPivotState *f=&s_front_pivot[0];
  assert(abs(pwm)<=300 && abs(pwm-old)<=(int)dt+1);
  assert(fabsf(s_states[0].integrator_permille)<=50.001f);
  assert(f->recovery_permille>=0 && f->recovery_permille<=50.001f);
  assert(f->load_permille>=0 && f->load_permille<=
    fabsf(WheelSpeedController_GetPivotLoadFeedforward(MOTOR_DRIVER_WHEEL_LF,target))+0.001f);
  assert(-s_states[0].integrator_permille+f->recovery_permille<=50.001f);
  assert(s_lf_first_start.elapsed_ms<=600 && s_lf_first_start.full_hold_ms<=100);
  assert(f->recovery_ms<=300 && f->no_motion_ms<=1500);
}

static void bounded_time(unsigned dt)
{
  const float magnitudes[]={20,40,80,160,240,280,1600};
  for(unsigned m=0;m<7;m++)
  {
    WheelSpeedController_Init();
    unsigned entries=0,exits=0;
    bool was=false;
    for(unsigned elapsed=dt;elapsed<=2000+dt;elapsed+=dt)
    {
      /* Vary shaped target until 600ms: cannot reset absolute deadline. */
      float magnitude=elapsed<600 ? fminf(magnitudes[m],20+(float)elapsed*.2f) : magnitudes[m];
      step(-magnitude,0,dt);
      const bool now=s_front_pivot[0].recovering;
      if(!was && now) ++entries;
      if(was && !now) ++exits;
      if(elapsed>=600) assert(!s_lf_first_start.first_recovery && !now);
      if(elapsed>=1500) assert(s_front_pivot[0].recovery_failed);
      was=now;
    }
    assert(entries==1 && exits==1 && s_lf_first_start.consumed);
    assert(s_front_pivot[0].recovery_permille==0);
  }
}

static void movement_and_later_restart(void)
{
  WheelSpeedController_Init();
  for(unsigned elapsed=20;elapsed<=400;elapsed+=20)
    step(-fminf(240,12+(float)(elapsed-20)*1.2f),0,20);
  assert(s_lf_first_start.first_recovery && s_front_pivot[0].recovering);
  step(-240,-100,20); step(-240,-100,20);
  assert(s_lf_first_start.has_moved && s_lf_first_start.consumed);
  assert(!s_front_pivot[0].recovering);
  /* Spend residual transient through motion handoff, then a new real stall
   * must use old 300ms pulse, never this first-start extension again. */
  for(unsigned k=0;k<100;k++) step(-240,-240,20);
  for(unsigned k=0;k<3;k++) step(-240,0,20);
  assert(s_front_pivot[0].recovering && !s_lf_first_start.first_recovery);
  for(unsigned k=0;k<15;k++) step(-240,0,20);
  assert(!s_front_pivot[0].recovering && s_front_pivot[0].recovery_ms==300);
  WheelSpeedController_Init();
  for(unsigned elapsed=20;elapsed<=400;elapsed+=20)
    step(-fminf(240,12+(float)(elapsed-20)*1.2f),0,20);
  step(-240,-400,20);
  assert(!s_front_pivot[0].recovering && s_lf_first_start.consumed);
}

static void invalid_and_clear(void)
{
  WheelSpeedController_Init();
  for(unsigned k=0;k<5;k++) step(-240,0,20);
  WheelSpeedFirstStartState before=s_lf_first_start;
  int16_t pwm=1234;
  assert(!WheelSpeedController_UpdateWithMode(MOTOR_DRIVER_WHEEL_LF,NAN,0,0,
    WHEEL_SPEED_CONTROL_MODE_PIVOT,20,&pwm) && pwm==1234);
  assert(!WheelSpeedController_UpdateWithMode(MOTOR_DRIVER_WHEEL_LF,-240,0,0,
    WHEEL_SPEED_CONTROL_MODE_PIVOT,0,&pwm));
  assert(!WheelSpeedController_UpdateWithMode(MOTOR_DRIVER_WHEEL_LF,-240,0,0,
    WHEEL_SPEED_CONTROL_MODE_PIVOT,101,&pwm));
  assert(memcmp(&before,&s_lf_first_start,sizeof(before))==0);
  assert(WheelSpeedController_Reset(MOTOR_DRIVER_WHEEL_RF));
  assert(memcmp(&before,&s_lf_first_start,sizeof(before))==0);
  step(240,0,20); assert(!s_lf_first_start.initialized);
  step(-240,0,20); assert(s_lf_first_start.initialized);
  assert(WheelSpeedController_Update(MOTOR_DRIVER_WHEEL_LF,-240,0,20,&pwm));
  assert(!s_lf_first_start.initialized);
  step(-240,0,20); step(-19,0,20); assert(!s_lf_first_start.initialized);
  step(-240,0,20); step(0,0,20); assert(!s_lf_first_start.initialized);
  step(-240,0,20); assert(WheelSpeedController_Reset(MOTOR_DRIVER_WHEEL_LF));
  assert(!s_lf_first_start.initialized);
}

static void late_entry_keeps_old_pulse(void)
{
  WheelSpeedController_Init();
  for(unsigned k=0;k<16;k++) step(-80,-21,20);
  for(unsigned k=0;k<3;k++) step(-80,0,20);
  assert(s_front_pivot[0].recovering && !s_lf_first_start.first_recovery);
  for(unsigned k=0;k<15;k++)
  {
    step(-80,0,20);
    assert(!s_lf_first_start.first_recovery);
  }
  assert(!s_front_pivot[0].recovering && s_front_pivot[0].recovery_ms==300);
}

int main(void)
{
  const unsigned dt[]={1,10,20,25,59,60,100};
  for(unsigned d=0;d<7;d++) bounded_time(dt[d]);
  movement_and_later_restart(); invalid_and_clear(); late_entry_keeps_old_pulse();
  puts("PASS: LF reverse special extension: absolute 600ms maximum, no target-change reset, no retries at rest; late entry keeps old 300ms pulse; 1500ms failure retained, dt1..100ms");
  puts("PASS: confirmed movement or FIRST overspeed ends extension; later real stall uses original 300ms pulse, unchanged effort/slew/shared-I caps");
  puts("PASS: invalid dt/NaN preserve first-start state; sign/mode/low-target/zero/reset clear it; resetting RF cannot change LF startup state");
  printf("INFO: prototype private first-start state size=%u bytes; no public state layout change\n",(unsigned)sizeof(s_lf_first_start));
  return 0;
}
