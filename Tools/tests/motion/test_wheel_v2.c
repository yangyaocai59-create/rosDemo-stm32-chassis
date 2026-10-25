/* Production revision 200 kernel/API, randomized properties and handoffs. */
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include "wheel_speed_controller.h"

static WheelSpeedControllerState State(unsigned w)
{ WheelSpeedControllerState s; assert(WheelSpeedController_GetState((MotorDriverWheel)w,&s)); return s; }
static void Update(unsigned w,float target,float measured,WheelSpeedControlMode mode,unsigned dt)
{ int16_t out; assert(WheelSpeedController_UpdateWithMode((MotorDriverWheel)w,target,measured,0,mode,dt,&out)); }

int main(void)
{
  WheelSpeedController_Init();
  for (unsigned w=0;w<4;w++) for (int sign=-1;sign<=1;sign+=2) {
    WheelSpeedController_Reset((MotorDriverWheel)w);
    for (unsigned k=0;k<160;k++) Update(w,sign*240,sign*200,WHEEL_SPEED_CONTROL_MODE_NORMAL,20);
    const WheelSpeedControllerState before=State(w);
    Update(w,sign*240,sign*200,WHEEL_SPEED_CONTROL_MODE_PIVOT,20);
    const WheelSpeedControllerState after=State(w);
    /* For the existing calibration all handoffs here fit within I50. */
    assert(abs(after.command_permille-before.command_permille)<=1);
    assert(after.limit_reasons&WHEEL_LIMIT_HANDOFF);
    assert(WheelSpeedController_IsPivotMode((MotorDriverWheel)w));
    Update(w,sign*240,sign*200,WHEEL_SPEED_CONTROL_MODE_NORMAL,20);
    assert(abs(State(w).command_permille-after.command_permille)<=1);
    assert(!WheelSpeedController_IsPivotMode((MotorDriverWheel)w));
  }
  puts("PASS: four wheels/both signs track applied output during NORMAL/PIVOT handoff");
  WheelSpeedController_Init();
  for (unsigned k=0;k<100000;k++) {
    const unsigned w=k%4;
    const unsigned dt=1+k%100;
    const float target=(float)((int)((k*37U)%4001U)-2000);
    const float measured=(float)((int)((k*89U)%8001U)-4000);
    const int previous=State(w).command_permille;
    Update(w,target,measured,(k/16)%2?WHEEL_SPEED_CONTROL_MODE_NORMAL:WHEEL_SPEED_CONTROL_MODE_PIVOT,dt);
    const WheelSpeedControllerState s=State(w);
    assert(abs(s.command_permille)<=300);
    assert(abs(s.command_permille-previous)<=(int)dt+1);
    assert(fabsf(s.integrator_permille)<=50.001f);
    assert(fabsf(s.target_counts_per_second)<=1600.001f);
    assert(isfinite(s.feedforward_permille)&&isfinite(s.proportional_permille));
  }
  puts("PASS: 100000 mixed commands enforce target/PWM/I/slew bounds through reversals and mode changes");
  WheelSpeedTransaction t;
  assert(WheelSpeedController_Begin(&t));
  int16_t out=123;
  assert(!WheelSpeedController_Step(&t,MOTOR_DRIVER_WHEEL_LF,NAN,0,0,WHEEL_SPEED_CONTROL_MODE_NORMAL,20,&out));
  assert(out==123);
  assert(!WheelSpeedController_UpdateWithMode(MOTOR_DRIVER_WHEEL_LF,200,INFINITY,0,WHEEL_SPEED_CONTROL_MODE_NORMAL,20,&out));
  assert(!WheelSpeedController_UpdateWithMode(MOTOR_DRIVER_WHEEL_LF,200,0,1,WHEEL_SPEED_CONTROL_MODE_PIVOT,20,&out));
  assert(!WheelSpeedController_UpdateWithMode(MOTOR_DRIVER_WHEEL_LF,200,0,0,WHEEL_SPEED_CONTROL_MODE_NORMAL,101,&out));
  assert(!WheelSpeedController_Update(MOTOR_DRIVER_WHEEL_LF,200,0,20,NULL));
  assert(WheelSpeedController_Step(&t,MOTOR_DRIVER_WHEEL_LF,200,0,0,WHEEL_SPEED_CONTROL_MODE_NORMAL,20,&out));
  WheelSpeedController_Reset(MOTOR_DRIVER_WHEEL_LF);
  assert(!WheelSpeedController_Commit(&t));
  assert(State(0).command_permille==0);
  assert(WheelSpeedController_Begin(&t));
  WheelSpeedController_PrimeOutput(MOTOR_DRIVER_WHEEL_RF,0);
  assert(!WheelSpeedController_Commit(&t));
  puts("PASS: invalid inputs leave outputs unchanged; reset/applied-output feedback invalidates pending transactions");
  WheelSpeedController_Init();
  for (unsigned k=0;k<100;k++) Update(0,240,200,WHEEL_SPEED_CONTROL_MODE_PIVOT,20);
  WheelSpeedControllerState before=State(0);
  assert(before.integrator_permille>0);
  assert(WheelSpeedController_PrimeOutput(MOTOR_DRIVER_WHEEL_LF,0));
  assert(State(0).integrator_permille==before.integrator_permille);
  assert(WheelSpeedController_IsPivotMode(MOTOR_DRIVER_WHEEL_LF));
  Update(0,240,200,WHEEL_SPEED_CONTROL_MODE_PIVOT,20);
  assert(State(0).command_permille<=20);
  Update(0,0,0,WHEEL_SPEED_CONTROL_MODE_PIVOT,20);
  assert(State(0).integrator_permille==0);
  assert(!WheelSpeedController_IsPivotMode(MOTOR_DRIVER_WHEEL_LF));
  puts("PASS: actuator interlock preserves direction/I, follows applied zero and respects next-step slew");
  return 0;
}
