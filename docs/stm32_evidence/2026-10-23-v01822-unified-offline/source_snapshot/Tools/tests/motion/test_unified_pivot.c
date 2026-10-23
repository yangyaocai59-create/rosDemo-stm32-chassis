/* Actual candidate PI, prescribed feedback only; NOT a physical motor model. */
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "../../../Application/Chassis/wheel_speed_controller.c"

static WheelSpeedControllerState Step(unsigned w,float target,float measured,unsigned dt)
{
  int16_t command;
  assert(WheelSpeedController_UpdateWithMode((MotorDriverWheel)w,target,measured,0,
    WHEEL_SPEED_CONTROL_MODE_PIVOT,dt,&command));
  WheelSpeedControllerState state;
  assert(WheelSpeedController_GetState((MotorDriverWheel)w,&state));
  assert(abs(command)<=300 && fabsf(state.integrator_permille)<=50.001f);
  assert(s_unified_pivot[w].active && s_unified_pivot[w].sign==(target>0?1:-1));
  return state;
}

int main(void)
{
  WheelSpeedController_Init();
  for(unsigned w=0;w<4;w++) for(int sign=-1;sign<=1;sign+=2)
    for(unsigned m=0;m<6;m++)
    {
      const float mag=(float[]){20,80,160,240,280,1600}[m];
      assert(WheelSpeedController_Reset((MotorDriverWheel)w));
      int previous=0;
      for(unsigned t=0;t<3000;t+=20)
      {
        float speed=t<400?0:t<1200?mag*.5f:t<2000?mag:mag*1.6f;
        WheelSpeedControllerState s=Step(w,sign*mag,sign*speed,20);
        assert(abs(s.command_permille-previous)<=21);
        previous=s.command_permille;
        float ff=(WheelSpeedController_GetFeedforward((MotorDriverWheel)w,sign*mag)+
          WheelSpeedController_GetPivotLoadFeedforward((MotorDriverWheel)w,sign*mag));
        assert(fabsf(s.feedforward_permille-ff)<.001f); /* no hidden kick */
      }
      assert(WheelSpeedController_Reset((MotorDriverWheel)w));
      assert(!s_unified_pivot[w].active);
    }
  puts("PASS: all four wheels/both signs/6 targets; fixed FF, PWM300/I50/slew1000 limits and reset");

  for(unsigned w=0;w<4;w++) for(int sign=-1;sign<=1;sign+=2)
  {
    WheelSpeedController_Reset((MotorDriverWheel)w);
    for(unsigned t=0;t<400;t+=20) Step(w,sign*240,sign*240,20);
    /* Low but moving feedback: integration must continue, not be replaced
     * by a wheel-specific recovery state or a repeated startup pulse. */
    for(unsigned t=0;t<500;t+=20)
    {
      float before=s_states[w].integrator_permille*sign;
      WheelSpeedControllerState s=Step(w,sign*240,sign*160,20);
      assert(fabsf(s.integrator_permille*sign-before-.4f)<.0001f);
    }
    const float held=s_states[w].integrator_permille;
    for(unsigned t=0;t<6000;t+=20)
      assert(fabsf(Step(w,sign*240,sign*240,20).integrator_permille-held)<.0001f);
    float before=s_states[w].integrator_permille*sign;
    WheelSpeedControllerState s=Step(w,sign*241,sign*240,20);
    assert(fabsf(s.integrator_permille*sign-before-.005f)<.0001f);
    before=s.integrator_permille*sign;
    s=Step(w,sign*241,sign*321,20);
    assert(fabsf(s.integrator_permille*sign-before+.4f)<.0001f);
  }
  puts("PASS: PI integrates every unsaturated tick in both signs, retains I at equilibrium and across same-direction target updates; overspeed unwinds");

  for(unsigned w=0;w<4;w++) for(int sign=-1;sign<=1;sign+=2)
  {
    WheelSpeedController_Reset((MotorDriverWheel)w);
    Step(w,sign*240,0,20);
    assert(s_states[w].integrator_permille==0); /* no windup behind startup slew */
    for(unsigned k=0;k<50;k++) Step(w,sign*1600,-sign*3000,20);
    assert(s_states[w].integrator_permille==0); /* no windup behind PWM cap */
    WheelSpeedController_Reset((MotorDriverWheel)w);
    for(unsigned k=0;k<20;k++) Step(w,sign*240,sign*240,20);
    s_states[w].integrator_permille=sign*10;
    for(unsigned k=0;k<50;k++) Step(w,sign*240,sign*320,20);
    assert(s_states[w].integrator_permille*sign<0); /* can cancel FF */
    Step(w,-sign*240,-sign*240,20);
    assert(s_states[w].integrator_permille==0); /* sign change clears old I */
  }
  puts("PASS: startup slew/PWM saturation anti-windup, negative I cancels excess FF, reversal clears old direction state");

  for(unsigned w=0;w<4;w++)
  {
    WheelSpeedController_Reset((MotorDriverWheel)w);
    for(unsigned t=20;t<=1480;t+=20) Step(w,240,0,20);
    assert(!s_unified_pivot[w].failed);
    Step(w,240,240,20); assert(s_unified_pivot[w].no_motion_ms==0);
    for(unsigned t=20;t<=1500;t+=20) Step(w,240,0,20);
    assert(s_unified_pivot[w].failed && s_unified_pivot[w].no_motion_ms==1500);
    Step(w,240,240,20); assert(s_unified_pivot[w].failed);
    WheelSpeedController_Reset((MotorDriverWheel)w);
    for(unsigned k=0;k<100;k++) Step(w,10,0,20);
    assert(!s_unified_pivot[w].failed); /* old >=20cps policy boundary */
  }
  puts("PASS: near-rest deadline resets on recovery, reports sticky failure at1500ms, target below20 excluded");

  WheelSpeedController_Init();
  for(unsigned k=0;k<30;k++) Step(0,240,100,20);
  for(unsigned w=1;w<4;w++) assert(!s_unified_pivot[w].active && s_states[w].integrator_permille==0);
  const UnifiedPivotState old=s_unified_pivot[0];
  const WheelSpeedControllerState old_state=s_states[0];
  int16_t out=123;
  assert(!WheelSpeedController_UpdateWithMode(MOTOR_DRIVER_WHEEL_LF,240,100,1,
    WHEEL_SPEED_CONTROL_MODE_PIVOT,20,&out));
  assert(!WheelSpeedController_UpdateWithMode(MOTOR_DRIVER_WHEEL_LF,NAN,100,0,
    WHEEL_SPEED_CONTROL_MODE_PIVOT,20,&out));
  assert(!WheelSpeedController_UpdateWithMode(MOTOR_DRIVER_WHEEL_LF,240,100,0,
    WHEEL_SPEED_CONTROL_MODE_PIVOT,0,&out));
  assert(!WheelSpeedController_UpdateWithMode(MOTOR_DRIVER_WHEEL_LF,240,INFINITY,0,
    WHEEL_SPEED_CONTROL_MODE_PIVOT,20,&out));
  assert(memcmp(&old,&s_unified_pivot[0],sizeof old)==0 && out==123);
  assert(memcmp(&old_state,&s_states[0],sizeof old_state)==0);
  assert(WheelSpeedController_UpdateWithMode(MOTOR_DRIVER_WHEEL_LF,0,0,0,
    WHEEL_SPEED_CONTROL_MODE_PIVOT,20,&out));
  assert(!s_unified_pivot[0].active && s_states[0].integrator_permille==0);
  Step(0,240,100,20);
  assert(WheelSpeedController_Update(MOTOR_DRIVER_WHEEL_LF,240,240,20,&out));
  assert(!s_unified_pivot[0].active);
  puts("PASS: independent state; invalid/conflicting owner calls transactional; zero and NORMAL exit clear pivot state");
  return 0;
}
