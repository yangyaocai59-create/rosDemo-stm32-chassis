/* Link once with frozen .14, once with .15; no motor/serial/physical model. */
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include "wheel_speed_controller.h"

static WheelSpeedControllerState step(MotorDriverWheel w,float t,float m)
{
  int16_t pwm;
  WheelSpeedControllerState s;
  assert(WheelSpeedController_UpdateWithMode(w,t,m,0,
    WHEEL_SPEED_CONTROL_MODE_PIVOT,20,&pwm));
  assert(WheelSpeedController_GetState(w,&s));
  return s;
}
static float support(WheelSpeedControllerState s,float sign)
{ return sign*(s.feedforward_permille+s.integrator_permille); }

int main(void)
{
  const MotorDriverWheel wheels[]={MOTOR_DRIVER_WHEEL_LF,MOTOR_DRIVER_WHEEL_RF};
  for(unsigned w=0;w<2;w++) for(int sign=-1;sign<=1;sign+=2)
  {
    const float t=sign*160.0f;
    WheelSpeedController_Init();
    for(unsigned k=0;k<20;k++) (void)step(wheels[w],t,t);
    assert(fabsf(WheelSpeedController_ReleaseFeedforwardBias(wheels[w],sign*20))<0.001f);
    const float before=support(step(wheels[w],t,t),(float)sign);
    WheelSpeedControllerState s;
    for(unsigned k=0;k<8;k++) s=step(wheels[w],t,0);
    const float peak=support(s,(float)sign);
    for(unsigned k=0;k<8;k++) s=step(wheels[w],t,sign*100.0f);
    const float after=support(s,(float)sign);
    for(unsigned k=0;k<100;k++) s=step(wheels[w],t,sign*100.0f);
    WheelSpeedFrontPivotState f;
    assert(WheelSpeedController_GetFrontPivotState(wheels[w],&f));
    printf("wheel=%u sign=%d before=%.5f rest_160ms=%.5f moving_160ms=%.5f moving_2160ms=%.5f I=%.5f transient=%.5f\n",
      (unsigned)wheels[w],sign,before,peak,after,support(s,(float)sign),
      s.integrator_permille*sign,f.recovery_permille);
  }
  return 0;
}
