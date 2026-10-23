/* Same source compiled against frozen .21 and candidate for NORMAL identity. */
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include "wheel_speed_controller.h"
int main(void)
{
  WheelSpeedController_Init();
  for(unsigned k=0;k<40000;k++)
  {
    unsigned w=k%4;
    if(k%397==0) assert(WheelSpeedController_Reset((MotorDriverWheel)w));
    float target=(float)((int)((k*37)%3301)-1650);
    float measured=(float)((int)((k*89)%4001)-2000);
    int16_t out;
    assert(WheelSpeedController_UpdateWithMode((MotorDriverWheel)w,target,measured,0,
      WHEEL_SPEED_CONTROL_MODE_NORMAL,1+k%100,&out));
    WheelSpeedControllerState s;
    assert(WheelSpeedController_GetState((MotorDriverWheel)w,&s));
    printf("%u %d %.9g %.9g %.9g %u\n",w,out,s.feedforward_permille,
      s.proportional_permille,s.integrator_permille,s.limited?1U:0U);
  }
  return 0;
}
