#include <assert.h>
#include <stdio.h>
#include "wheel_speed_controller.h"
int main(void) {
  WheelSpeedController_Init();
  for(unsigned k=0;k<80000;k++) {
    unsigned w=k%4;
    if(k%397==0) assert(WheelSpeedController_Reset((MotorDriverWheel)w));
    if(k%499==0) assert(WheelSpeedController_PrimeOutput((MotorDriverWheel)w,(int)(k%601)-300));
    float target=(float)((int)((k*37)%3301)-1650);
    float speed=(float)((int)((k*89)%4001)-2000);
    int16_t output;
    assert(WheelSpeedController_UpdateWithMode((MotorDriverWheel)w,target,speed,0,
      (k/16)%2?WHEEL_SPEED_CONTROL_MODE_NORMAL:WHEEL_SPEED_CONTROL_MODE_PIVOT,
      1+k%100,&output));
    WheelSpeedControllerState s;
    assert(WheelSpeedController_GetState((MotorDriverWheel)w,&s));
    printf("%u %d %a %a %a %u\n",w,output,(double)s.feedforward_permille,
      (double)s.proportional_permille,(double)s.integrator_permille,s.limited?1U:0U);
  }
}
