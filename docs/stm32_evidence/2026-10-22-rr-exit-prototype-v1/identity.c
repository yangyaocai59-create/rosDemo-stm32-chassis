/* Production RR running control versus frozen validated prototype; no fault manager. */
#include "fixture.h"
int main(void)
{
  const unsigned dt[]={1,10,20,59,60,100};
  for(unsigned trial=0;trial<300;trial++)
  {
    WheelSpeedController_Init();
    const float target=(20.0f+40.0f*(trial%15));
    int16_t pwm;
    for(unsigned k=0;k<2;k++) assert(WheelSpeedController_UpdateWithMode(
      MOTOR_DRIVER_WHEEL_RR,target,240,0,WHEEL_SPEED_CONTROL_MODE_PIVOT,20,&pwm));
    assert(WheelSpeedController_SelectRrRunningSupport(true));
    s_states[3].integrator_permille=(float)((int)(trial%101)-50);
    for(unsigned k=0;k<100;k++)
    {
      float measured=k<10 ? target : k<40 ? 0 : k<65 ? target*2 : target;
      assert(WheelSpeedController_UpdateWithMode(MOTOR_DRIVER_WHEEL_RR,target,measured,0,
        WHEEL_SPEED_CONTROL_MODE_PIVOT,dt[(trial+k)%6],&pwm));
      WheelSpeedControllerState s=s_states[3]; WheelSpeedRrRunningState r=s_rr_running;
      printf("%u,%u,%d,%a,%a,%a,%u,%u,%u,%u,%d,%d,%d\n",trial,k,pwm,
        (double)s.feedforward_permille,(double)s.integrator_permille,
        (double)r.temporary_permille,r.moving_ms,r.under_ms,r.settled_ms,
        r.episode_ms,r.enabled,r.active,r.spent);
    }
  }
  return 0;
}
