#include "rr_running_fixture.h"

int main(void)
{
  WheelSpeedController_Init();
  int16_t pwm;
  assert(!WheelSpeedController_SelectRrRunningSupport(true));
  assert(WheelSpeedController_UpdateWithMode(MOTOR_DRIVER_WHEEL_RR,
    240,240,0,WHEEL_SPEED_CONTROL_MODE_PIVOT,20,&pwm));
  assert(!WheelSpeedController_SelectRrRunningSupport(true));
  assert(WheelSpeedController_UpdateWithMode(MOTOR_DRIVER_WHEEL_RR,
    240,240,0,WHEEL_SPEED_CONTROL_MODE_PIVOT,20,&pwm));
  assert(!WheelSpeedController_SelectRrRunningSupport(false));
  assert(WheelSpeedController_SelectRrRunningSupport(true));
  assert(WheelSpeedController_SelectRrRunningSupport(false));
  puts("PASS: RR requires 40ms confirmed prior motion and zero-legacy-bias caller permission; ownership remains across bounded pulse exit");

  const ChassisVelocityCommand c={0,.3f};
  for(unsigned which=0;which<3;which++)
  {
    WheelSpeedController_Init();
    for(unsigned w=0;w<4;w++) ChassisController_ClearPivotWheel((MotorDriverWheel)w);
    s_rr_running.moving_ms=40;
    s_pivot_breakaway_sign[3]=1;
    s_pivot_has_moved[3]=true;
    if(which==0) s_pivot_breakaway_active[3]=true;
    if(which==1) s_pivot_applied_bias[3]=1;
    if(which==2) s_pivot_steady_bias[3]=1;
    (void)ChassisController_GetFeedforwardBias(&c,MOTOR_DRIVER_WHEEL_RR,240,240,100,20);
    assert(!s_rr_running.enabled);
    ChassisController_ClearPivotWheel(MOTOR_DRIVER_WHEEL_RR);
    /* Clear deliberately clears motion qualification too. */
    assert(!s_rr_running.moving_ms);
    s_rr_running.moving_ms=40;
    s_pivot_breakaway_sign[3]=1;
    s_pivot_has_moved[3]=true;
    assert(ChassisController_GetFeedforwardBias(&c,MOTOR_DRIVER_WHEEL_RR,240,240,120,20)==0);
    assert(s_rr_running.enabled);
  }
  puts("PASS: actual chassis cannot acquire RR while old pulse/applied/steady bias remains; zero-bias handoff and chassis-clear lifetime verified");
  return 0;
}
