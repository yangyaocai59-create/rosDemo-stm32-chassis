/* Offline-only prototype replay from reset; recorded
 * encoder increments are inputs, NOT a simulated motor or mechanical load. */
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include "prototype/wheel_speed_controller.c"
#include "prototype/chassis_motion_profile.c"
#include "prototype/chassis_kinematics.c"

int main(void)
{
  unsigned t, count=0, last_motion=0;
  int dc;
  float measured=0;
  ChassisVelocityCommand applied={0,0}, next;
  const ChassisVelocityCommand requested={0,0.3f};
  WheelSpeedController_Init();
  puts("t,applied_mrad,target,measured,ff,p,i,pwm,load,recovery,ceiling,headroom,still_ms,moving_ms,recovery_ms,no_motion_ms,recovering,exhausted,failed,first_elapsed,full_hold,first_recovery,consumed,has_moved");
  while(scanf("%u %d", &t, &dc)==2)
  {
    const float raw=(float)dc*50.0f;
    measured += (20.0f/70.0f)*(raw-measured);
    if(dc!=0) { last_motion=t; }
    else if(t-last_motion>=200) { measured=0; }
    assert(ChassisMotionProfile_Step(&applied,&requested,count ? 20U : 10U,&next));
    applied=next;
    ChassisWheelTargets targets;
    assert(ChassisKinematics_ToWheelTargets(&applied,1600,&targets));
    const float target=targets.counts_per_second[MOTOR_DRIVER_WHEEL_LF];
    int16_t pwm;
    assert(WheelSpeedController_UpdateWithMode(MOTOR_DRIVER_WHEEL_LF,target,measured,0,
      WHEEL_SPEED_CONTROL_MODE_PIVOT,20,&pwm));
    const WheelSpeedControllerState *s=&s_states[MOTOR_DRIVER_WHEEL_LF];
    const WheelSpeedFrontPivotState *f=&s_front_pivot[MOTOR_DRIVER_WHEEL_LF];
    const float ceiling=fabsf(WheelSpeedController_GetPivotLoadFeedforward(MOTOR_DRIVER_WHEEL_LF,target));
    const float authority=50*fminf(fabsf(target)/160,1);
    const float room=fmaxf(0,fminf(50+s->integrator_permille,authority));
    assert(abs(pwm)<=300 && fabsf(s->integrator_permille)<=50.001f);
    printf("%u,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,%d,%.9f,%.9f,%.9f,%.9f,%u,%u,%u,%u,%d,%d,%d,%u,%u,%d,%d,%d\n",
      t,applied.angular_z_rad_s*1000,target,measured,s->feedforward_permille,
      s->proportional_permille,s->integrator_permille,pwm,f->load_permille,
      f->recovery_permille,ceiling,room,f->still_ms,f->moving_ms,
      f->recovery_ms,f->no_motion_ms,f->recovering,f->exhausted,f->recovery_failed,
      s_lf_first_start.elapsed_ms,s_lf_first_start.full_hold_ms,
      s_lf_first_start.first_recovery,s_lf_first_start.consumed,s_lf_first_start.has_moved);
    ++count;
  }
  return 0;
}
