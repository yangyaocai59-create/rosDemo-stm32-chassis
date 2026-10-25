#include <assert.h>
#include <math.h>
#include <stdio.h>
#include "chassis_motion_profile.h"
#include "chassis_config.h"
int main(void)
{
  const float half_track=CHASSIS_EFFECTIVE_TRACK_M*.5f;
  const float acceleration=fmaxf(CHASSIS_PROFILE_LINEAR_ACCEL_M_S2,
    half_track*CHASSIS_PROFILE_ANGULAR_ACCEL_RAD_S2);
  const float deceleration=fmaxf(CHASSIS_PROFILE_LINEAR_DECEL_M_S2,
    half_track*CHASSIS_PROFILE_ANGULAR_DECEL_RAD_S2);
  for(unsigned k=0;k<60000;k++) {
    ChassisVelocityCommand before={((int)(k%401)-200)*.001f,((int)((k*17)%3001)-1500)*.001f};
    ChassisVelocityCommand target={((int)((k*3)%401)-200)*.001f,((int)((k*31)%3001)-1500)*.001f};
    ChassisVelocityCommand next;
    unsigned dt=1+k%100;
    assert(ChassisMotionProfile_Step(&before,&target,dt,&next));
    assert(fabsf(next.linear_x_m_s-before.linear_x_m_s)<=.3f*dt/1000+.00001f);
    assert(fabsf(next.angular_z_rad_s-before.angular_z_rad_s)<=2.0f*dt/1000+.00001f);
    for(int side=-1;side<=1;side+=2) {
      float old_wheel=before.linear_x_m_s+side*half_track*before.angular_z_rad_s;
      float new_wheel=next.linear_x_m_s+side*half_track*next.angular_z_rad_s;
      float limit=fabsf(new_wheel)<fabsf(old_wheel)?deceleration:acceleration;
      assert(fabsf(new_wheel-old_wheel)<=limit*dt/1000+.00001f);
    }
  }
  ChassisVelocityCommand zero={0},target={.2f,0},next;
  assert(ChassisMotionProfile_Step(&zero,&target,20,&next));assert(fabsf(next.linear_x_m_s-.004f)<.000001f);
  target=(ChassisVelocityCommand){0,.4f};
  assert(ChassisMotionProfile_Step(&zero,&target,20,&next));assert(fabsf(next.angular_z_rad_s-.03f)<.000001f);
  target=(ChassisVelocityCommand){.2f,.4f};
  assert(ChassisMotionProfile_Step(&zero,&target,20,&next));
  assert(next.linear_x_m_s<.004f&&next.angular_z_rad_s<.03f);
  puts("PASS: 60000 mixed/reversing commands constrain both wheel accelerations; pure straight and pivot ramps retain existing engineering limits");
  return 0;
}
