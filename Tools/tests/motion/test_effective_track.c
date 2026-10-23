/* Hand-computed .4rad/s operating point plus forward/inverse invariants.
 * Pure kinematics: not a motor or tyre slip simulation. */
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include "chassis_kinematics.h"
#include "chassis_config.h"
static void Near(float a,float b) { assert(fabsf(a-b)<.0002f); }
int main(void) {
  ChassisVelocityCommand c={0,.4f}; ChassisWheelTargets t; ChassisVelocityEstimate e;
  assert(CHASSIS_WHEEL_TRACK_MM==210U);
  Near(CHASSIS_EFFECTIVE_TRACK_M,.395f);
  assert(ChassisKinematics_ToWheelTargets(&c,1600,&t));
  /* .4 * .395 / 2 = .079m/s. Wheel angular speed=.079/.0325. */
  Near(t.left_linear_m_s,-.079f); Near(t.right_linear_m_s,.079f);
  const float expected=(float)(.079*1560.0/(3.141592653589793*.065));
  Near(t.counts_per_second[0],-expected); Near(t.counts_per_second[1],-expected);
  Near(t.counts_per_second[2],expected); Near(t.counts_per_second[3],expected);
  Near(t.scale,1); assert(ChassisKinematics_FromWheelSpeeds(t.counts_per_second,&e));
  Near(e.angular_z_rad_s,.4f); Near(e.linear_x_m_s,0);
  printf("PASS: .40rad/s => +/-%.3fcps, both wheels per side share targets; physical210/effective395 kept distinct\n",expected);
  c.angular_z_rad_s=-.4f; assert(ChassisKinematics_ToWheelTargets(&c,1600,&t));
  Near(t.counts_per_second[0],expected);Near(t.counts_per_second[3],-expected);
  c.angular_z_rad_s=.1f; assert(ChassisKinematics_ToWheelTargets(&c,1600,&t));
  Near(t.counts_per_second[3],expected*.25f); /* no minimum-speed inflation */
  puts("PASS: right-turn symmetry and small .10rad/s command retained without hidden floor");
  for(int v=-20;v<=20;v++) for(int w=-15;w<=15;w++) {
    c=(ChassisVelocityCommand){v*.01f,w*.1f};
    assert(ChassisKinematics_ToWheelTargets(&c,1600,&t));
    assert(t.counts_per_second[0]==t.counts_per_second[1]);
    assert(t.counts_per_second[2]==t.counts_per_second[3]);
    for(unsigned i=0;i<4;i++) assert(fabsf(t.counts_per_second[i])<1600.001f);
    assert(ChassisKinematics_FromWheelSpeeds(t.counts_per_second,&e));
    Near(e.linear_x_m_s,c.linear_x_m_s*t.scale);
    Near(e.angular_z_rad_s,c.angular_z_rad_s*t.scale);
  }
  puts("PASS: 1271 mixed forward/reverse/turn commands round-trip consistently with curvature-preserving saturation");
  c=(ChassisVelocityCommand){.04f,0};
  assert(ChassisKinematics_ToWheelTargets(&c,1600,&t));
  for(unsigned i=0;i<4;i++) Near(t.counts_per_second[i],(float)(.04*1560/(3.141592653589793*.065)));
  assert(!ChassisKinematics_ToWheelTargets(NULL,1600,&t));
  c.angular_z_rad_s=NAN;assert(!ChassisKinematics_ToWheelTargets(&c,1600,&t));
  c.angular_z_rad_s=0;assert(!ChassisKinematics_ToWheelTargets(&c,0,&t));
  puts("PASS: straight targets unchanged; NULL/NaN/invalid limit rejected");
}
