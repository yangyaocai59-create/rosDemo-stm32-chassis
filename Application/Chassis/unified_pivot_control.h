/* Original implementation: one continuous PI law, four independent instances.
 * No HAL, clock reads, encoder writes, startup pulses or running recovery. */
#ifndef UNIFIED_PIVOT_CONTROL_H_
#define UNIFIED_PIVOT_CONTROL_H_
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include "wheel_speed_control_config.h"

typedef struct {
  int8_t sign;
  bool active;
} UnifiedPivotState;

static inline float UnifiedPivot_Clamp(float value,float low,float high)
{ return fminf(high,fmaxf(low,value)); }

/* Direction-normalized PI. Every valid update computes error integration;
 * conditional anti-windup only blocks integration further into PWM/slew
 * saturation. Negative I is allowed to remove excessive feedforward.
 * Changing a same-direction target never resets I. Safety owns shutdown. */
static inline bool UnifiedPivot_Update(UnifiedPivotState *memory,
  float target,float measured,float base_ff,float load_ff,float kp,float ki,
  uint32_t dt,float previous_output,float *signed_i,float *signed_ff,
  float *signed_p,float *output,bool *limited)
{
  if(memory==0 || signed_i==0 || signed_ff==0 || signed_p==0 || output==0 ||
     limited==0 || !isfinite(target) || !isfinite(measured) ||
     !isfinite(base_ff) || !isfinite(load_ff) || !isfinite(kp) || !isfinite(ki) ||
     !isfinite(previous_output) || !isfinite(*signed_i) || dt==0 || dt>100 ||
     fabsf(target)<WHEEL_SPEED_CONTROL_ZERO_TARGET_CPS ||
     base_ff<0 || load_ff<0 || kp<0 || ki<0)
    return false;
  UnifiedPivotState next=*memory;
  const int8_t sign=target>0?1:-1;
  float j=*signed_i*(float)sign;
  if(!next.active || next.sign!=sign)
  {
    next=(UnifiedPivotState){.active=true,.sign=sign};
    j=0;
  }
  const float magnitude=fabsf(target), speed=measured*(float)sign;
  const float error=magnitude-speed, seconds=(float)dt/1000;
  const float limit=WHEEL_SPEED_CONTROL_INTEGRATOR_LIMIT_PERMILLE;
  const float ceiling=(float)WHEEL_SPEED_CONTROL_OUTPUT_LIMIT_PERMILLE;
  const float step=WHEEL_SPEED_CONTROL_SLEW_PERMILLE_PER_SECOND*seconds;
  const float old_j=UnifiedPivot_Clamp(j,-limit,limit);
  j=UnifiedPivot_Clamp(old_j+ki*error*seconds,-limit,limit);
  const float p=kp*error, ff=base_ff+load_ff;
  float wanted=ff+j+p;
  float desired=UnifiedPivot_Clamp(wanted,0,ceiling)*(float)sign;
  float actual=UnifiedPivot_Clamp(desired,previous_output-step,previous_output+step);
  const float achieved=actual*(float)sign;
  if((j-old_j)*(wanted-achieved)>0)
  {
    j=old_j; /* Output cannot yet realize this integral increment. */
    wanted=ff+j+p;
    desired=UnifiedPivot_Clamp(wanted,0,ceiling)*(float)sign;
    actual=UnifiedPivot_Clamp(desired,previous_output-step,previous_output+step);
  }
  *signed_i=j*(float)sign;
  *signed_ff=ff*(float)sign;
  *signed_p=p*(float)sign;
  *output=actual;
  *limited=wanted<0 || wanted>ceiling || actual!=desired;
  *memory=next;
  return true;
}
#endif
