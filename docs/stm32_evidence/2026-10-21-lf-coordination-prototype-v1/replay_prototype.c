/* Offline diagnostic TU: isolated prototype .c included UNMODIFIED so only
 * inferred initial private state can be seeded. No GPIO/RTOS/serial code. */
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include "prototype/wheel_speed_controller.c"
#include "prototype/chassis_config.h"

static void emit(unsigned t,MotorDriverWheel w,float measured)
{
  const WheelSpeedControllerState *s=&s_states[w];
  const WheelSpeedFrontPivotState *f=&s_front_pivot[w];
  printf("%u,%.9f,%.9f,%.9f,%.9f,%d,%.9f,%.9f,%u,%u,%u,%u,%u,%d,%d,%d\n",
    t,measured,s->feedforward_permille,s->proportional_permille,
    s->integrator_permille,s->command_permille,f->load_permille,
    f->recovery_permille,f->still_ms,f->moving_ms,f->load_overspeed_ms,
    f->recovery_ms,f->no_motion_ms,f->recovering,f->exhausted,f->recovery_failed);
}
int main(void)
{
  unsigned index,t;
  float load,integral,measured;
  assert(scanf("%u %u %f %f %f",&index,&t,&load,&integral,&measured)==5);
  assert(index==MOTOR_DRIVER_WHEEL_LF || index==MOTOR_DRIVER_WHEEL_RF);
  const MotorDriverWheel w=(MotorDriverWheel)index;
  const int sign=w==MOTOR_DRIVER_WHEEL_LF ? 1 : -1;
  const float counts_per_metre=(float)CHASSIS_ENCODER_COUNTS_PER_WHEEL_REV /
    CHASSIS_WHEEL_CIRCUMFERENCE_M;
  const float target=(float)sign*(0.2f*(CHASSIS_WHEEL_TRACK_M*0.5f))*counts_per_metre;
  WheelSpeedController_Init();
  s_front_pivot[w]=(WheelSpeedFrontPivotState){.active=true,.sign=(int8_t)sign,
    .moving_ms=40,.load_permille=load};
  const float ff=WheelSpeedController_GetFeedforward(w,target)+sign*load;
  const float error=target-measured;
  const float kp=measured*sign>fabsf(target) ?
    WHEEL_SPEED_CONTROL_PIVOT_OVERSPEED_KP : WHEEL_SPEED_CONTROL_KP_PERMILLE_PER_CPS;
  const float p=kp*error;
  const float output=ff+p+integral;
  s_states[w]=(WheelSpeedControllerState){.target_counts_per_second=target,
    .measured_counts_per_second=measured,.error_counts_per_second=error,
    .feedforward_permille=ff,.proportional_permille=p,.integrator_permille=integral,
    .command_permille=WheelSpeedController_RoundToInt16(output)};
  s_previous_target[w]=target;
  s_previous_output[w]=output;
  emit(t,w,measured);
  while(scanf("%u %f",&t,&measured)==2)
  {
    int16_t command;
    assert(WheelSpeedController_UpdateWithMode(w,target,measured,0,
      WHEEL_SPEED_CONTROL_MODE_PIVOT,20,&command));
    assert(abs(command)<=300 && fabsf(s_states[w].integrator_permille)<=50.001f);
    assert(sign*s_states[w].integrator_permille+
      s_front_pivot[w].recovery_permille<=50.001f);
    emit(t,w,measured);
  }
  return 0;
}
