/* Byte comparison of frozen .19 and offline RR prototype. */
#include "rr_exit_fixture.h"
static void reset(void)
{
  WheelSpeedController_Init();
  for(unsigned w=0;w<4;w++) ChassisController_ClearPivotWheel((MotorDriverWheel)w);
}
static void emit(unsigned wheel, float target, float measured, bool pivot, unsigned dt,uint32_t now)
{
  const MotorDriverWheel w=(MotorDriverWheel)wheel;
  ChassisVelocityCommand c={pivot ? 0.0f : .04f,pivot ? .3f : .2f};
  int16_t pwm=1234;
  float bias=0;
#if !defined(BASELINE) && defined(INJECT_COUNTS)
  WheelSpeedController_ObserveRrRecoveryCounts((int)(now/20%15),20,now,now);
#endif
  if(dt && dt<=100) bias=ChassisController_GetFeedforwardBias(&c,w,target,measured,now,dt);
  const bool ok=WheelSpeedController_UpdateWithMode(w,target,measured,bias,
    pivot ? WHEEL_SPEED_CONTROL_MODE_PIVOT : WHEEL_SPEED_CONTROL_MODE_NORMAL,dt,&pwm);
  const WheelSpeedControllerState *s=&s_states[w];
  printf("%u,%d,%d,%a,%a,%a,%a,%a,%a,%d,%d,%d,%a,%a,%d,%d\n",wheel,ok,pwm,
    (double)s->target_counts_per_second,(double)s->measured_counts_per_second,
    (double)s->error_counts_per_second,(double)s->feedforward_permille,
    (double)s->proportional_permille,(double)s->integrator_permille,
    s->command_permille,s->limited,s_pivot_breakaway_active[w],
    (double)s_pivot_applied_bias[w],(double)s_pivot_steady_bias[w],
    s_pivot_has_moved[w],s_pivot_rear_restart[w]);
  printf("LR,%a,%u,%u,%u,%u,%d,%d,%d\n", (double)s_lr_running.temporary_permille, s_lr_running.moving_ms,s_lr_running.under_ms,s_lr_running.settled_ms,s_lr_running.episode_ms,s_lr_running.enabled,s_lr_running.active,s_lr_running.spent);
}
int main(int argc,char **argv)
{
  (void)argc;
  if(argv[1] && strcmp(argv[1],"startup")==0)
  {
    for(unsigned trial=0;trial<200;trial++)
    {
      reset();
      const float target=(40.0f+(float)(trial%5)*60.0f);
      const uint32_t base=trial%2 ? 0xFFFFFF00U : 0;
      for(unsigned k=0;k<50;k++)
      {
        const float speed=k<5 ? 0 : target;
        emit(3,target,speed,true,20,base+k*20);
      }
    }
    return 0;
  }
  const float mags[]={0,.5f,1,19.99f,20,40,80,160,240,280,1600,2000};
  const float speeds[]={0,4,19,21,49,50,100,181,250,500,2000,-2000};
  const unsigned periods[]={0,1,10,20,59,60,100,101};
  reset();
  for(unsigned w=0;w<4;w++) for(unsigned k=0;k<20000;k++)
  {
    const float sign=k/121%2 ? -1.0f : 1.0f;
    const bool pivot=!(w==3 && sign>0) && !(k/811%2);
    if(k%311==0) { WheelSpeedController_Reset((MotorDriverWheel)w); ChassisController_ClearPivotWheel((MotorDriverWheel)w); }
    if(k%127==0) assert(WheelSpeedController_PrimeOutput((MotorDriverWheel)w,(int16_t)(sign*120)));
    emit(w,sign*mags[k/17%12],sign*speeds[k/37%12],pivot,periods[k%8],0xFFFF0000U+k*20);
  }
  return 0;
}
