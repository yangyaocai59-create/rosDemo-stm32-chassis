/* Two separate executables produce identical streams on unmodified scope. */
#ifdef BASELINE
#include "../../../docs/stm32_evidence/2026-10-22-v01818-lr-rf-review/source_snapshot/wheel_speed_control_config.h"
#include "../../../docs/stm32_evidence/2026-10-22-v01818-lr-rf-review/source_snapshot/wheel_speed_controller.h"
#else
#include "../../../Application/Chassis/wheel_speed_control_config.h"
#include "../../../Application/Chassis/wheel_speed_controller.h"
#endif
#define main ExistingRuntimeRegressionMain
#ifdef BASELINE
#include "../../../docs/stm32_evidence/2026-10-22-v01819-offline/baseline_0.18.18/test_runtime_motion.c"
#else
#include "test_runtime_motion.c"
#endif
#undef main
#ifdef BASELINE
#include "../../../docs/stm32_evidence/2026-10-22-v01818-lr-rf-review/source_snapshot/chassis_controller.c"
#include "../../../docs/stm32_evidence/2026-10-22-v01818-lr-rf-review/source_snapshot/wheel_speed_controller.c"
#else
#include "../../../Application/Chassis/chassis_controller.c"
#include "../../../Application/Chassis/wheel_speed_controller.c"
#endif

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
}
int main(int argc,char **argv)
{
  (void)argc;
  if(argv[1] && strcmp(argv[1],"startup")==0)
  {
    for(unsigned trial=0;trial<200;trial++)
    {
      reset();
      const float target=-(40.0f+(float)(trial%5)*60.0f);
      const uint32_t base=trial%2 ? 0xFFFFFF00U : 0;
      for(unsigned k=0;k<50;k++)
      {
        const float speed=k<5 ? 0 : target;
        emit(1,target,speed,true,20,base+k*20);
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
    /* RR forward is intentionally changed in .20 and covered against .19
     * by run_rr_running_tests.py, not this older .18 differential. */
    const bool pivot=!(w==1 && sign<0) && !(w==3 && sign>0) && !(k/811%2);
    if(k%311==0) { WheelSpeedController_Reset((MotorDriverWheel)w); ChassisController_ClearPivotWheel((MotorDriverWheel)w); }
    if(k%127==0) assert(WheelSpeedController_PrimeOutput((MotorDriverWheel)w,(int16_t)(sign*120)));
    emit(w,sign*mags[k/17%12],sign*speeds[k/37%12],pivot,periods[k%8],0xFFFF0000U+k*20);
  }
  return 0;
}
