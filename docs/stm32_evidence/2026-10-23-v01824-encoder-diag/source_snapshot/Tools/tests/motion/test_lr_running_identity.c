/* Current production control versus validated v2 prototype; no fault consumer. */
#ifdef REFERENCE
/* Do not pull a newer runtime harness into the frozen LR reference.
 * Its new RR API is intentionally absent from the old reference header. */
#include "../../../docs/stm32_evidence/2026-10-22-lr-running-prototype-v2/prototype/wheel_speed_control_config.h"
#include "../../../docs/stm32_evidence/2026-10-22-lr-running-prototype-v2/prototype/wheel_speed_controller.h"
#define main ExistingRuntimeRegressionMain
#include "../../../docs/stm32_evidence/2026-10-22-v01820-offline/baseline_0.18.19/test_runtime_motion.c"
#undef main
#include "../../../docs/stm32_evidence/2026-10-22-lr-running-prototype-v2/prototype/chassis_controller.c"
#include "../../../docs/stm32_evidence/2026-10-22-lr-running-prototype-v2/prototype/wheel_speed_controller.c"
#else
#include "lr_running_fixture.h"
#endif
/* Only the main below varies target/I/dt; no serial or real motor access. */
int main(void)
{
  const unsigned dt[]={1,10,20,59,60,100};
  for(unsigned trial=0;trial<300;trial++)
  {
    WheelSpeedController_Init();
    const float target=-(20.0f+40.0f*(trial%15));
    int16_t pwm;
    for(unsigned k=0;k<2;k++) assert(WheelSpeedController_UpdateWithMode(
      MOTOR_DRIVER_WHEEL_LR,target,-240,0,WHEEL_SPEED_CONTROL_MODE_PIVOT,20,&pwm));
    assert(WheelSpeedController_SelectLrRunningSupport(true));
    s_states[1].integrator_permille=(float)((int)(trial%101)-50);
    for(unsigned k=0;k<100;k++)
    {
      float measured=k<10 ? target : k<40 ? 0 : k<65 ? target*2 : target;
      assert(WheelSpeedController_UpdateWithMode(MOTOR_DRIVER_WHEEL_LR,target,measured,0,
        WHEEL_SPEED_CONTROL_MODE_PIVOT,dt[(trial+k)%6],&pwm));
      WheelSpeedControllerState s=s_states[1]; WheelSpeedLrRunningState r=s_lr_running;
      printf("%u,%u,%d,%a,%a,%a,%u,%u,%u,%u,%d,%d,%d\n",trial,k,pwm,
        (double)s.feedforward_permille,(double)s.integrator_permille,
        (double)r.temporary_permille,r.moving_ms,r.under_ms,r.settled_ms,
        r.episode_ms,r.enabled,r.active,r.spent);
    }
  }
  return 0;
}
