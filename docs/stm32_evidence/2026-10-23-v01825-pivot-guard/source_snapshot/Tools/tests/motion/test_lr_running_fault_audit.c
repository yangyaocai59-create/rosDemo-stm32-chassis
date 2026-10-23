/* Audit the real existing output-qualified stall detector, not just STOP. */
#ifdef BASELINE
#include "../../../docs/stm32_evidence/2026-10-22-v01818-lr-rf-review/source_snapshot/wheel_speed_control_config.h"
#include "../../../docs/stm32_evidence/2026-10-22-v01818-lr-rf-review/source_snapshot/wheel_speed_controller.h"
#else
#include "../../../Application/Chassis/wheel_speed_control_config.h"
#include "../../../Application/Chassis/wheel_speed_controller.h"
#endif
#define main ExistingFrontRecoverySafetyMain
#include "test_front_recovery_safety.c"
#undef main
#ifdef BASELINE
#include "../../../docs/stm32_evidence/2026-10-22-v01818-lr-rf-review/source_snapshot/chassis_controller.c"
#include "../../../docs/stm32_evidence/2026-10-22-v01818-lr-rf-review/source_snapshot/wheel_speed_controller.c"
#else
#include "../../../Application/Chassis/chassis_controller.c"
#include "../../../Application/Chassis/wheel_speed_controller.c"
#endif
int main(int argc,char **argv)
{
  const uint32_t base=argc>2 ? (uint32_t)strtoul(argv[2],NULL,0) : 0;
  Init(base);
  const float seed_i=argc>1 ? strtof(argv[1],NULL) : 0.0f;
  assert(fabsf(seed_i)<=50);
  unsigned trip=0;
  puts("t,state,lr_pwm,lr_i");
  for(unsigned t=0;t<=6000;t+=20)
  {
    const uint32_t now=base+t;
    Heartbeat(now); FeedbackExcept(t>=1000 ? 1 : -1);
    if(t>=100 && t<400) feedback[1]=-500;
    assert(ChassisRuntime_SetVelocity(0,.3f,now));
    ChassisRuntime_Process(now,false);
    if(t==900)
    {
      WheelSpeedControllerState prior;
      assert(WheelSpeedController_GetState(MOTOR_DRIVER_WHEEL_LR,&prior));
      /* Captured/retained integrator history is seeded through the existing
       * API, not by altering the fault manager or its thresholds. Leave
       * 100 ms at exact target for output slew to settle before the stall. */
      assert(fabsf(WheelSpeedController_ReleaseFeedforwardBias(MOTOR_DRIVER_WHEEL_LR,
        seed_i-prior.integrator_permille))<0.001f);
    }
#ifndef BASELINE
    if(t==980) assert(s_lr_running.enabled);
#endif
    Safety(now);
    WheelSpeedControllerState control;
    assert(WheelSpeedController_GetState(MOTOR_DRIVER_WHEEL_LR,&control));
    printf("%u,%d,%d,%.9f\n",t,Runtime().state,control.command_permille,control.integrator_permille);
    if(Runtime().state==CHASSIS_RUNTIME_STATE_FAULT) { trip=t; break; }
  }
  assert(trip>1000);
  FaultManagerStatus status; assert(FaultManager_GetStatus(&status));
  assert(status.latched_stalled_wheel_mask==2 && Runtime().fault==CHASSIS_RUNTIME_FAULT_HEALTH_MANAGER);
  AssertDisabled();
  for(unsigned dt=20;dt<=400;dt+=20)
  {
    const uint32_t now=base+trip+dt;
    Heartbeat(now);
    assert(!ChassisRuntime_SetVelocity(0,.3f,now));
    ChassisRuntime_Process(now,false); Safety(now); AssertDisabled();
    assert(FaultManager_HasCriticalFault());
  }
#ifndef BASELINE
  assert(!s_lr_running.enabled && s_lr_running.temporary_permille==0);
#endif
  /* Simulated explicit clear only: no old command resumes automatically. */
  const uint32_t clear=base+trip+420;
  Heartbeat(clear); Safety(clear);
  assert(ChassisRuntime_ClearFault(clear));
  assert(Runtime().state==CHASSIS_RUNTIME_STATE_IDLE);
  AssertDisabled();
  for(unsigned dt=20;dt<=600;dt+=20)
  {
    const uint32_t now=clear+dt;
    Heartbeat(now); FeedbackExcept(-1);
    assert(ChassisRuntime_SetVelocity(0,.3f,now));
    ChassisRuntime_Process(now,false); Safety(now);
    assert(Runtime().state==CHASSIS_RUNTIME_STATE_ACTIVE);
  }
  ChassisRuntime_Stop(clear+620); AssertDisabled();
  printf("RESULT trip_from_stall_ms=%u\n",trip-1000);
  return 0;
}
