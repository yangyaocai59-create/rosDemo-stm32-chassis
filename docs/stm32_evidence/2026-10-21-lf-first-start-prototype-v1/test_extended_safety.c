/* Existing real runtime harness/peripheral stubs, plus new 400ms phase. */
#define main ExistingRuntimeRegressionMain
#include "Tools/tests/motion/test_runtime_motion.c"
#undef main
#include "prototype/wheel_speed_controller.c"

static void extended_shutdown(uint32_t base,unsigned action)
{
  Init(base);
  for(unsigned elapsed=0;elapsed<=400;elapsed+=20)
  {
    time_ms=base+elapsed;
    assert(ChassisRuntime_SetVelocity(0,.3f,time_ms));
    ChassisRuntime_Process(time_ms,false);
  }
  assert(s_lf_first_start.first_recovery && s_front_pivot[0].recovering);
  assert(s_front_pivot[0].recovery_ms==300);
  time_ms=base+401;
  switch(action)
  {
    case 0: ChassisRuntime_Stop(time_ms); break;
    case 1: assert(ChassisRuntime_SetVelocity(0,0,time_ms));
      ChassisRuntime_Process(time_ms,false); break;
    case 2: ChassisRuntime_StopForTransportLoss(time_ms); break;
    case 3: ChassisRuntime_LatchHealthFault(time_ms); break;
    case 4: estop=true; ChassisRuntime_SafetyProcess(time_ms); break;
    case 5: ChassisRuntime_SafetyProcess(base+650); break;
    case 6: ChassisRuntime_Process(base+501,false); break;
    default: feedback[0]=3000; ChassisRuntime_Process(base+420,false); break;
  }
  assert(!motor.standby_enabled && !ChassisController_IsActive());
  for(unsigned w=0;w<4;w++) assert(motor.command_permille[w]==0);
  assert(!s_lf_first_start.initialized && !s_lf_first_start.first_recovery);
  assert(s_front_pivot[0].recovery_permille==0 && s_states[0].integrator_permille==0);
  assert(depth==0);
}
int main(void)
{
  for(unsigned action=0;action<8;action++)
  {
    extended_shutdown(0,action);
    extended_shutdown(0xFFFFFF00U,action);
  }
  puts("PASS: LF extended 400ms first-start phase clears immediately on all 8 real runtime safety paths, including tick wrap; all four outputs disabled");
  return 0;
}
