/* Real runtime/controller/safety, supplied wheel feedback; not a physical plant. */
#define main HistoricalFixtureMain
#include "unified_runtime_fixture.h"
#undef main
#include "chassis_config.h"
int main(void) {
  for(unsigned wrap=0;wrap<2;wrap++) for(int sign=-1;sign<=1;sign+=2) {
    uint32_t base=wrap?0xfffffc00U:10000U;
    Init(base);
    ChassisVelocityCommand command={0,sign*.4f};ChassisWheelTargets expected;
    assert(ChassisKinematics_ToWheelTargets(&command,1600,&expected));
    for(unsigned ms=0;ms<=15200;ms+=20) {
      uint32_t now=base+ms;Heartbeat(now);
      ChassisControllerStatus before;assert(ChassisController_GetStatus(&before));
      for(unsigned w=0;w<4;w++) feedback[w]=before.targets.counts_per_second[w];
      assert(ChassisRuntime_SetVelocity(0,sign*.4f,now));
      ChassisRuntime_Process(now,false);Safety(now);
      assert(Runtime().state==CHASSIS_RUNTIME_STATE_ACTIVE && !FaultManager_HasCriticalFault());
      ChassisControllerStatus after;assert(ChassisController_GetStatus(&after));
      if(ms>=400) for(unsigned w=0;w<4;w++) {
        assert(fabsf(after.targets.counts_per_second[w]-expected.counts_per_second[w])<.001f);
        assert(abs(motor.command_permille[w])<=300);
      }
    }
    ChassisRuntime_Stop(base+15201);AssertDisabled();
    assert(Runtime().state==CHASSIS_RUNTIME_STATE_IDLE);
  }
  puts("PASS: real runtime both turn directions/wrap retain .40rad/s and +/-604cps targets for15.2s supplied-feedback simulation, then STOP disables four outputs");
  return 0;
}
