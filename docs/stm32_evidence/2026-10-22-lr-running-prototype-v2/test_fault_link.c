/* OFFLINE: real fault manager/runtime, synthetic coherent policy snapshots.
 * Policy acquisition itself is exercised by audit_fault.c through the real
 * chassis. Here private seeding isolates consumer boundary/reset semantics. */
#include "prototype/wheel_speed_control_config.h"
#include "prototype/wheel_speed_controller.h"
#define main ExistingFrontRecoverySafetyMain
#include "../../../Tools/tests/motion/test_front_recovery_safety.c"
#undef main
#include "prototype/chassis_controller.c"
#include "prototype/wheel_speed_controller.c"

static float targets[4], speeds[4];
static int16_t outputs[4];
static void Setup(uint32_t base, unsigned wheel, float target, bool owned)
{
  Init(base); WheelSpeedController_Init();
  memset(targets,0,sizeof(targets)); memset(speeds,0,sizeof(speeds));
  memset(outputs,0,sizeof(outputs)); targets[wheel]=target;
  s_lr_running.enabled=owned;
  s_states[1].target_counts_per_second=targets[1];
}
static void Observe(uint32_t now)
{
  Heartbeat(now);
  FaultManager_ObserveMotion(now,true,targets,speeds,outputs);
  FaultManager_Process(now);
}
static void Expect(bool fault)
{
  assert(FaultManager_HasCriticalFault()==fault);
  FaultManagerStatus s; assert(FaultManager_GetStatus(&s));
  assert(s.latched_faults==(fault ? FAULT_MANAGER_FAULT_WHEEL_STALL : 0));
}
static void TestExactDeadline(uint32_t base)
{
  Setup(base,1,-200,true);
  for(unsigned t=0;t<=1750;t++)
  {
    Observe(base+t); Expect(t>=1750);
  }
  FaultManagerStatus s; assert(FaultManager_GetStatus(&s));
  assert(s.latched_stalled_wheel_mask==2);
  assert(s.first_fault_ms==base+1750);
  /* Dropping the current condition never clears the latched fault. */
  speeds[1]=-200; Observe(base+1751); Expect(true);
}
static void TestScopeAndLegacy(void)
{
  for(unsigned wheel=0;wheel<4;wheel++) for(int sign=-1;sign<=1;sign+=2)
    for(unsigned owned=0;owned<2;owned++) for(unsigned high=0;high<2;high++)
    {
      Setup(0,wheel,240.0f*sign,owned!=0);
      outputs[wheel]=high ? (int16_t)(100*sign) : 0;
      const bool trips=high || (wheel==1 && sign<0 && owned);
      for(unsigned t=0;t<=1760;t+=10) { Observe(t); Expect(trips && t>=1750); }
    }
  const float target[]={-199.99f,-240,-240,-240};
  const float measured[]={0,-36,-100,-239};
  for(unsigned k=0;k<4;k++)
  {
    Setup(0,1,target[k],true); speeds[1]=measured[k];
    for(unsigned t=0;t<=3000;t+=10) { Observe(t); Expect(false); }
  }
  /* Snapshot mismatch must not qualify the extra path. */
  Setup(0,1,-240,true); s_states[1].target_counts_per_second=-239;
  for(unsigned t=0;t<=3000;t+=10) { Observe(t); Expect(false); }
}
static void TestInterruptedConditions(uint32_t base)
{
  for(unsigned reset=0;reset<6;reset++)
  {
    Setup(base,1,-240,true);
    for(unsigned t=0;t<=1690;t+=10) { Observe(base+t); Expect(false); }
    const uint32_t gap=base+1700;
    switch(reset)
    {
      case 0: speeds[1]=-240; Observe(gap); speeds[1]=0; break;
      case 1: s_lr_running.enabled=false; Observe(gap); s_lr_running.enabled=true; break;
      case 2: targets[1]=-199; s_states[1].target_counts_per_second=-199;
              Observe(gap); targets[1]=-240; s_states[1].target_counts_per_second=-240; break;
      case 3: FaultManager_ObserveMotion(gap,false,targets,speeds,outputs); break;
      case 4: FaultManager_ObserveMotion(gap,true,NULL,speeds,outputs); break;
      default: targets[1]=240; Observe(gap); targets[1]=-240; break;
    }
    Expect(false);
    /* Runtime-inactive/NULL restart also re-arms the original 1000ms grace.
     * A sign change suppresses one observation; other gaps reset only 750ms. */
    const unsigned expected=reset==3 || reset==4 ? 3460 : reset==5 ? 2470 : 2460;
    for(unsigned t=1710;t<=expected;t+=10)
    {
      Observe(base+t); Expect(t>=expected);
    }
  }
  Setup(base,1,-240,true);
  for(unsigned t=0;t<=6000;t+=10)
  {
    /* Repeated 160ms zero-count-like windows with real recovery between. */
    speeds[1]=(t>=1000 && (t-1000)%300<160) ? 0 : -240;
    Observe(base+t); Expect(false);
  }
}
static void TestQueryLifetime(void)
{
  WheelSpeedController_Init();
  assert(!WheelSpeedController_IsLrRunningSupportOwned(-240));
  int16_t pwm;
  for(unsigned k=0;k<2;k++) assert(WheelSpeedController_UpdateWithMode(
    MOTOR_DRIVER_WHEEL_LR,-240,-240,0,WHEEL_SPEED_CONTROL_MODE_PIVOT,20,&pwm));
  assert(WheelSpeedController_SelectLrRunningSupport(true));
  assert(WheelSpeedController_IsLrRunningSupportOwned(-240));
  WheelSpeedLrRunningState before=s_lr_running;
  for(unsigned k=0;k<100;k++)
  {
    assert(WheelSpeedController_IsLrRunningSupportOwned(-240));
    assert(!WheelSpeedController_IsLrRunningSupportOwned(-239));
    assert(!WheelSpeedController_IsLrRunningSupportOwned(240));
    assert(!WheelSpeedController_IsLrRunningSupportOwned(NAN));
    assert(!WheelSpeedController_IsLrRunningSupportOwned(INFINITY));
  }
  assert(memcmp(&before,&s_lr_running,sizeof(before))==0);
  assert(WheelSpeedController_UpdateWithMode(MOTOR_DRIVER_WHEEL_LR,
    -240,-240,0,WHEEL_SPEED_CONTROL_MODE_NORMAL,20,&pwm));
  assert(!WheelSpeedController_IsLrRunningSupportOwned(-240));
  s_lr_running.enabled=true; assert(WheelSpeedController_Reset(MOTOR_DRIVER_WHEEL_LR));
  assert(!WheelSpeedController_IsLrRunningSupportOwned(-240));
  s_lr_running.enabled=true;
  assert(WheelSpeedController_UpdateWithMode(MOTOR_DRIVER_WHEEL_LR,
    240,240,0,WHEEL_SPEED_CONTROL_MODE_PIVOT,20,&pwm));
  assert(!WheelSpeedController_IsLrRunningSupportOwned(240));
}
int main(void)
{
  TestExactDeadline(0); TestExactDeadline(0xFFFFFC00U);
  puts("PASS: LR owned low-output stall trips at exact 1000+750ms, including tick wrap; recovery never clears latch");
  TestScopeAndLegacy();
  puts("PASS: legacy output-qualified guard retained for every wheel/direction; new path only owned LR reverse >=200cps below 15%, coherent target required");
  TestInterruptedConditions(0); TestInterruptedConditions(0xFFFFFC00U);
  puts("PASS: speed/ownership/target/direction/inactive/NULL reset semantics, no accumulation of repeated 160ms gaps, including tick wrap");
  TestQueryLifetime();
  puts("PASS: ownership query read-only; rejects stale target/nonfinite, clears on NORMAL/sign/reset; acquisition uses actual PI updates");
  return 0;
}
