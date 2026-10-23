/* Production controller + runtime + fault manager; prescribed feedback,
 * mocked GPIO/encoder. No serial, real hardware, or physical motor model. */
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "chassis_runtime.h"
#include "fault_manager.h"
#include "wheel_speed_estimator.h"
#include "debug_log.h"

static unsigned depth;
static bool estop;
static MotorDriverStatus motor={.initialized=true};
static float feedback[4];
void Test_EnterCritical(void) { ++depth; }
void Test_ExitCritical(void) { assert(depth); --depth; }
bool DebugLog_Printf(DebugLog_Level level,const char *format,...)
{ (void)level; (void)format; assert(!depth); return true; }
bool EmergencyStop_CanEnableMotor(void) { return !estop; }
bool EmergencyStop_IsLatched(void) { return estop; }
void MotorDriver_Disable(void)
{ motor.standby_enabled=false; memset(motor.command_permille,0,sizeof(motor.command_permille)); }
bool MotorDriver_Enable(void)
{ if(estop) return false; motor.standby_enabled=true; return true; }
bool MotorDriver_SetCommand(MotorDriverWheel wheel,int16_t command)
{
  assert((unsigned)wheel<4 && abs(command)<=300);
  if(estop || (!motor.standby_enabled && command)) return false;
  motor.command_permille[wheel]=command; return true;
}
bool MotorDriver_GetStatus(MotorDriverStatus *state) { *state=motor; return true; }
bool WheelSpeedEstimator_GetSnapshot(WheelSpeedState states[ENCODER_WHEEL_COUNT])
{
  memset(states,0,sizeof(*states)*ENCODER_WHEEL_COUNT);
  for(unsigned w=0;w<4;w++)
  { states[w].valid=true; states[w].sample_period_ms=20;
    states[w].filtered_counts_per_second=feedback[w]; }
  return true;
}
static void Heartbeat(uint32_t now)
{
  for(unsigned t=0;t<FAULT_MANAGER_TASK_COUNT;t++)
    FaultManager_Heartbeat((FaultManagerTask)t,now);
}
static ChassisRuntimeStatus Runtime(void)
{ ChassisRuntimeStatus s; assert(ChassisRuntime_GetStatus(&s)); return s; }
static void Init(uint32_t now)
{
  estop=false; memset(feedback,0,sizeof(feedback));
  FaultManager_Init(now); FaultManager_MarkSystemInitComplete(now); Heartbeat(now);
  ChassisController_Init(); assert(ChassisRuntime_Init(now));
}
static void Safety(uint32_t now)
{
  ChassisRuntimeStatus runtime=Runtime(); ChassisControllerStatus controller;
  assert(ChassisController_GetStatus(&controller));
  FaultManager_ObserveMotion(now,runtime.state==CHASSIS_RUNTIME_STATE_ACTIVE,
    controller.targets.counts_per_second,controller.measured_counts_per_second,
    controller.output_permille);
  FaultManager_Process(now);
  if(FaultManager_HasCriticalFault()) ChassisRuntime_LatchHealthFault(now);
  ChassisRuntime_SafetyProcess(now); ChassisController_SafetyProcess(now);
}
static void AssertDisabled(void)
{
  assert(!motor.standby_enabled && !ChassisController_IsActive());
  for(unsigned w=0;w<4;w++) assert(motor.command_permille[w]==0);
  assert(!depth);
}
static void FeedbackExcept(int stuck)
{
  ChassisControllerStatus controller;
  assert(ChassisController_GetStatus(&controller));
  for(unsigned w=0;w<4;w++)
    feedback[w]=((int)w==stuck) ? 0 : controller.targets.counts_per_second[w];
}
static void TestQualifiedFailure(unsigned wheel,float angular,uint32_t base)
{
  Init(base);
  uint32_t trip=0;
  for(unsigned elapsed=0;elapsed<=2200;elapsed+=20)
  {
    const uint32_t now=base+elapsed; Heartbeat(now); FeedbackExcept((int)wheel);
    assert(ChassisRuntime_SetVelocity(0,angular,now));
    ChassisRuntime_Process(now,false); Safety(now);
    if(Runtime().state==CHASSIS_RUNTIME_STATE_FAULT) { trip=elapsed; break; }
  }
  assert(trip>=1500 && trip<=1600);
  FaultManagerStatus fault;
  assert(FaultManager_GetStatus(&fault));
  assert(fault.latched_faults==FAULT_MANAGER_FAULT_WHEEL_STALL &&
    fault.latched_stalled_wheel_mask==(1U<<wheel));
  assert(Runtime().fault==CHASSIS_RUNTIME_FAULT_HEALTH_MANAGER);
  assert(!FaultManager_CanEnableMotion()); AssertDisabled();
  /* Controller reset clears the maneuver report, NOT the health fault latch. */
  WheelSpeedFrontPivotState front;
  assert(WheelSpeedController_GetFrontPivotState((MotorDriverWheel)wheel,&front));
  assert(!front.active && !front.recovery_failed);
  for(unsigned elapsed=trip+20;elapsed<=trip+400;elapsed+=20)
  {
    const uint32_t now=base+elapsed; Heartbeat(now);
    assert(!ChassisRuntime_SetVelocity(0,angular,now));
    ChassisRuntime_Process(now,false); Safety(now); AssertDisabled();
  }
  const uint32_t clear=base+trip+420;
  Heartbeat(clear); Safety(clear);
  assert(ChassisRuntime_ClearFault(clear));
  assert(Runtime().state==CHASSIS_RUNTIME_STATE_IDLE && FaultManager_CanEnableMotion());
  /* Explicit clear alone does not resume the old command. */
  AssertDisabled();
  assert(ChassisRuntime_SetVelocity(0,angular,clear+20));
  for(unsigned elapsed=20;elapsed<=600;elapsed+=20)
  {
    uint32_t now=clear+elapsed; Heartbeat(now); FeedbackExcept(-1);
    assert(ChassisRuntime_SetVelocity(0,angular,now));
    ChassisRuntime_Process(now,false); Safety(now);
    assert(Runtime().state==CHASSIS_RUNTIME_STATE_ACTIVE);
  }
  ChassisRuntime_Stop(clear+620); AssertDisabled();
}
static void TestNonFailureModes(float linear,float angular,int stuck)
{
  Init(10000);
  for(unsigned elapsed=0;elapsed<=3000;elapsed+=20)
  {
    uint32_t now=10000+elapsed; Heartbeat(now); FeedbackExcept(stuck);
    assert(ChassisRuntime_SetVelocity(linear,angular,now));
    ChassisRuntime_Process(now,false); Safety(now);
    assert(Runtime().state==CHASSIS_RUNTIME_STATE_ACTIVE);
    assert(!FaultManager_HasCriticalFault());
  }
  ChassisRuntime_Stop(13001); AssertDisabled();
}
static void TestRecoveryBeforeDeadline(unsigned wheel)
{
  Init(20000);
  for(unsigned elapsed=0;elapsed<=3000;elapsed+=20)
  {
    uint32_t now=20000+elapsed; Heartbeat(now);
    /* Two zero-feedback episodes, each shorter than the deadline, separated
     * by measured normal motion. They must not add up to a false trip. */
    const bool stopped=elapsed<=1200 || (elapsed>=1600 && elapsed<=2600);
    FeedbackExcept(stopped ? (int)wheel : -1);
    assert(ChassisRuntime_SetVelocity(0,0.2f,now));
    ChassisRuntime_Process(now,false); Safety(now);
    assert(Runtime().state==CHASSIS_RUNTIME_STATE_ACTIVE);
  }
  ChassisRuntime_Stop(23001); AssertDisabled();
}
static void TestLegacyGuard(void)
{
  Init(0); WheelSpeedController_Init();
  float targets[4]={0,280,0,0}, speeds[4]={0}; int16_t outputs[4]={0,150,0,0};
  for(unsigned now=0;now<=1780;now+=20)
  {
    Heartbeat(now); FaultManager_ObserveMotion(now,true,targets,speeds,outputs);
    FaultManager_Process(now);
    assert(FaultManager_HasCriticalFault()==(now>=1760));
  }
  FaultManagerStatus status; assert(FaultManager_GetStatus(&status));
  assert(status.latched_stalled_wheel_mask==(1U<<MOTOR_DRIVER_WHEEL_LR));
}
int main(void)
{
  const unsigned fronts[]={MOTOR_DRIVER_WHEEL_LF,MOTOR_DRIVER_WHEEL_RF};
  for(unsigned w=0;w<2;w++) for(int sign=-1;sign<=1;sign+=2)
  {
    TestQualifiedFailure(fronts[w],(float)sign*0.2f,5000);
    TestQualifiedFailure(fronts[w],(float)sign*0.025f,5000);
    TestQualifiedFailure(fronts[w],(float)sign*0.35f,0xFFFFFC00U);
  }
  for(unsigned w=0;w<2;w++) TestRecoveryBeforeDeadline(fronts[w]);
  TestNonFailureModes(0,0.2f,-1); /* front pivots moving at target */
  TestNonFailureModes(0,0.2f,MOTOR_DRIVER_WHEEL_LR); /* low-speed rear unchanged */
  TestNonFailureModes(0.02f,0,MOTOR_DRIVER_WHEEL_RF); /* NORMAL below old 200 gate */
  TestLegacyGuard();
  puts("PASS: real front policy/fault manager/runtime trip on both fronts/directions at approximately 20/160/280 cps, including tick wrap; all outputs disabled, latch/reject/explicit clear/new command verified");
  puts("PASS: recovery before deadline, moving pivot, rear and NORMAL isolation; legacy 1000ms grace + 750ms stall detection preserved");
  return 0;
}
