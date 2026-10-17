/* Actual runtime/controller/kinematics/PI/profile/trace; peripherals mocked.
 * Not a physical-plant, ROS/USB or hardware-safety validation. */
#include <assert.h>
#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "chassis_runtime.h"
#include "wheel_speed_estimator.h"
#include "debug_log.h"
#include "motion_trace.h"

static unsigned depth, enable_count, trace_count;
static bool estop, inject_stop, health_ok=true;
static MotorDriverStatus motor={.initialized=true};
static uint32_t time_ms;
void Test_EnterCritical(void) { ++depth; }
void Test_ExitCritical(void)
{
  assert(depth!=0U); --depth;
  if (depth==0U && inject_stop)
  { inject_stop=false; ChassisRuntime_Stop(time_ms); }
}
bool DebugLog_Printf(DebugLog_Level level,const char *format,...)
{
  (void)level; assert(depth==0U);
  if (strncmp(format,"TRACE t=",8)==0) ++trace_count;
  return true;
}
bool EmergencyStop_CanEnableMotor(void) { return !estop; }
bool EmergencyStop_IsLatched(void) { return estop; }
bool FaultManager_CanEnableMotion(void) { return health_ok; }
bool FaultManager_Clear(uint32_t now) { (void)now; return health_ok; }
void MotorDriver_Disable(void)
{ motor.standby_enabled=false; memset(motor.command_permille,0,sizeof(motor.command_permille)); }
bool MotorDriver_Enable(void)
{ if(estop) return false; motor.standby_enabled=true; ++enable_count; return true; }
bool MotorDriver_SetCommand(MotorDriverWheel wheel,int16_t output)
{
  assert((unsigned)wheel<4U && abs(output)<=300);
  if(estop || (!motor.standby_enabled && output!=0)) return false;
  motor.command_permille[wheel]=output; return true;
}
bool MotorDriver_GetStatus(MotorDriverStatus *status) { *status=motor; return true; }
bool WheelSpeedEstimator_GetSnapshot(WheelSpeedState states[ENCODER_WHEEL_COUNT])
{
  memset(states,0,sizeof(*states)*ENCODER_WHEEL_COUNT);
  for(unsigned i=0;i<ENCODER_WHEEL_COUNT;i++)
  { states[i].valid=true; states[i].sample_period_ms=10U; }
  return true;
}
static ChassisRuntimeStatus Status(void)
{ ChassisRuntimeStatus s; assert(ChassisRuntime_GetStatus(&s)); return s; }
static void Init(uint32_t now)
{
  time_ms=now; estop=false; health_ok=true; inject_stop=false;
  ChassisController_Init(); assert(ChassisRuntime_Init(now));
}
static void Start(uint32_t now,float goal)
{
  time_ms=now;
  assert(ChassisRuntime_SetVelocity(goal,0,now));
  ChassisRuntime_Process(now,false);
  assert(Status().state==CHASSIS_RUNTIME_STATE_ACTIVE);
}
static void AssertStopped(ChassisRuntimeStopReason reason)
{
  ChassisRuntimeStatus s=Status();
  assert(s.state==CHASSIS_RUNTIME_STATE_IDLE && s.last_stop_reason==reason);
  assert(!s.command_valid && !motor.standby_enabled);
  assert(s.applied_command.linear_x_m_s==0.0f);
  for(unsigned i=0;i<4;i++) assert(motor.command_permille[i]==0);
}
int main(void)
{
  Init(0); Start(10,0.04f);
  assert(fabsf(Status().applied_command.linear_x_m_s-0.002f)<1e-6f);
  for(time_ms=20;time_ms<=3000;time_ms+=10)
  {
    if(time_ms%50==0) assert(ChassisRuntime_SetVelocity(0.04f,0,time_ms));
    ChassisRuntime_Process(time_ms,false);
    ChassisRuntimeStatus s=Status();
    assert(s.state==CHASSIS_RUNTIME_STATE_ACTIVE &&
           s.requested_command.linear_x_m_s==0.04f);
    if(time_ms>=200) assert(fabsf(s.applied_command.linear_x_m_s-0.04f)<1e-6f);
    MotionTrace_Record(time_ms);
  }
  assert(enable_count==1 && !MotionTrace_RequestDump());
  ChassisRuntime_Stop(3001);
  AssertStopped(CHASSIS_RUNTIME_STOP_EXPLICIT); MotionTrace_Record(3001);
  assert(MotionTrace_RequestDump()); assert(!MotionTrace_RequestDump());
  while(MotionTrace_ProcessDump()) {}
  assert(trace_count==128U);
  puts("PASS: real runtime+controller/profile persistent 40mm/s, no re-enable/reset, bounded trace wrap/freeze/dump");

  Start(4000,-0.04f);
  assert(fabsf(Status().applied_command.linear_x_m_s+0.002f)<1e-6f);
  assert(ChassisRuntime_SetVelocity(0,0,4010));
  ChassisRuntime_Process(4010,false); AssertStopped(CHASSIS_RUNTIME_STOP_ZERO_COMMAND);
  Start(4100,0.04f); ChassisRuntime_StopForTransportLoss(4101);
  AssertStopped(CHASSIS_RUNTIME_STOP_TRANSPORT_LOST);
  Start(4200,0.04f); ChassisRuntime_SafetyProcess(4450);
  AssertStopped(CHASSIS_RUNTIME_STOP_COMMAND_TIMEOUT);
  Start(4500,0.04f); ChassisRuntime_LatchHealthFault(4501);
  assert(Status().state==CHASSIS_RUNTIME_STATE_FAULT && !motor.standby_enabled);
  assert(ChassisRuntime_ClearFault(4502));
  Start(4600,0.04f); estop=true; ChassisRuntime_SafetyProcess(4601);
  assert(Status().state==CHASSIS_RUNTIME_STATE_ESTOP && !motor.standby_enabled);
  puts("PASS: explicit STOP, zero, transport loss, exact 250ms timeout, health fault and ESTOP bypass ramp immediately");

  Init(5000); assert(ChassisRuntime_SetVelocity(0.04f,0,5000));
  const unsigned enables_before=enable_count;
  inject_stop=true; time_ms=5010; ChassisRuntime_Process(5010,false);
  AssertStopped(CHASSIS_RUNTIME_STOP_EXPLICIT); assert(enable_count==enables_before);
  Start(5100,0.04f);
  inject_stop=true; time_ms=5110; ChassisRuntime_Process(5110,false);
  AssertStopped(CHASSIS_RUNTIME_STOP_EXPLICIT);
  Start(5200,0.04f);
  inject_stop=true; time_ms=5301; ChassisRuntime_Process(5301,false);
  AssertStopped(CHASSIS_RUNTIME_STOP_EXPLICIT);
  Start(5400,0.04f);
  inject_stop=true; time_ms=5410; ChassisRuntime_SafetyProcess(5410);
  AssertStopped(CHASSIS_RUNTIME_STOP_EXPLICIT);
  puts("PASS: STOP preempts stale IDLE/start, ACTIVE step, timing fault and safety snapshot; no restart or replacement fault");

  Init(6000); Start(6000,0.04f); ChassisRuntime_Process(6101,false);
  assert(Status().state==CHASSIS_RUNTIME_STATE_FAULT &&
         Status().controller_fault==CHASSIS_CONTROLLER_FAULT_CONTROL_TIMING &&
         !motor.standby_enabled);
  ChassisControllerSafetyState compact;
  ChassisControllerStatus full;
  assert(!ChassisController_GetSafetyState(NULL));
  assert(ChassisController_GetStatus(&full));
  assert(ChassisController_GetSafetyState(&compact) && !compact.active &&
         compact.active==full.active && compact.fault==full.fault);
  Init(6200); Start(6200,0.04f);
  assert(ChassisController_GetSafetyState(&compact) && compact.active &&
         compact.fault==CHASSIS_CONTROLLER_FAULT_NONE);
  /* Low-level failure must still propagate through the compact safety view. */
  ChassisController_Process(6301U);
  assert(ChassisController_GetSafetyState(&compact) && !compact.active &&
         compact.fault==CHASSIS_CONTROLLER_FAULT_CONTROL_TIMING);
  ChassisRuntime_SafetyProcess(6302U);
  assert(Status().state==CHASSIS_RUNTIME_STATE_FAULT &&
         Status().controller_fault==CHASSIS_CONTROLLER_FAULT_CONTROL_TIMING &&
         !motor.standby_enabled);
  puts("PASS: compact safety snapshot preserves active/fault, NULL validation and nested timing-fault shutdown");
  Init(0xFFFFFFF0U); Start(0xFFFFFFF0U,0.04f);
  ChassisRuntime_Process(0xFFFFFFFAU,false); ChassisRuntime_Process(4U,false);
  assert(Status().state==CHASSIS_RUNTIME_STATE_ACTIVE);
  ChassisRuntime_Stop(5U); AssertStopped(CHASSIS_RUNTIME_STOP_EXPLICIT);
  assert(depth==0);
  puts("PASS: real >100ms control timing fault still disables output; source/profile tick wrap works");
  return 0;
}
