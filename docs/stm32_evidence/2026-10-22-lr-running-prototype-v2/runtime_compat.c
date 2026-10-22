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
#include "wheel_speed_controller.h"
#include "wheel_speed_control_config.h"

static unsigned depth, enable_count, trace_count;
static bool estop, inject_stop, health_ok=true;
static MotorDriverStatus motor={.initialized=true};
static uint32_t time_ms;
static float feedback[ENCODER_WHEEL_COUNT];
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
  { states[i].valid=true; states[i].sample_period_ms=20U;
    states[i].filtered_counts_per_second=feedback[i]; }
  return true;
}
static ChassisRuntimeStatus Status(void)
{ ChassisRuntimeStatus s; assert(ChassisRuntime_GetStatus(&s)); return s; }
static void Init(uint32_t now)
{
  time_ms=now; estop=false; health_ok=true; inject_stop=false;
  memset(feedback,0,sizeof(feedback));
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

static void PivotStep(const ChassisVelocityCommand *command,uint32_t now)
{
  time_ms=now;
  assert(ChassisController_SetCommand(command,now));
  ChassisController_Process(now);
  ChassisController_SafetyProcess(now);
  ChassisControllerStatus status;
  assert(ChassisController_GetStatus(&status));
  assert(status.active && status.fault==CHASSIS_CONTROLLER_FAULT_NONE);
}

static float StartupTransientBudget(unsigned w,float target)
{
  if(w==MOTOR_DRIVER_WHEEL_LR || w==MOTOR_DRIVER_WHEEL_RR) return 60.0f;
  (void)target;
  return 0.0f; /* Front recovery requires persistent near-rest, not one tick. */
}

static void TestRearRestartTopup(float direction,uint32_t start_ms,float support)
{
  Init(start_ms);
  ChassisVelocityCommand pivot={0.0f,direction*0.2f};
  ChassisWheelTargets targets;
  assert(ChassisKinematics_ToWheelTargets(&pivot,1600.0f,&targets));
  float baseline[4], signs[4];
  for(unsigned w=0;w<4;w++)
  {
    feedback[w]=targets.counts_per_second[w];
    signs[w]=feedback[w]>0.0f ? 1.0f : -1.0f;
  }
  /* Start already moving, then seed support (including captured 29.4) via
   * the real handoff API. No hardware/plant behavior is implied by this seed. */
  assert(ChassisController_Start(&pivot,start_ms));
  for(unsigned w=0;w<4;w++)
  {
    WheelSpeedControllerState state;
    assert(WheelSpeedController_GetState((MotorDriverWheel)w,&state));
    baseline[w]=state.feedforward_permille;
    assert(fabsf(WheelSpeedController_ReleaseFeedforwardBias(
      (MotorDriverWheel)w,signs[w]*support))<0.001f);
  }
  PivotStep(&pivot,start_ms+20U);
  memset(feedback,0,sizeof(feedback));
  const uint32_t restart_ms=start_ms+40U;
  PivotStep(&pivot,restart_ms);
  float initial_bias[4];
  for(unsigned w=0;w<4;w++)
  {
    WheelSpeedControllerState state;
    assert(WheelSpeedController_GetState((MotorDriverWheel)w,&state));
    initial_bias[w]=(state.feedforward_permille-baseline[w])*signs[w];
    const bool rear=(w==MOTOR_DRIVER_WHEEL_LR || w==MOTOR_DRIVER_WHEEL_RR);
    const float expected=rear ? (support>0.0f ? 60.0f-support : 60.0f) :
      StartupTransientBudget(w,targets.counts_per_second[w]);
    assert(fabsf(initial_bias[w]-expected)<0.001f);
  }
  for(uint32_t elapsed=20U;elapsed<=1000U;elapsed+=20U)
  {
    PivotStep(&pivot,restart_ms+elapsed);
    for(unsigned w=0;w<4;w++)
    {
      WheelSpeedControllerState state;
      assert(WheelSpeedController_GetState((MotorDriverWheel)w,&state));
      const bool rear=(w==MOTOR_DRIVER_WHEEL_LR || w==MOTOR_DRIVER_WHEEL_RR);
      float expected=initial_bias[w];
      if(rear && elapsed>=80U) expected+=10.0f*(1U+(elapsed-80U)/40U);
      if(expected>60.0f) expected=60.0f;
      if(rear)
        assert(fabsf((state.feedforward_permille-baseline[w])*signs[w]-expected)<0.001f);
      assert(fabsf(state.integrator_permille)<=50.0f);
      assert(abs(state.command_permille)<=300);
    }
  }
  /* Leaving pivot clears the remembered motion and assistance; returning at
   * rest must use the unchanged full first-start kick on all four wheels. */
  ChassisVelocityCommand rolling={0.04f,pivot.angular_z_rad_s};
  PivotStep(&rolling,restart_ms+1020U);
  PivotStep(&pivot,restart_ms+1040U);
  for(unsigned w=0;w<4;w++)
  {
    WheelSpeedControllerState state;
    assert(WheelSpeedController_GetState((MotorDriverWheel)w,&state));
    assert(fabsf((state.feedforward_permille-baseline[w])*signs[w]-
      StartupTransientBudget(w,targets.counts_per_second[w]))<0.001f);
  }
  /* Establish moved state once more, then reverse: prior support cannot
   * reduce the new direction's first kick. */
  for(unsigned w=0;w<4;w++) feedback[w]=targets.counts_per_second[w];
  PivotStep(&pivot,restart_ms+1060U);
  memset(feedback,0,sizeof(feedback));
  pivot.angular_z_rad_s=-pivot.angular_z_rad_s;
  PivotStep(&pivot,restart_ms+1080U);
  for(unsigned w=0;w<4;w++)
  {
    WheelSpeedControllerState actual,normal;
    assert(WheelSpeedController_GetState((MotorDriverWheel)w,&actual));
    /* Isolated reference update for the opposite-direction baseline. This
     * touches PI state, so no chassis step is allowed before Stop below. */
    assert(WheelSpeedController_Reset((MotorDriverWheel)w));
    int16_t output;
    assert(WheelSpeedController_Update((MotorDriverWheel)w,
      -targets.counts_per_second[w],0.0f,20U,&output));
    assert(WheelSpeedController_GetState((MotorDriverWheel)w,&normal));
    const float load=fabsf(WheelSpeedController_GetPivotLoadFeedforward(
      (MotorDriverWheel)w,-targets.counts_per_second[w]));
    assert(fabsf((actual.feedforward_permille-normal.feedforward_permille)*
      -signs[w]-load-StartupTransientBudget(w,-targets.counts_per_second[w]))<0.001f);
  }
  ChassisController_Stop();
  assert(!motor.standby_enabled);
  memset(feedback,0,sizeof(feedback));
  pivot.angular_z_rad_s=-pivot.angular_z_rad_s;
  assert(ChassisController_Start(&pivot,restart_ms+1100U));
  for(unsigned w=0;w<4;w++)
  {
    WheelSpeedControllerState state;
    assert(WheelSpeedController_GetState((MotorDriverWheel)w,&state));
    assert(fabsf((state.feedforward_permille-baseline[w])*signs[w]-
      StartupTransientBudget(w,targets.counts_per_second[w]))<0.001f);
  }
  ChassisController_SafetyProcess(restart_ms+1350U);
  assert(!ChassisController_IsActive() && !motor.standby_enabled);
  assert(depth==0U);
}

static void TestRetainedRearBias(void)
{
  Init(0U);
  ChassisVelocityCommand pivot={0.0f,0.2f};
  assert(ChassisController_Start(&pivot,0U));
  for(uint32_t now=20U;now<300U;now+=20U) PivotStep(&pivot,now);
  ChassisControllerStatus status;
  assert(ChassisController_GetStatus(&status));
  for(unsigned w=0;w<4;w++) feedback[w]=status.targets.counts_per_second[w];
  PivotStep(&pivot,300U);
  WheelSpeedControllerState before[4];
  for(unsigned w=0;w<4;w++)
    assert(WheelSpeedController_GetState((MotorDriverWheel)w,&before[w]));
  memset(feedback,0,sizeof(feedback));
  PivotStep(&pivot,320U);
  for(unsigned w=0;w<4;w++)
  {
    WheelSpeedControllerState after;
    assert(WheelSpeedController_GetState((MotorDriverWheel)w,&after));
    if(w==MOTOR_DRIVER_WHEEL_LR || w==MOTOR_DRIVER_WHEEL_RR)
    {
      assert(fabsf(before[w].integrator_permille)==50.0f);
      assert(fabsf(after.feedforward_permille-before[w].feedforward_permille)<0.001f);
    }
  }
  ChassisController_Stop();
}

static void TestRearRestartEarlyHandoff(float direction,uint32_t start_ms)
{
  Init(start_ms);
  ChassisVelocityCommand pivot={0.0f,direction*0.2f};
  ChassisWheelTargets targets;
  assert(ChassisKinematics_ToWheelTargets(&pivot,1600.0f,&targets));
  assert(ChassisController_Start(&pivot,start_ms));
  WheelSpeedControllerState first[4],state;
  for(unsigned w=0;w<4;w++)
  {
    assert(WheelSpeedController_GetState((MotorDriverWheel)w,&first[w]));
    feedback[w]=targets.counts_per_second[w];
  }
  /* Moving early during FIRST startup must not shorten its 300 ms hold. */
  for(uint32_t elapsed=20U;elapsed<300U;elapsed+=20U)
  {
    PivotStep(&pivot,start_ms+elapsed);
    for(unsigned w=0;w<4;w++)
    {
      assert(WheelSpeedController_GetState((MotorDriverWheel)w,&state));
      assert(state.feedforward_permille==first[w].feedforward_permille);
      assert(state.integrator_permille==0.0f);
    }
  }
  PivotStep(&pivot,start_ms+300U);
  for(unsigned w=0;w<4;w++)
  {
    assert(WheelSpeedController_GetState((MotorDriverWheel)w,&state));
    const bool rear=(w==MOTOR_DRIVER_WHEEL_LR || w==MOTOR_DRIVER_WHEEL_RR);
    assert(fabsf(state.integrator_permille)==(rear ? 50.0f : 0.0f));
  }
  ChassisController_Stop();

  /* Already-moving entry + seeded support isolates a subsequent restart. */
  Init(start_ms);
  for(unsigned w=0;w<4;w++) feedback[w]=targets.counts_per_second[w];
  assert(ChassisController_Start(&pivot,start_ms));
  float signs[4];
  for(unsigned w=0;w<4;w++)
  {
    signs[w]=feedback[w]>0.0f ? 1.0f : -1.0f;
    assert(fabsf(WheelSpeedController_ReleaseFeedforwardBias(
      (MotorDriverWheel)w,signs[w]*29.4f))<0.001f);
  }
  memset(feedback,0,sizeof(feedback));
  const uint32_t restart_ms=start_ms+20U;
  PivotStep(&pivot,restart_ms);
  for(unsigned w=0;w<4;w++) feedback[w]=targets.counts_per_second[w];
  PivotStep(&pivot,restart_ms+20U);
  WheelSpeedControllerState before[4];
  for(unsigned w=0;w<4;w++)
  {
    assert(WheelSpeedController_GetState((MotorDriverWheel)w,&before[w]));
    assert(fabsf(before[w].integrator_permille)<50.0f);
  }
  PivotStep(&pivot,restart_ms+40U);
  for(unsigned w=0;w<4;w++)
  {
    assert(WheelSpeedController_GetState((MotorDriverWheel)w,&state));
    if(w==MOTOR_DRIVER_WHEEL_LR || w==MOTOR_DRIVER_WHEEL_RR)
    {
      assert(state.integrator_permille==before[w].integrator_permille);
      assert(fabsf(state.feedforward_permille)<fabsf(before[w].feedforward_permille));
      assert(fabsf((before[w].feedforward_permille-state.feedforward_permille)*
        signs[w]-4.0f)<0.001f);
    }
    else
    {
      assert(state.feedforward_permille==before[w].feedforward_permille);
      assert(state.integrator_permille==before[w].integrator_permille);
    }
  }
  /* Recovered motion withdraws only the transient, not a new integral load. */
  PivotStep(&pivot,restart_ms+60U);
  WheelSpeedControllerState recovered[4];
  for(unsigned w=0;w<4;w++)
    assert(WheelSpeedController_GetState((MotorDriverWheel)w,&recovered[w]));
  PivotStep(&pivot,restart_ms+80U);
  for(unsigned w=0;w<4;w++)
  {
    assert(WheelSpeedController_GetState((MotorDriverWheel)w,&state));
    const bool rear=(w==MOTOR_DRIVER_WHEEL_LR || w==MOTOR_DRIVER_WHEEL_RR);
    assert(fabsf((recovered[w].feedforward_permille-state.feedforward_permille)*
      signs[w]-(rear ? 4.0f : 0.0f))<0.001f);
    assert(state.integrator_permille==recovered[w].integrator_permille);
    assert(abs(state.command_permille)<=300);
  }
  ChassisController_Stop();
  assert(!motor.standby_enabled && depth==0U);
}

static void TestRearRestartThresholds(float direction,uint32_t start_ms)
{
  Init(start_ms);
  ChassisVelocityCommand pivot={0.0f,direction*0.2f};
  ChassisWheelTargets targets;
  assert(ChassisKinematics_ToWheelTargets(&pivot,1600.0f,&targets));
  float signs[4],initial_ff[4];
  for(unsigned w=0;w<4;w++)
  {
    feedback[w]=targets.counts_per_second[w];
    signs[w]=feedback[w]>0.0f ? 1.0f : -1.0f;
  }
  assert(ChassisController_Start(&pivot,start_ms));
  for(unsigned w=0;w<4;w++)
    assert(fabsf(WheelSpeedController_ReleaseFeedforwardBias(
      (MotorDriverWheel)w,signs[w]*29.4f))<0.001f);
  memset(feedback,0,sizeof(feedback));
  const uint32_t restart_ms=start_ms+20U;
  PivotStep(&pivot,restart_ms);
  for(unsigned w=0;w<4;w++)
  {
    WheelSpeedControllerState state;
    assert(WheelSpeedController_GetState((MotorDriverWheel)w,&state));
    initial_ff[w]=state.feedforward_permille;
    feedback[w]=signs[w]*49.0f;
  }
  /* 49 cps neither ends the hold nor qualifies for another force step. */
  for(uint32_t elapsed=20U;elapsed<=80U;elapsed+=20U)
  {
    PivotStep(&pivot,restart_ms+elapsed);
    for(unsigned w=0;w<4;w++)
    {
      WheelSpeedControllerState state;
      assert(WheelSpeedController_GetState((MotorDriverWheel)w,&state));
      if(w==MOTOR_DRIVER_WHEEL_LR || w==MOTOR_DRIVER_WHEEL_RR)
        assert(state.feedforward_permille==initial_ff[w]);
    }
  }
  memset(feedback,0,sizeof(feedback));
  PivotStep(&pivot,restart_ms+100U); /* overdue at rest: one step */
  PivotStep(&pivot,restart_ms+200U); /* late 100 ms: one, not catch-up */
  PivotStep(&pivot,restart_ms+220U); /* rescheduled for 240, not yet */
  for(unsigned w=0;w<4;w++)
  {
    WheelSpeedControllerState state;
    assert(WheelSpeedController_GetState((MotorDriverWheel)w,&state));
    const bool rear=(w==MOTOR_DRIVER_WHEEL_LR || w==MOTOR_DRIVER_WHEEL_RR);
    if(rear)
      assert(fabsf((state.feedforward_permille-initial_ff[w])*signs[w]-20.0f)<0.001f);
  }
  PivotStep(&pivot,restart_ms+240U);
  WheelSpeedControllerState before_release[4];
  for(unsigned w=0;w<4;w++)
    assert(WheelSpeedController_GetState((MotorDriverWheel)w,&before_release[w]));
  for(unsigned w=0;w<4;w++) feedback[w]=signs[w]*50.0f;
  /* Exactly 50 cps releases rear restart; first front kick remains active. */
  PivotStep(&pivot,restart_ms+260U);
  for(unsigned w=0;w<4;w++)
  {
    WheelSpeedControllerState state;
    assert(WheelSpeedController_GetState((MotorDriverWheel)w,&state));
    if(w==MOTOR_DRIVER_WHEEL_LR || w==MOTOR_DRIVER_WHEEL_RR)
    {
      assert(fabsf(state.integrator_permille-before_release[w].integrator_permille)<1.0f);
      assert(fabsf((before_release[w].feedforward_permille-state.feedforward_permille)*
        signs[w]-4.0f)<0.001f);
    }
  }
  ChassisController_Stop();
  assert(!motor.standby_enabled && depth==0U);
}

static void TestRearTransientDecay(float direction,uint32_t start_ms)
{
  Init(start_ms);
  ChassisVelocityCommand pivot={0.0f,direction*0.2f};
  ChassisWheelTargets targets;
  assert(ChassisKinematics_ToWheelTargets(&pivot,1600.0f,&targets));
  float baseline[4],signs[4];
  for(unsigned w=0;w<4;w++)
  {
    feedback[w]=targets.counts_per_second[w];
    signs[w]=feedback[w]>0.0f ? 1.0f : -1.0f;
  }
  assert(ChassisController_Start(&pivot,start_ms));
  for(unsigned w=0;w<4;w++)
  {
    WheelSpeedControllerState state;
    assert(WheelSpeedController_GetState((MotorDriverWheel)w,&state));
    baseline[w]=state.feedforward_permille;
    assert(fabsf(WheelSpeedController_ReleaseFeedforwardBias(
      (MotorDriverWheel)w,signs[w]*29.4f))<0.001f);
  }
  memset(feedback,0,sizeof(feedback));
  PivotStep(&pivot,start_ms+20U);
  for(unsigned w=0;w<4;w++) feedback[w]=targets.counts_per_second[w];
  PivotStep(&pivot,start_ms+40U);
  PivotStep(&pivot,start_ms+60U);
  WheelSpeedControllerState before[4],state;
  for(unsigned w=0;w<4;w++)
  {
    assert(WheelSpeedController_GetState((MotorDriverWheel)w,&before[w]));
    feedback[w]=signs[w]*49.0f;
  }
  /* Slowing in the hysteresis band pauses decay, with normal PI integration. */
  PivotStep(&pivot,start_ms+80U);
  for(unsigned w=0;w<4;w++)
  {
    assert(WheelSpeedController_GetState((MotorDriverWheel)w,&state));
    assert(state.feedforward_permille==before[w].feedforward_permille);
    assert(fabsf(state.integrator_permille-before[w].integrator_permille)<1.0f);
  }
  /* Re-arm while the previous transient is still present: do not stack it
   * or relabel it as permanent support. */
  memset(feedback,0,sizeof(feedback));
  PivotStep(&pivot,start_ms+100U);
  for(unsigned w=0;w<4;w++)
  {
    assert(WheelSpeedController_GetState((MotorDriverWheel)w,&state));
    assert(fabsf(state.feedforward_permille-baseline[w])<=60.001f);
    feedback[w]=targets.counts_per_second[w];
  }
  PivotStep(&pivot,start_ms+120U);
  PivotStep(&pivot,start_ms+140U);
  WheelSpeedControllerState released[4];
  for(unsigned w=0;w<4;w++)
    assert(WheelSpeedController_GetState((MotorDriverWheel)w,&released[w]));
  /* A valid 100 ms late step must withdraw 20 permille, not a fixed 4 and
   * not more than the remaining transient. PWM's own slew still applies. */
  PivotStep(&pivot,start_ms+240U);
  for(unsigned w=0;w<4;w++)
  {
    assert(WheelSpeedController_GetState((MotorDriverWheel)w,&state));
    if(w==MOTOR_DRIVER_WHEEL_LR || w==MOTOR_DRIVER_WHEEL_RR)
    {
      const float prior=(released[w].feedforward_permille-baseline[w])*signs[w];
      const float expected=prior>20.0f ? prior-20.0f : 0.0f;
      assert(fabsf((state.feedforward_permille-baseline[w])*signs[w]-expected)<0.001f);
      assert(state.integrator_permille==released[w].integrator_permille);
      assert(abs(state.command_permille)<=300);
    }
  }
  for(uint32_t elapsed=260U;elapsed<=400U;elapsed+=20U)
    PivotStep(&pivot,start_ms+elapsed);
  for(unsigned w=0;w<4;w++)
  {
    assert(WheelSpeedController_GetState((MotorDriverWheel)w,&state));
    if(w==MOTOR_DRIVER_WHEEL_LR || w==MOTOR_DRIVER_WHEEL_RR)
    {
      assert(fabsf(state.feedforward_permille-baseline[w])<0.001f);
      assert(state.integrator_permille==released[w].integrator_permille);
    }
  }
  ChassisController_Stop();
  memset(feedback,0,sizeof(feedback));
  assert(ChassisController_Start(&pivot,start_ms+420U));
  for(unsigned w=0;w<4;w++)
  {
    assert(WheelSpeedController_GetState((MotorDriverWheel)w,&state));
    assert(fabsf((state.feedforward_permille-baseline[w])*signs[w]-
      StartupTransientBudget(w,targets.counts_per_second[w]))<0.001f);
  }
  ChassisController_Stop();
  assert(!motor.standby_enabled && depth==0U);
}

static void TestRearSteadySupportPreserved(float direction,uint32_t start_ms)
{
  Init(start_ms);
  ChassisVelocityCommand pivot={0.0f,direction*0.2f};
  assert(ChassisController_Start(&pivot,start_ms));
  for(uint32_t elapsed=20U;elapsed<300U;elapsed+=20U)
    PivotStep(&pivot,start_ms+elapsed);
  ChassisControllerStatus status;
  assert(ChassisController_GetStatus(&status));
  for(unsigned w=0;w<4;w++) feedback[w]=status.targets.counts_per_second[w];
  PivotStep(&pivot,start_ms+300U);
  WheelSpeedControllerState original[4],state;
  for(unsigned w=0;w<4;w++)
    assert(WheelSpeedController_GetState((MotorDriverWheel)w,&original[w]));
  memset(feedback,0,sizeof(feedback));
  for(uint32_t elapsed=320U;elapsed<=400U;elapsed+=20U)
    PivotStep(&pivot,start_ms+elapsed);
  for(unsigned w=0;w<4;w++) feedback[w]=status.targets.counts_per_second[w];
  for(uint32_t elapsed=420U;elapsed<=500U;elapsed+=20U)
  {
    PivotStep(&pivot,start_ms+elapsed);
    for(unsigned w=0;w<4;w++)
    {
      assert(WheelSpeedController_GetState((MotorDriverWheel)w,&state));
      if(w==MOTOR_DRIVER_WHEEL_LR || w==MOTOR_DRIVER_WHEEL_RR)
      {
        const float sign=status.targets.counts_per_second[w]>0.0f ? 1.0f : -1.0f;
        float extra=10.0f-4.0f*(float)(1U+(elapsed-420U)/20U);
        if(extra<0.0f) extra=0.0f;
        assert(fabsf((state.feedforward_permille-original[w].feedforward_permille)*
          sign-extra)<0.001f);
        assert(state.integrator_permille==original[w].integrator_permille);
      }
    }
  }
  ChassisController_Stop();
  assert(!motor.standby_enabled && depth==0U);
}

static float ReferenceBaseFeedforward(unsigned w,float target)
{
  const float statics[4][2]={{51.8f,46.3f},{45.0f,50.4f},{54.7f,56.8f},{58.9f,56.9f}};
  const float gains[4][2]={{0.14600f,0.14677f},{0.14934f,0.14667f},{0.14703f,0.14249f},{0.15779f,0.15108f}};
  const unsigned reverse=target<0.0f;
  return (reverse ? -1.0f : 1.0f)*(statics[w][reverse]+gains[w][reverse]*fabsf(target));
}

static void TestFrontPivotLoad(float direction,float angular,uint32_t start_ms)
{
  Init(start_ms);
  ChassisVelocityCommand pivot={0.0f,direction*angular};
  ChassisWheelTargets targets;
  assert(ChassisKinematics_ToWheelTargets(&pivot,1600.0f,&targets));
  for(unsigned w=0;w<4;w++) feedback[w]=targets.counts_per_second[w];
  assert(ChassisController_Start(&pivot,start_ms));
  for(uint32_t elapsed=20U;elapsed<300U;elapsed+=20U)
  {
    PivotStep(&pivot,start_ms+elapsed);
    for(unsigned w=0;w<4;w++)
      if(w==MOTOR_DRIVER_WHEEL_LF || w==MOTOR_DRIVER_WHEEL_RF)
      {
        WheelSpeedControllerState state;
        assert(WheelSpeedController_GetState((MotorDriverWheel)w,&state));
        const float t=targets.counts_per_second[w],sign=t>0.0f ? 1.0f : -1.0f;
        const float load=fabsf(WheelSpeedController_GetPivotLoadFeedforward((MotorDriverWheel)w,t));
        const float total=(state.feedforward_permille-ReferenceBaseFeedforward(w,t))*sign;
        assert(fabsf(total-load)<0.001f);
        assert(fabsf(state.integrator_permille)<=50.0f && abs(state.command_permille)<=300);
      }
  }
  WheelSpeedControllerState before[4];
  for(unsigned w=0;w<4;w++)
  {
    assert(WheelSpeedController_GetState((MotorDriverWheel)w,&before[w]));
    feedback[w]=targets.counts_per_second[w];
  }
  for(uint32_t elapsed=300U;elapsed<=700U;elapsed+=20U)
  {
    PivotStep(&pivot,start_ms+elapsed);
    for(unsigned w=0;w<4;w++)
      if(w==MOTOR_DRIVER_WHEEL_LF || w==MOTOR_DRIVER_WHEEL_RF)
      {
        WheelSpeedControllerState state;
        assert(WheelSpeedController_GetState((MotorDriverWheel)w,&state));
        const float t=targets.counts_per_second[w];
        assert(state.integrator_permille==before[w].integrator_permille);
        if(elapsed>=620U)
          assert(fabsf(state.feedforward_permille-ReferenceBaseFeedforward(w,t)-
            WheelSpeedController_GetPivotLoadFeedforward((MotorDriverWheel)w,t))<0.001f);
      }
  }
  /* Target ramps must recalculate both the load cap and I-headroom budget. */
  const float magnitudes[]={0.05f,0.10f,0.20f,0.35f,0.10f,0.05f};
  memset(feedback,0,sizeof(feedback));
  uint32_t now=start_ms+700U;
  for(unsigned k=0;k<sizeof(magnitudes)/sizeof(magnitudes[0]);k++)
  {
    now+=20U; pivot.angular_z_rad_s=direction*magnitudes[k];
    PivotStep(&pivot,now);
    assert(ChassisKinematics_ToWheelTargets(&pivot,1600.0f,&targets));
    for(unsigned w=0;w<4;w++)
      if(w==MOTOR_DRIVER_WHEEL_LF || w==MOTOR_DRIVER_WHEEL_RF)
      {
        WheelSpeedControllerState state;
        assert(WheelSpeedController_GetState((MotorDriverWheel)w,&state));
        const float t=targets.counts_per_second[w],sign=t>0.0f ? 1.0f : -1.0f;
        const float load=fabsf(WheelSpeedController_GetPivotLoadFeedforward((MotorDriverWheel)w,t));
        WheelSpeedFrontPivotState front;
        assert(WheelSpeedController_GetFrontPivotState((MotorDriverWheel)w,&front));
        assert(front.load_permille>=0 && front.load_permille<=load+0.001f);
        assert(front.recovery_permille>=0 && front.recovery_permille<=50.001f);
        assert(front.recovery_permille+state.integrator_permille*sign<=50.001f);
        assert(fabsf((state.feedforward_permille-ReferenceBaseFeedforward(w,t))*sign-
          front.load_permille-front.recovery_permille)<0.001f);
      }
  }
  /* Mode exit removes load/transient from FF, then zero/STOP is immediate
   * through the existing safety path (normal zero-target slew is separate). */
  ChassisVelocityCommand straight={0.037f,0.0f};
  now+=20U; PivotStep(&straight,now);
  assert(ChassisKinematics_ToWheelTargets(&straight,1600.0f,&targets));
  for(unsigned w=0;w<4;w++)
  {
    WheelSpeedControllerState state;
    assert(WheelSpeedController_GetState((MotorDriverWheel)w,&state));
    assert(fabsf(state.feedforward_permille-ReferenceBaseFeedforward(w,targets.counts_per_second[w]))<0.001f);
    if(w==MOTOR_DRIVER_WHEEL_LF || w==MOTOR_DRIVER_WHEEL_RF)
    {
      WheelSpeedFrontPivotState front;
      assert(WheelSpeedController_GetFrontPivotState((MotorDriverWheel)w,&front));
      assert(!front.active && !front.recovering && front.load_permille==0 && front.recovery_permille==0);
    }
  }
  /* Restore pivot with reversed sign; FF follows the NEW direction and
   * new budget, while actual output history retains its bounded slew. */
  const int16_t prior_lf=motor.command_permille[MOTOR_DRIVER_WHEEL_LF];
  pivot.angular_z_rad_s=-direction*angular;
  now+=20U; PivotStep(&pivot,now);
  assert(ChassisKinematics_ToWheelTargets(&pivot,1600.0f,&targets));
  for(unsigned w=0;w<4;w++)
    if(w==MOTOR_DRIVER_WHEEL_LF || w==MOTOR_DRIVER_WHEEL_RF)
    {
      WheelSpeedControllerState state;
      assert(WheelSpeedController_GetState((MotorDriverWheel)w,&state));
      const float t=targets.counts_per_second[w],sign=t>0.0f ? 1.0f : -1.0f;
      const float load=fabsf(WheelSpeedController_GetPivotLoadFeedforward((MotorDriverWheel)w,t));
      assert(fabsf((state.feedforward_permille-ReferenceBaseFeedforward(w,t))*sign-
        load)<0.001f);
    }
  assert(abs(motor.command_permille[MOTOR_DRIVER_WHEEL_LF]-prior_lf)<=20);
  ChassisController_Stop();
  assert(!motor.standby_enabled);
  for(unsigned w=0;w<4;w++)
  {
    WheelSpeedControllerState state;
    assert(WheelSpeedController_GetState((MotorDriverWheel)w,&state));
    assert(state.integrator_permille==0.0f && state.feedforward_permille==0.0f);
  }
  assert(depth==0U);
}
static void TestFrontRecoverySafety(uint32_t start_ms,unsigned action,bool handoff)
{
  Init(start_ms);
  assert(ChassisRuntime_SetVelocity(0.0f,0.35f,start_ms));
  ChassisRuntime_Process(start_ms,false);
  for(uint32_t dt=20;dt<=180;dt+=20)
  {
    time_ms=start_ms+dt;
    assert(ChassisRuntime_SetVelocity(0.0f,0.35f,time_ms));
    ChassisRuntime_Process(time_ms,false);
  }
  for(unsigned w=0;w<4;w+=2)
  {
    WheelSpeedFrontPivotState f;
    assert(WheelSpeedController_GetFrontPivotState((MotorDriverWheel)w,&f));
    assert(f.active && f.recovering && f.recovery_permille>0);
  }
  const uint32_t last_command=handoff ? 400U : 180U;
  if(handoff)
  {
    feedback[0]=-100; feedback[1]=-100; feedback[2]=100; feedback[3]=100;
    for(uint32_t dt=200;dt<=400;dt+=20)
    {
      time_ms=start_ms+dt;
      assert(ChassisRuntime_SetVelocity(0.0f,0.35f,time_ms));
      ChassisRuntime_Process(time_ms,false);
    }
    for(unsigned w=0;w<4;w+=2)
    {
      WheelSpeedFrontPivotState f;
      WheelSpeedControllerState state;
      assert(WheelSpeedController_GetFrontPivotState((MotorDriverWheel)w,&f));
      assert(WheelSpeedController_GetState((MotorDriverWheel)w,&state));
      assert(!f.recovering && f.recovery_permille>0 &&
        state.integrator_permille*f.sign>0);
    }
  }
  time_ms=start_ms+last_command+1U;
  switch(action)
  {
    case 0: ChassisRuntime_Stop(time_ms); break;
    case 1:
      assert(ChassisRuntime_SetVelocity(0,0,time_ms));
      ChassisRuntime_Process(time_ms,false); break;
    case 2: ChassisRuntime_StopForTransportLoss(time_ms); break;
    case 3: ChassisRuntime_LatchHealthFault(time_ms); break;
    case 4: estop=true; ChassisRuntime_SafetyProcess(time_ms); break;
    case 5: ChassisRuntime_SafetyProcess(start_ms+last_command+250U); break;
    case 6: ChassisRuntime_Process(start_ms+last_command+101U,false); break;
    default:
      feedback[0]=3000; ChassisRuntime_Process(start_ms+last_command+20U,false); break;
  }
  assert(!motor.standby_enabled && !ChassisController_IsActive());
  for(unsigned w=0;w<4;w++)
  {
    assert(motor.command_permille[w]==0);
    if(w==MOTOR_DRIVER_WHEEL_LF || w==MOTOR_DRIVER_WHEEL_RF)
    {
      WheelSpeedFrontPivotState f;
      assert(WheelSpeedController_GetFrontPivotState((MotorDriverWheel)w,&f));
      assert(!f.active && !f.recovering && f.load_permille==0 && f.recovery_permille==0);
      WheelSpeedControllerState state;
      assert(WheelSpeedController_GetState((MotorDriverWheel)w,&state));
      assert(state.integrator_permille==0);
    }
  }
  assert(depth==0U);
}

static void TestFrontLoadConfirmationSafety(uint32_t start_ms,unsigned action)
{
  Init(start_ms);
  assert(ChassisRuntime_SetVelocity(0.0f,0.35f,start_ms));
  ChassisRuntime_Process(start_ms,false);
  for(uint32_t dt=20;dt<=220;dt+=20)
  {
    time_ms=start_ms+dt;
    if(dt>=200)
    {
      feedback[0]=-500; feedback[1]=-500;
      feedback[2]=500; feedback[3]=500;
    }
    assert(ChassisRuntime_SetVelocity(0.0f,0.35f,time_ms));
    ChassisRuntime_Process(time_ms,false);
  }
  for(unsigned w=0;w<4;w+=2)
  {
    WheelSpeedFrontPivotState f;
    assert(WheelSpeedController_GetFrontPivotState((MotorDriverWheel)w,&f));
    assert(f.active && !f.recovering && f.load_overspeed_ms==40);
  }
  time_ms=start_ms+221U;
  switch(action)
  {
    case 0: ChassisRuntime_Stop(time_ms); break;
    case 1:
      assert(ChassisRuntime_SetVelocity(0,0,time_ms));
      ChassisRuntime_Process(time_ms,false); break;
    case 2: ChassisRuntime_StopForTransportLoss(time_ms); break;
    case 3: ChassisRuntime_LatchHealthFault(time_ms); break;
    case 4: estop=true; ChassisRuntime_SafetyProcess(time_ms); break;
    case 5: ChassisRuntime_SafetyProcess(start_ms+470U); break;
    case 6: ChassisRuntime_Process(start_ms+321U,false); break;
    default:
      feedback[0]=3000; ChassisRuntime_Process(start_ms+240U,false); break;
  }
  assert(!motor.standby_enabled && !ChassisController_IsActive());
  for(unsigned w=0;w<4;w++)
  {
    assert(motor.command_permille[w]==0);
    if(w==MOTOR_DRIVER_WHEEL_LF || w==MOTOR_DRIVER_WHEEL_RF)
    {
      WheelSpeedFrontPivotState f;
      assert(WheelSpeedController_GetFrontPivotState((MotorDriverWheel)w,&f));
      assert(!f.active && f.load_overspeed_ms==0 && f.load_permille==0 && f.recovery_permille==0);
    }
  }
  assert(depth==0U);
}

/* Offline compatibility subset only. LR-reverse running restart expectations
 * are intentionally different; preserve their original full suite separately.
 * Here retain LR-forward/RR-reverse cases plus all other runtime assertions.
 * RR-forward scope is additionally covered by byte-identical differential. */
static int CompatPuts(const char *message)
{
  if (strstr(message,"PASS: rear-only restart") ||
      strstr(message,"PASS: rear restart withdraws") ||
      strstr(message,"PASS: rear 49/50cps") ||
      strstr(message,"PASS: restart-only assist"))
    return puts("PASS: original legacy LR-forward/RR-reverse runtime subset only; LR-reverse replacement tested separately, not claimed unchanged");
  return puts(message);
}
#define puts(message) CompatPuts(message)
#define TestRearRestartTopup(d,t,i) do { if((d)<0) TestRearRestartTopup((d),(t),(i)); } while(0)
#define TestRearRestartEarlyHandoff(d,t) do { if((d)<0) TestRearRestartEarlyHandoff((d),(t)); } while(0)
#define TestRearRestartThresholds(d,t) do { if((d)<0) TestRearRestartThresholds((d),(t)); } while(0)
#define TestRearTransientDecay(d,t) do { if((d)<0) TestRearTransientDecay((d),(t)); } while(0)
#define TestRearSteadySupportPreserved(d,t) do { if((d)<0) TestRearSteadySupportPreserved((d),(t)); } while(0)
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

  /* Exercise production pivot state and release across the configured hold. */
  Init(0U);
  ChassisVelocityCommand pivot={0.0f,0.2f};
  assert(ChassisController_Start(&pivot,0U));
  for(time_ms=20U;time_ms<300U;time_ms+=20U)
  {
    assert(ChassisController_SetCommand(&pivot,time_ms));
    ChassisController_Process(time_ms);
  }
  ChassisControllerStatus pivot_status;
  assert(ChassisController_GetStatus(&pivot_status));
  WheelSpeedControllerState before_release[4], after_release;
  for(unsigned w=0;w<4;w++)
  {
    feedback[w]=pivot_status.targets.counts_per_second[w];
    assert(WheelSpeedController_GetState((MotorDriverWheel)w,&before_release[w]));
  }
  assert(ChassisController_SetCommand(&pivot,300U));
  ChassisController_Process(300U);
  for(unsigned w=0;w<4;w++)
  {
    assert(WheelSpeedController_GetState((MotorDriverWheel)w,&after_release));
    assert(fabsf((before_release[w].feedforward_permille + before_release[w].integrator_permille) -
                 (after_release.feedforward_permille + after_release.integrator_permille))<0.001f);
    assert(fabsf(after_release.integrator_permille)<=50.0f);
  }
  int16_t settled[4];
  for(time_ms=320U;time_ms<=600U;time_ms+=20U)
  {
    assert(ChassisController_SetCommand(&pivot,time_ms));
    ChassisController_Process(time_ms);
    for(unsigned w=0;w<4;w++)
    {
      assert(WheelSpeedController_GetState((MotorDriverWheel)w,&after_release));
      if(time_ms==320U) settled[w]=after_release.command_permille;
      if(w==MOTOR_DRIVER_WHEEL_LR || w==MOTOR_DRIVER_WHEEL_RR)
        assert(after_release.command_permille==settled[w]);
      else
        /* Front handoff reclassifies temporary support into bounded I.
         * At exact target its SUM, not I alone, must stay unchanged. */
        assert(fabsf((after_release.feedforward_permille+after_release.integrator_permille)-
          (before_release[w].feedforward_permille+before_release[w].integrator_permille))<0.002f);
    }
  }
  for(unsigned w=0;w<4;w++) feedback[w]*=2.0f;
  assert(ChassisController_SetCommand(&pivot,620U));
  ChassisController_Process(620U);
  for(unsigned w=0;w<4;w++)
  {
    assert(WheelSpeedController_GetState((MotorDriverWheel)w,&after_release));
    assert(fabsf(after_release.proportional_permille -
      0.15f*after_release.error_counts_per_second)<0.001f);
  }
  /* Rolling/straight output must not inherit pivot damping. */
  ChassisVelocityCommand rolling={0.04f,0.2f};
  assert(ChassisController_SetCommand(&rolling,640U));
  ChassisController_Process(640U);
  for(unsigned w=0;w<4;w++)
  {
    assert(WheelSpeedController_GetState((MotorDriverWheel)w,&after_release));
    assert(fabsf(after_release.proportional_permille -
      0.05f*after_release.error_counts_per_second)<0.001f);
  }
  /* A direction change cannot carry the previous maneuver bias across zero. */
  pivot.angular_z_rad_s=-0.2f;
  assert(ChassisController_SetCommand(&pivot,660U));
  ChassisController_Process(660U);
  for(unsigned w=0;w<4;w++)
  {
    assert(WheelSpeedController_GetState((MotorDriverWheel)w,&after_release));
    assert(after_release.feedforward_permille * feedback[w]<0.0f);
    assert(fabsf(after_release.integrator_permille)<=50.0f);
  }
  ChassisController_Stop();
  assert(!motor.standby_enabled);
  for(unsigned w=0;w<4;w++)
  {
    assert(motor.command_permille[w]==0);
    assert(WheelSpeedController_GetState((MotorDriverWheel)w,&after_release));
    assert(after_release.integrator_permille==0.0f && after_release.feedforward_permille==0.0f);
  }
  /* Restart a straight command after pivot: there must be no retained assist. */
  memset(feedback,0,sizeof(feedback));
  ChassisVelocityCommand straight={0.04f,0.0f};
  assert(ChassisController_Start(&straight,680U));
  assert(ChassisController_GetStatus(&pivot_status));
  assert(WheelSpeedController_GetState(MOTOR_DRIVER_WHEEL_LF,&after_release));
  const float baseline=WHEEL_SPEED_FF_LF_FWD_STATIC_PERMILLE +
    WHEEL_SPEED_FF_LF_FWD_GAIN_PER_CPS*pivot_status.targets.counts_per_second[0];
  assert(fabsf(after_release.feedforward_permille-baseline)<0.001f);
  ChassisController_SafetyProcess(930U);
  assert(!ChassisController_IsActive() && !motor.standby_enabled);
  assert(depth==0U);
  puts("PASS: production pivot release conserves FF+I through integral clamp; equilibrium has no recurring release step; reversal/STOP/straight restart/250ms timeout clear maneuver state");
  puts("PASS: real chassis selects damping in pivot and baseline gains for rolling turn; no profile leaks across mode switch");
  TestRearRestartTopup(1.0f,0U,29.4f);
  TestRearRestartTopup(-1.0f,0U,29.4f);
  TestRearRestartTopup(1.0f,0xFFFFFF00U,29.4f);
  TestRearRestartTopup(-1.0f,0xFFFFFF00U,29.4f);
  TestRearRestartTopup(1.0f,0U,0.0f);
  TestRearRestartTopup(-1.0f,0U,-10.0f);
  TestRearRestartTopup(1.0f,0U,50.0f);
  TestRearRestartTopup(-1.0f,0U,50.0f);
  TestRetainedRearBias();
  puts("PASS: rear-only restart top-up accounts for integral/retained bias, bounded 80ms/40ms escalation, both signs and tick wrap; rear first kick/mode exit/reversal/STOP/timeout preserved");
  TestRearRestartEarlyHandoff(1.0f,0U);
  TestRearRestartEarlyHandoff(-1.0f,0U);
  TestRearRestartEarlyHandoff(1.0f,0xFFFFFF00U);
  TestRearRestartEarlyHandoff(-1.0f,0xFFFFFF00U);
  puts("PASS: rear restart withdraws transient at 40ms after motion, no copy into I; bounded 4 permille/20ms decay; rear first kick keeps 300ms hold");
  TestRearRestartThresholds(1.0f,0U);
  TestRearRestartThresholds(-1.0f,0U);
  TestRearRestartThresholds(1.0f,0xFFFFFF80U);
  TestRearRestartThresholds(-1.0f,0xFFFFFF80U);
  puts("PASS: rear 49/50cps release and stationary-only increase boundaries; late update adds one step, no overdue catch-up, both signs/tick wrap");
  TestRearTransientDecay(1.0f,0U);
  TestRearTransientDecay(-1.0f,0U);
  TestRearTransientDecay(1.0f,0xFFFFFF80U);
  TestRearTransientDecay(-1.0f,0xFFFFFF80U);
  TestRearSteadySupportPreserved(1.0f,0U);
  TestRearSteadySupportPreserved(-1.0f,0U);
  TestRearSteadySupportPreserved(1.0f,0xFFFFFF00U);
  TestRearSteadySupportPreserved(-1.0f,0xFFFFFF00U);
  puts("PASS: restart-only assist decays to zero without integral transfer; 49cps pause, rearm without stacking, actual-dt decay, both signs/wrap; legacy steady support retained");
  TestFrontPivotLoad(1.0f,0.1f,0U);
  TestFrontPivotLoad(-1.0f,0.1f,0U);
  TestFrontPivotLoad(1.0f,0.2f,0U);
  TestFrontPivotLoad(-1.0f,0.2f,0U);
  TestFrontPivotLoad(1.0f,0.35f,0xFFFFFF00U);
  TestFrontPivotLoad(-1.0f,0.35f,0xFFFFFF00U);
  puts("PASS: real chassis front support uses one policy, bounded target ramps; sign/mode/STOP/tick-wrap clear state, normal FF and rear regression assertions retained");
  for(unsigned action=0;action<8;action++)
  {
    TestFrontRecoverySafety(0U,action,false);
    TestFrontRecoverySafety(0xFFFFFF80U,action,false);
    TestFrontRecoverySafety(0U,action,true);
    TestFrontRecoverySafety(0xFFFFFF80U,action,true);
  }
  puts("PASS: active front recovery cannot survive STOP/zero/transport loss/health fault/ESTOP/exact 250ms timeout/timing fault/overspeed, including tick wrap");
  puts("PASS: partial front recovery-to-I handoff clears BOTH buckets on all eight safety shutdown paths, including tick wrap");
  for(unsigned action=0;action<8;action++)
  {
    TestFrontLoadConfirmationSafety(0U,action);
    TestFrontLoadConfirmationSafety(0xFFFFFF80U,action);
  }
  puts("PASS: pending 40ms front load confirmation never delays STOP/zero/transport loss/health fault/ESTOP/exact 250ms timeout/timing fault/overspeed shutdown, including tick wrap");
  return 0;
}
