/* Deterministic task-boundary interleavings, not an RTOS timing proof. */
#include "wheel_speed_control_config.h"
#include "wheel_speed_controller.h"
#define Test_EnterCritical Frozen_Test_EnterCritical
#define Test_ExitCritical Frozen_Test_ExitCritical
#define WheelSpeedEstimator_GetSnapshot FrozenEstimator_GetSnapshot
#define main FrozenRuntimeRegressionMain
#include "../../../docs/stm32_evidence/2026-10-22-v01821-offline/baseline_0.18.20/test_runtime_motion.c"
#undef main
#undef Test_EnterCritical
#undef Test_ExitCritical
#undef WheelSpeedEstimator_GetSnapshot

static unsigned boundary, stop_at, stop_kind, observations, checked_rr;
static bool armed, stop_on_snapshot, stop_pending, isr_estop;
static unsigned snapshot_calls;
static int32_t rr_delta=5;
static int16_t restarted_output[4];
static uint32_t restarted_control_time;
static void StopNow(void)
{
  armed=false; stop_pending=false; stop_on_snapshot=false;
  if(stop_kind==1) ChassisRuntime_LatchHealthFault(time_ms);
  else if(stop_kind==2) ChassisRuntime_StopForTransportLoss(time_ms);
  else ChassisRuntime_Stop(time_ms);
  if(stop_kind==3)
  {
    assert(ChassisRuntime_SetVelocity(0,-.3f,time_ms));
    ChassisRuntime_Process(time_ms,false);
    ChassisControllerStatus c;assert(ChassisController_GetStatus(&c));
    assert(c.active);
    restarted_control_time=c.last_control_ms;
    memcpy(restarted_output,motor.command_permille,sizeof(restarted_output));
  }
}
void Test_EnterCritical(void) { Frozen_Test_EnterCritical(); }
void Test_ExitCritical(void)
{
  assert(depth);
  const bool outer=depth==1;
  Frozen_Test_ExitCritical();
  if(armed && outer && (++boundary==stop_at || stop_pending)) StopNow();
}
bool WheelSpeedEstimator_GetSnapshot(WheelSpeedState states[ENCODER_WHEEL_COUNT])
{
  if(!FrozenEstimator_GetSnapshot(states)) return false;
  states[3].sample_time_ms=time_ms;
  states[3].sample_period_ms=20;
  states[3].delta_counts=rr_delta;
  ++snapshot_calls;
  if(armed && stop_on_snapshot) StopNow();
  return true;
}
static void CheckedObserve(int32_t delta,uint32_t period,uint32_t stamp,uint32_t now)
{
  assert(depth>0); ++observations;
  WheelSpeedController_ObserveRrRecoveryCounts(delta,period,stamp,now);
  if(armed && stop_at==99) stop_pending=true; /* task STOP waits for unlock */
  if(armed && isr_estop)
  {
    /* Priority-4 ISR is not masked by BASEPRI=5. Simulate its direct power
     * cutoff, not an illegal RTOS API call inside a high-priority ISR. */
    estop=true;MotorDriver_Disable();
  }
}
static bool CheckedUpdate(MotorDriverWheel w,float target,float speed,float bias,
  WheelSpeedControlMode mode,uint32_t dt,int16_t *pwm)
{
  if(w==MOTOR_DRIVER_WHEEL_RR)
  {
    assert(depth>0);++checked_rr;
    if(stop_pending) assert(motor.standby_enabled); /* not serviced mid-lock */
  }
  return WheelSpeedController_UpdateWithMode(w,target,speed,bias,mode,dt,pwm);
}
#define WheelSpeedController_ObserveRrRecoveryCounts CheckedObserve
#define WheelSpeedController_UpdateWithMode CheckedUpdate
#include "../../../Application/Chassis/chassis_controller.c"
#undef WheelSpeedController_ObserveRrRecoveryCounts
#undef WheelSpeedController_UpdateWithMode
#include "../../../Application/Chassis/wheel_speed_controller.c"

static void Warm(uint32_t base)
{
  armed=false;isr_estop=false;stop_on_snapshot=false;stop_pending=false;
  rr_delta=5;Init(base);
  for(unsigned t=0;t<=600;t+=20)
  {
    time_ms=base+t;
    const float v=t<100 ? 0 : t<400 ? 500 : 240;
    feedback[0]=feedback[1]=-v;feedback[2]=feedback[3]=v;
    assert(ChassisRuntime_SetVelocity(0,.3f,time_ms));ChassisRuntime_Process(time_ms,false);
  }
  feedback[3]=100;rr_delta=2;
  for(unsigned t=620;t<=760;t+=20)
  {
    time_ms=base+t;
    assert(ChassisRuntime_SetVelocity(0,.3f,time_ms));ChassisRuntime_Process(time_ms,false);
  }
  rr_delta=7;
  for(unsigned t=780;t<=820;t+=20)
  {
    time_ms=base+t;
    assert(ChassisRuntime_SetVelocity(0,.3f,time_ms));ChassisRuntime_Process(time_ms,false);
  }
  assert(s_rr_exit.exiting && s_rr_running.temporary_permille>0 && !depth);
}
static void Cleared(void)
{
  const WheelSpeedRrExitState zero={0};
  assert(memcmp(&zero,&s_rr_exit,sizeof(zero))==0);
  assert(!s_rr_running.enabled && s_states[3].integrator_permille==0 && !depth);
}
int main(void)
{
  for(unsigned wrap=0;wrap<2;wrap++) for(unsigned kind=0;kind<3;kind++)
    for(unsigned point=0;point<5;point++)
  {
    Warm(wrap ? 0xFFFFFC00U : 0);
    const unsigned before=observations;
    time_ms+=20;boundary=0;stop_kind=kind;
    stop_on_snapshot=point==0; stop_at=point==4 ? 99 : point;
    armed=true;
    ChassisController_Process(time_ms);
    assert(!armed && !motor.standby_enabled);
    for(unsigned w=0;w<4;w++) assert(!motor.command_permille[w]);
    Cleared();
    if(point<=1) assert(observations==before);
    else assert(observations==before+1);
  }
  puts("PASS: 30 STOP/health/transport interleavings at capture, snapshot, RR unlock, output commit and pending-inside-RR boundaries: no stale RR state/output resurrection, wrap");
  for(unsigned point=1;point<=2;point++)
  {
    Warm(0);time_ms+=20;boundary=0;stop_kind=3;stop_at=point;armed=true;
    ChassisController_Process(time_ms);
    assert(!armed && !depth);
    assert(memcmp(restarted_output,motor.command_permille,sizeof(restarted_output))==0);
    assert(s_status.last_control_ms==restarted_control_time);
    assert(!s_rr_running.enabled && s_rr_exit.confirm_ms==0);
    ChassisRuntime_Stop(time_ms);Cleared();
  }
  puts("PASS: STOP followed immediately by new start cannot accept prior-generation RR observation or output commit");
  Warm(0);time_ms+=20;boundary=0;stop_kind=0;stop_at=100;armed=true;isr_estop=true;
  ChassisController_Process(time_ms);
  assert(estop && !motor.standby_enabled);
  for(unsigned w=0;w<4;w++) assert(!motor.command_permille[w]);
  armed=false;ChassisRuntime_SafetyProcess(time_ms);Cleared();
  assert(observations && checked_rr && snapshot_calls);
  puts("PASS: simulated unmasked priority-4 ESTOP direct power cutoff during RR transaction is not undone by output commit");
  return 0;
}
