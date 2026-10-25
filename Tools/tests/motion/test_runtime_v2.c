#define main HistoricalFixtureMain
#include "unified_runtime_fixture.h"
#undef main

static void Start(uint32_t now,float linear,float angular)
{ Init(now); assert(ChassisRuntime_SetVelocity(linear,angular,now)); ChassisRuntime_Process(now,false); assert(ChassisRuntime_IsActive()); }
static void StopHook(void) { ChassisRuntime_Stop(fixture_feedback_time); }
static void CommandHook(void) {
  const ChassisVelocityCommand command={0.02f,0};
  assert(ChassisController_SetCommand(&command,fixture_feedback_time));
}
static void RuntimeCommandHook(void)
{ assert(ChassisRuntime_SetVelocity(.02f,.1f,fixture_feedback_time)); }
int main(void)
{
#if CHASSIS_CONTROL_REVISION >= 201
  for(unsigned wrap=0;wrap<2;wrap++) {
    uint32_t now=wrap?0xfffffff0U:500U;
    ChassisVelocityCommand c={0,.3f};
    Init(now);fixture_exit_hook=StopHook;
    assert(!ChassisController_Start(&c,now));AssertDisabled();
    for(unsigned w=0;w<4;w++) {
      WheelSpeedControllerState state;
      assert(WheelSpeedController_GetState((MotorDriverWheel)w,&state));
      assert(state.integrator_permille==0&&state.command_permille==0);
    }
    Init(now);motor.standby_enabled=true;
    motor.command_permille[0]=motor.command_permille[1]=-80;
    motor.command_permille[2]=motor.command_permille[3]=80;
    fixture_exit_hook=StopHook;
    assert(!ChassisController_StartFromActiveOutputs(&c,now));AssertDisabled();
    Init(now);
    assert(ChassisController_Start(&c,now));ChassisController_Stop();AssertDisabled();
    Init(now);fixture_snapshot_hook=StopHook;
    assert(!ChassisController_Start(&c,now));AssertDisabled();
    ChassisControllerStatus stopped_start;
    assert(ChassisController_GetStatus(&stopped_start)&&stopped_start.fault==CHASSIS_CONTROLLER_FAULT_NONE);
    Init(now);fixture_snapshot_hook=StopHook;fixture_feedback_invalid=true;
    assert(!ChassisController_Start(&c,now));AssertDisabled();
    assert(ChassisController_GetStatus(&stopped_start)&&stopped_start.fault==CHASSIS_CONTROLLER_FAULT_NONE);
    Init(now);motor.standby_enabled=true;
    motor.command_permille[0]=motor.command_permille[1]=-80;
    motor.command_permille[2]=motor.command_permille[3]=80;
    assert(ChassisController_StartFromActiveOutputs(&c,now));
    for(unsigned w=0;w<4;w++) assert(abs(motor.command_permille[w])<=100);
    ChassisController_Stop();AssertDisabled();
  }
  puts("PASS: Stop invalidates cold/adopted startup before enable; discarded start leaves four PI states reset, including wrap");
#endif
  for (unsigned wrap=0;wrap<2;wrap++) {
    const uint32_t base=wrap?0xfffffff0U:1000U;
    Start(base,0,.3f);
    Heartbeat(base+20); FeedbackExcept(-1);
    fixture_exit_hook=StopHook;
    ChassisController_Process(base+20);
    AssertDisabled();
    assert(Runtime().state==CHASSIS_RUNTIME_STATE_IDLE);

    Start(base,0,.3f);
    Heartbeat(base+20); FeedbackExcept(-1);
    ChassisControllerStatus before;
    assert(ChassisController_GetStatus(&before));
    fixture_exit_hook=CommandHook;
    ChassisController_Process(base+20);
    ChassisControllerStatus after;
    assert(ChassisController_GetStatus(&after));
    assert(after.active && after.last_control_ms==before.last_control_ms);
    for(unsigned w=0;w<4;w++) assert(after.output_permille[w]==before.output_permille[w]);
    Heartbeat(base+40); ChassisController_Process(base+40);
    assert(ChassisController_GetStatus(&after)&&after.active&&after.last_control_ms==base+40);
    ChassisRuntime_Stop(base+41); AssertDisabled();

    Start(base,0,.3f);
#if CHASSIS_CONTROL_REVISION >= 202 || defined(TEST_STALE_SAFETY_CLOCK)
    /* Safety captured its time, then motor committed a newer valid snapshot.
     * Test normal and uint32 wrap without weakening the 40/41ms deadline. */
    ChassisController_SafetyProcess(base-1U);
    assert(ChassisController_IsActive());
    ChassisController_SafetyProcess(base-20U);
    assert(ChassisController_IsActive());
#endif
    ChassisController_SafetyProcess(base+40);
    assert(ChassisController_IsActive());
    ChassisController_SafetyProcess(base+41);
    AssertDisabled();
    assert(ChassisController_GetStatus(&after)&&after.fault==CHASSIS_CONTROLLER_FAULT_SPEED_SNAPSHOT);

    Start(base,0,.3f);
    fixture_feedback_frozen=true;
    Heartbeat(base+20); ChassisController_Process(base+20);
    AssertDisabled();
    assert(ChassisController_GetStatus(&after)&&after.fault==CHASSIS_CONTROLLER_FAULT_SPEED_SNAPSHOT);
    Start(base,0,.3f);
    Heartbeat(base+20); fixture_feedback_invalid=true;
    ChassisController_Process(base+20); AssertDisabled();
    Start(base,0,.3f);
    Heartbeat(base+20);feedback[2]=NAN;
    ChassisController_Process(base+20);AssertDisabled();
    Start(base,0,.3f);
    Heartbeat(base+20);fixture_feedback_sequence--;
    ChassisController_Process(base+20);AssertDisabled();
    Start(base,0,.3f);
    Heartbeat(base+20);fixture_feedback_time=base+21;
    ChassisController_Process(base+20);AssertDisabled();
  }
  puts("PASS: Stop and replacement command interrupt private computation; stale/repeated/invalid feedback cuts all outputs; 40/41ms and tick wrap");
  Start(8000,0,.3f);Heartbeat(8020);FeedbackExcept(-1);
  fixture_exit_hook=RuntimeCommandHook;
  ChassisRuntime_Process(8020,false);
  assert(Runtime().state==CHASSIS_RUNTIME_STATE_ACTIVE&&Runtime().fault==CHASSIS_RUNTIME_FAULT_NONE);
  Heartbeat(8040);ChassisRuntime_Process(8040,false);
  assert(Runtime().state==CHASSIS_RUNTIME_STATE_ACTIVE);
  ChassisRunMetrics metrics;
  assert(ChassisController_GetRunMetrics(&metrics)&&metrics.samples>=2);
  for(unsigned w=0;w<4;w++) assert(metrics.wheel[w].samples==metrics.samples);
  ChassisRuntime_Stop(8041);AssertDisabled();
  ChassisRunMetrics stopped;
  assert(ChassisController_GetRunMetrics(&stopped)&&stopped.samples==metrics.samples);
  puts("PASS: a new command supersedes runtime shaping without false fault; run metrics cover all wheels and survive Stop");
  Start(10000,0,.3f);
  for (unsigned k=1;k<=40;k++) {
    uint32_t now=10000+k*20; Heartbeat(now); FeedbackExcept(-1);
    ChassisVelocityCommand c={k%2?.0009f:.0011f,.3f};
    assert(ChassisController_SetCommand(&c,now)); ChassisController_Process(now);
    ChassisControllerStatus s; assert(ChassisController_GetStatus(&s));
    assert(s.active && s.pivot_mask==15U);
  }
  Heartbeat(10820);
  ChassisVelocityCommand arc={.004f,.3f};
  assert(ChassisController_SetCommand(&arc,10820));ChassisController_Process(10820);
  ChassisControllerStatus s; assert(ChassisController_GetStatus(&s)&&s.pivot_mask==0U);
  ChassisRuntime_Stop(10821);AssertDisabled();
  puts("PASS: pivot hysteresis survives 0.9/1.1mm/s jitter and exits at 4mm/s");
  Init(20000);
  unsigned trip=0;
  for (unsigned ms=0;ms<=3500;ms+=20) {
    uint32_t now=20000+ms; Heartbeat(now); FeedbackExcept(MOTOR_DRIVER_WHEEL_LF);
    assert(ChassisRuntime_SetVelocity(.01f,0,now));ChassisRuntime_Process(now,false);Safety(now);
    if (Runtime().state==CHASSIS_RUNTIME_STATE_FAULT) {trip=ms;break;}
  }
  assert(trip>=3000&&trip<=3100);AssertDisabled();
  puts("PASS: low-speed NORMAL no-feedback condition now latches within the existing 1s+2s feedback-guard envelope");
  for(int sign=-1;sign<=1;sign+=2) {
    Init(30000); ChassisVelocityCommand c={.15f,sign*1.5f};
    assert(ChassisController_Start(&c,30000));
    assert(ChassisController_GetStatus(&s)&&s.active&&s.targets.scale<1.0f);
    float expected_left=c.linear_x_m_s-c.angular_z_rad_s*.395f/2;
    float expected_right=c.linear_x_m_s+c.angular_z_rad_s*.395f/2;
    assert(fabsf(s.targets.left_linear_m_s/expected_left-
                 s.targets.right_linear_m_s/expected_right)<.0001f);
    for(unsigned w=0;w<4;w++) {
      float bound=WheelSpeedController_GetFeedforwardTargetLimit((MotorDriverWheel)w,
        s.targets.counts_per_second[w]>0,WHEEL_SPEED_CONTROL_MODE_NORMAL);
      assert(fabsf(s.targets.counts_per_second[w])<=bound+.001f);
    }
    ChassisController_Stop();AssertDisabled();
  }
  puts("PASS: static FF envelope scales all targets together, preserving mixed-motion curvature in both directions");
  Init(40000);
  float targets[4]={1,0,0,0},speeds[4]={0};
  int16_t outputs[4]={50,0,0,0},command;
  for(unsigned ms=0;ms<=3000;ms+=20) {
    Heartbeat(40000+ms);
    assert(WheelSpeedController_UpdateWithMode(MOTOR_DRIVER_WHEEL_LF,1,0,0,
      (ms/20)%2?WHEEL_SPEED_CONTROL_MODE_NORMAL:WHEEL_SPEED_CONTROL_MODE_PIVOT,20,&command));
    FaultManager_ObserveMotion(40000+ms,true,targets,speeds,outputs);
    FaultManager_Process(40000+ms);
    assert(FaultManager_HasCriticalFault()==(ms==3000));
  }
  FaultManagerStatus health;assert(FaultManager_GetStatus(&health));
  assert(health.latched_stalled_wheel_mask==1U);
  puts("PASS: rapid NORMAL/PIVOT transitions cannot refresh the 1cps no-feedback grace/deadline");
  return 0;
}
