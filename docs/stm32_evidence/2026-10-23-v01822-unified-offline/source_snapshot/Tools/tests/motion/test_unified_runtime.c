/* Reuse peripheral stubs only; exercise actual candidate runtime/controller/FM. */
#define main FrozenFrontSafetyMain
#define Test_ExitCritical FrozenTest_ExitCritical
#include "test_front_recovery_safety.c"
#undef Test_ExitCritical
#undef main

static unsigned exit_index, inject_at;
static bool inject_armed, injected;
static uint32_t inject_time;
void Test_ExitCritical(void)
{
  FrozenTest_ExitCritical();
  if(depth==0 && inject_armed && ++exit_index==inject_at)
  {
    inject_armed=false; injected=true;
    ChassisRuntime_Stop(inject_time);
  }
}

static void ResetAll(uint32_t base)
{ inject_armed=false; Init(base); }

static void TestAllWheelFaults(void)
{
  for(unsigned w=0;w<4;w++) for(int sign=-1;sign<=1;sign+=2)
    for(unsigned wrap=0;wrap<2;wrap++)
    {
      const uint32_t base=wrap?0xfffffc00U:5000U;
      ResetAll(base);
      unsigned trip=0;
      for(unsigned t=0;t<=2200;t+=20)
      {
        const uint32_t now=base+t;
        Heartbeat(now); FeedbackExcept((int)w);
        assert(ChassisRuntime_SetVelocity(0,sign*.3f,now));
        ChassisRuntime_Process(now,false); Safety(now);
        if(Runtime().state==CHASSIS_RUNTIME_STATE_FAULT) {trip=t;break;}
      }
      if(trip<1500 || trip>1600)
        fprintf(stderr,"wheel=%u sign=%d wrap=%u trip=%u state=%d fault=%d\n",w,sign,wrap,trip,Runtime().state,Runtime().fault);
      assert(trip>=1500 && trip<=1600);
      FaultManagerStatus f; assert(FaultManager_GetStatus(&f));
      assert(f.latched_stalled_wheel_mask==(1U<<w));
      AssertDisabled();
      assert(!ChassisRuntime_SetVelocity(0,sign*.3f,base+trip+20));
      Heartbeat(base+trip+40); Safety(base+trip+40);
      assert(ChassisRuntime_ClearFault(base+trip+40));
      AssertDisabled();
      assert(Runtime().state==CHASSIS_RUNTIME_STATE_IDLE);
    }
  puts("PASS: actual runtime/FM all four wheels/both directions/wrap trip finitely; fault latches, rejects and explicit clear does not resume");
}

static void TestStopPaths(void)
{
  for(unsigned action=0;action<8;action++) for(unsigned wrap=0;wrap<2;wrap++)
  {
    const uint32_t base=wrap?0xfffffc00U:10000;
    ResetAll(base);
    for(unsigned t=0;t<=600;t+=20)
    {
      Heartbeat(base+t);
      for(unsigned w=0;w<4;w++) feedback[w]=w<2?-80:80;
      assert(ChassisRuntime_SetVelocity(0,.3f,base+t));
      ChassisRuntime_Process(base+t,false); Safety(base+t);
      assert(Runtime().state==CHASSIS_RUNTIME_STATE_ACTIVE);
    }
    uint32_t now=base+601;
    switch(action)
    {
      case 0: ChassisRuntime_Stop(now); break;
      case 1: assert(ChassisRuntime_SetVelocity(0,0,now));ChassisRuntime_Process(now,false);break;
      case 2: ChassisRuntime_StopForTransportLoss(now);break;
      case 3: ChassisRuntime_LatchHealthFault(now);break;
      case 4: estop=true;ChassisRuntime_SafetyProcess(now);ChassisController_SafetyProcess(now);break;
      case 5: now=base+850;ChassisRuntime_SafetyProcess(now);ChassisController_SafetyProcess(now);break;
      case 6: ChassisController_Process(base+701);ChassisRuntime_Process(base+701,false);break;
      case 7: for(unsigned w=0;w<4;w++)feedback[w]=3000;
              ChassisController_Process(base+620);ChassisRuntime_Process(base+620,false);break;
    }
    AssertDisabled();
    for(unsigned w=0;w<4;w++)
    {
      WheelSpeedControllerState s;assert(WheelSpeedController_GetState((MotorDriverWheel)w,&s));
      assert(s.integrator_permille==0 && s.command_permille==0);
      assert(!WheelSpeedController_IsUnifiedPivotOwned((MotorDriverWheel)w,w<2?-240:240));
    }
  }
  puts("PASS: STOP/zero/transport/health/ESTOP/exact250ms timeout/timing/overspeed clear all memory and outputs, including wrap");
}

static void TestStopInterleavings(void)
{
  unsigned covered=0;
  for(unsigned boundary=1;boundary<=20;boundary++)
  {
    ResetAll(20000);
    Heartbeat(20000);
    assert(ChassisRuntime_SetVelocity(0,.3f,20000));
    ChassisRuntime_Process(20000,false);
    for(unsigned t=20;t<=200;t+=20)
    {
      Heartbeat(20000+t);
      assert(ChassisRuntime_SetVelocity(0,.3f,20000+t));
      ChassisRuntime_Process(20000+t,false);
    }
    inject_time=20220; exit_index=0; inject_at=boundary; injected=false; inject_armed=true;
    ChassisController_Process(inject_time);
    inject_armed=false;
    if(injected)
    {
      covered++; AssertDisabled();
      for(unsigned w=0;w<4;w++)
      { WheelSpeedControllerState s;assert(WheelSpeedController_GetState((MotorDriverWheel)w,&s));assert(s.integrator_permille==0); }
    }
    else ChassisRuntime_Stop(20220);
  }
  assert(covered>=6);
  printf("PASS: task STOP at %u reachable controller critical-exit boundaries cannot resurrect per-wheel I or outputs\n",covered);
}

static void TestPersistentTargets(void)
{
  for(int sign=-1;sign<=1;sign+=2) for(unsigned wrap=0;wrap<2;wrap++)
  {
    uint32_t base=wrap?0xfffffc00U:30000U;
    float fixed_ff[4]={0};
    ResetAll(base);
    for(unsigned t=0;t<=20000;t+=20)
    {
      Heartbeat(base+t); FeedbackExcept(-1);
      for(unsigned w=0;w<4;w++) feedback[w]*=.8f;
      assert(ChassisRuntime_SetVelocity(0,sign*.3f,base+t));
      ChassisRuntime_Process(base+t,false); Safety(base+t);
      assert(Runtime().state==CHASSIS_RUNTIME_STATE_ACTIVE);
      assert(!FaultManager_HasCriticalFault());
      if(t>=500)
        for(unsigned w=0;w<4;w++)
        {
          WheelSpeedControllerState s;
          assert(WheelSpeedController_GetState((MotorDriverWheel)w,&s));
          if(t==500) fixed_ff[w]=s.feedforward_permille;
          assert(fabsf(fixed_ff[w]-s.feedforward_permille)<.001f);
          assert(abs(s.command_permille)>0 && motor.command_permille[w]==s.command_permille);
        }
    }
    ChassisRuntime_Stop(base+20001); AssertDisabled();
  }
  puts("PASS: 20s persistent pivot targets/both directions/wrap keep four independent FF+PI outputs active with no pulse/re-enable gaps");
}

static void TestStartStopInterleavings(void)
{
  unsigned covered=0;
  for(unsigned boundary=1;boundary<=30;boundary++)
  {
    ResetAll(60000); Heartbeat(60000);
    assert(ChassisRuntime_SetVelocity(0,.3f,60000));
    inject_time=60020; exit_index=0; inject_at=boundary;
    injected=false; inject_armed=true;
    ChassisRuntime_Process(60020,false);
    inject_armed=false;
    if(injected)
    {
      covered++; AssertDisabled();
      assert(Runtime().state==CHASSIS_RUNTIME_STATE_IDLE);
      for(unsigned w=0;w<4;w++)
      { WheelSpeedControllerState s; assert(WheelSpeedController_GetState((MotorDriverWheel)w,&s)); assert(s.integrator_permille==0); }
    }
    else ChassisRuntime_Stop(60021);
  }
  /* Startup is one nested outer transaction: only snapshot and final commit
   * leave depth zero. There are no schedulable inner per-wheel boundaries. */
  assert(covered==2);
  printf("PASS: STOP at %u reachable first-start runtime boundaries cannot enable or restart stale output\n",covered);
}

int main(void)
{
  TestAllWheelFaults(); TestStopPaths(); TestStopInterleavings(); TestPersistentTargets(); TestStartStopInterleavings();
  return 0;
}
