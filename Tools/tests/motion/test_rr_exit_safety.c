#define LIVE_FAST_STUB
#include "rr_exit_fixture.h"
static void check(unsigned action,uint32_t base)
{
  fast_delta=5;fast_period=20;fast_age=0;
  Init(base);
  for(unsigned t=0;t<=600;t+=20)
  {
    time_ms=base+t;
    const float speed=t<100 ? 0 : t<400 ? 500 : 240;
    feedback[0]=feedback[1]=-speed;feedback[2]=feedback[3]=speed;
    assert(ChassisRuntime_SetVelocity(0,.3f,time_ms));
    ChassisRuntime_Process(time_ms,false);
  }
  assert(s_rr_running.enabled);
  feedback[3]=100;fast_delta=2;
  for(unsigned t=620;t<=760;t+=20)
  {
    time_ms=base+t;
    assert(ChassisRuntime_SetVelocity(0,.3f,time_ms));
    ChassisRuntime_Process(time_ms,false);
  }
  assert(s_rr_running.temporary_permille>0);
  fast_delta=7;
  for(unsigned t=780;t<=820;t+=20)
  {
    time_ms=base+t;
    assert(ChassisRuntime_SetVelocity(0,.3f,time_ms));
    ChassisRuntime_Process(time_ms,false);
  }
  assert(s_rr_exit.exiting && s_rr_running.temporary_permille>0);
  time_ms=base+821;
  switch(action)
  {
    case 0: ChassisRuntime_Stop(time_ms);break;
    case 1: assert(ChassisRuntime_SetVelocity(0,0,time_ms));ChassisRuntime_Process(time_ms,false);break;
    case 2: ChassisRuntime_StopForTransportLoss(time_ms);break;
    case 3: ChassisRuntime_LatchHealthFault(time_ms);break;
    case 4: estop=true;ChassisRuntime_SafetyProcess(time_ms);break;
    case 5: ChassisRuntime_SafetyProcess(base+1070);break;
    case 6: ChassisRuntime_Process(base+921,false);break;
    default: feedback[3]=3000;ChassisRuntime_Process(base+840,false);break;
  }
  assert(!motor.standby_enabled && !ChassisController_IsActive() && !depth);
  for(unsigned w=0;w<4;w++) assert(motor.command_permille[w]==0);
  const WheelSpeedRrExitState zero={0};
  assert(memcmp(&zero,&s_rr_exit,sizeof(zero))==0);
  assert(!s_rr_running.enabled && s_states[3].integrator_permille==0);
}
int main(void)
{
  for(unsigned a=0;a<8;a++){check(a,0);check(a,0xFFFFFF00U);}
  puts("PASS: actual chassis consumes timestamped RR snapshots; early-exit active/fading state clears on all 8 immediate runtime shutdowns, four outputs zero, tick wrap");
  return 0;
}
