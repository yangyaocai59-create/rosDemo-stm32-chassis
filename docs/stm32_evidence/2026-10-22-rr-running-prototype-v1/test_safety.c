#include "fixture.h"

static void safety(unsigned action,uint32_t base)
{
  Init(base);
  for(unsigned t=0;t<=600;t+=20)
  {
    time_ms=base+t;
    if(t>=100)
    {
      const float speed=t<400 ? 500.0f : 240.0f;
      feedback[0]=-speed; feedback[1]=-speed; feedback[2]=speed; feedback[3]=speed;
    }
    assert(ChassisRuntime_SetVelocity(0,.3f,time_ms));
    ChassisRuntime_Process(time_ms,false);
  }
  assert(s_rr_running.enabled);
  feedback[3]=100;
  for(unsigned t=620;t<=760;t+=20)
  {
    time_ms=base+t;
    assert(ChassisRuntime_SetVelocity(0,.3f,time_ms));
    ChassisRuntime_Process(time_ms,false);
  }
  assert(s_rr_running.temporary_permille>0);
  time_ms=base+761;
  switch(action)
  {
    case 0: ChassisRuntime_Stop(time_ms); break;
    case 1: assert(ChassisRuntime_SetVelocity(0,0,time_ms)); ChassisRuntime_Process(time_ms,false); break;
    case 2: ChassisRuntime_StopForTransportLoss(time_ms); break;
    case 3: ChassisRuntime_LatchHealthFault(time_ms); break;
    case 4: estop=true; ChassisRuntime_SafetyProcess(time_ms); break;
    case 5: ChassisRuntime_SafetyProcess(base+1010); break;
    case 6: ChassisRuntime_Process(base+861,false); break;
    default: feedback[3]=3000; ChassisRuntime_Process(base+780,false); break;
  }
  assert(!motor.standby_enabled && !ChassisController_IsActive());
  for(unsigned w=0;w<4;w++) assert(motor.command_permille[w]==0);
  assert(!s_rr_running.enabled && !s_rr_running.active && s_rr_running.temporary_permille==0);
  assert(s_states[3].integrator_permille==0 && depth==0);
}

int main(void)
{
  for(unsigned a=0;a<8;a++) { safety(a,0); safety(a,0xFFFFFF00U); }
  puts("PASS: real runtime STOP/zero/transport/health/ESTOP/exact250ms/timing/overspeed immediately disable four outputs and clear RR support, including tick wrap");
  return 0;
}
