/* Actual candidate controller/runtime/FM, existing prescribed feedback fixture. */
#define main ReviewFrozenFrontMain
#include "test_front_recovery_safety.c"
#undef main

int main(void)
{
  for(unsigned w=0;w<4;w++) for(int sign=-1;sign<=1;sign+=2)
    for(unsigned mode=0;mode<4;mode++)
    {
      float angular=sign*(mode==0?.025f:mode==3?.3f:.2f);
      float speed=mode<2?0:mode==2?21:30;
      Init(10000);
      unsigned trip=0;
      float target=0;
      for(unsigned elapsed=0;elapsed<=6000;elapsed+=20)
      {
        unsigned now=10000+elapsed;
        Heartbeat(now); FeedbackExcept((int)w);
        feedback[w]=speed*sign*(w<2?-1:1);
        assert(ChassisRuntime_SetVelocity(0,angular,now));
        ChassisRuntime_Process(now,false);
        ChassisControllerStatus c;assert(ChassisController_GetStatus(&c));
        if(elapsed==1000) target=fabsf(c.targets.counts_per_second[w]);
        Safety(now);
        if(Runtime().state==CHASSIS_RUNTIME_STATE_FAULT){trip=elapsed;break;}
      }
      if(mode==2) assert(trip==0 && Runtime().state==CHASSIS_RUNTIME_STATE_ACTIVE);
      else if(mode==3) assert(trip>=1740 && trip<=1800);
      else assert(trip>=1500 && trip<=1600);
      printf("FAULT_BOUNDARY,%u,%d,%u,%.9g,%.9g,%u\n",w,sign,mode,target,speed,trip);
      ChassisRuntime_Stop(16001);AssertDisabled();
    }
  puts("PASS: real near-rest guard covers all wheels at20/160cps; 160target/21feedback gap and240target/30feedback severe guard verified in both directions");
  Init(20000);Heartbeat(20000);
  assert(ChassisRuntime_SetVelocity(0,.3f,20000));
  uint32_t sequence=Runtime().command_sequence;
  for(unsigned elapsed=0;elapsed<=240;elapsed+=20)
  {
    Heartbeat(20000+elapsed);FeedbackExcept(-1);
    ChassisRuntime_Process(20000+elapsed,false);Safety(20000+elapsed);
    assert(Runtime().state==CHASSIS_RUNTIME_STATE_ACTIVE);
    assert(Runtime().command_sequence==sequence);
    if(elapsed>=20) for(unsigned w=0;w<4;w++) assert(motor.command_permille[w]!=0);
  }
  Heartbeat(20250);Safety(20250);AssertDisabled();
  assert(Runtime().last_stop_reason==CHASSIS_RUNTIME_STOP_COMMAND_TIMEOUT);
  puts("PASS: one command continues periodic four-wheel control through240ms without a second packet; exact250ms timeout still disables outputs");
  return 0;
}
