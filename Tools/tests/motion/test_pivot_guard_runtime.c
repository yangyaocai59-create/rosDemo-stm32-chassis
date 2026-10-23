#define main UnusedLegacyMain
#include "test_front_recovery_safety.c"
#undef main
#include "pivot_feedback_guard.h"
int main(void)
{
  for(unsigned wheel=0;wheel<4;wheel++) for(int sign=-1;sign<=1;sign+=2)
  {
    Init(0);
    for(unsigned t=0;t<=6000;t+=20)
    {
      Heartbeat(t); FeedbackExcept(t>=1500 && t<2700?(int)wheel:-1);
      assert(ChassisRuntime_SetVelocity(0,sign*.3f,t));
      ChassisRuntime_Process(t,false);Safety(t);
      assert(Runtime().state==CHASSIS_RUNTIME_STATE_ACTIVE && !FaultManager_HasCriticalFault());
    }
    ChassisRuntime_Stop(6001);AssertDisabled();
  }
  puts("PASS: real runtime/FM tolerates1200ms recoverable single-wheel underspeed, all wheels/signs, no global latch");
  for(unsigned w=0;w<4;w++)
  {
    Init(0); unsigned trip=0;
    for(unsigned t=0;t<=3500;t+=20)
    {
      Heartbeat(t);FeedbackExcept((int)w);
      assert(ChassisRuntime_SetVelocity(0,.3f,t));ChassisRuntime_Process(t,false);Safety(t);
      if(Runtime().state==CHASSIS_RUNTIME_STATE_FAULT) {trip=t;break;}
    }
    assert(trip>=3000 && trip<=3100);AssertDisabled();
    assert(!ChassisRuntime_SetVelocity(0,.3f,trip+1));
  }
  puts("PASS: old750ms and1.5s paths do not bypass new pivot guard; persistent zero feedback stops all four finitely");
  /* Ordinary-mode >=200cps protection remains the original1000+750ms policy. */
  TestLegacyGuard();
  puts("PASS: legacy/NORMAL fault predicate, grace, qualification unchanged");
}
