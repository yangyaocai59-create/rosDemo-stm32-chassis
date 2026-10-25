#define main HistoricalFixtureMain
#include "unified_runtime_fixture.h"
#undef main
#include "chassis_yaw_feedback.h"

static void Calibrate(uint32_t base)
{
  for(unsigned k=0;k<=160;k++)
    ChassisYawFeedback_Observe(base+k*20,true,true,.04f,base+k*20,k*2,1);
}
static void YawStopHook(void) { ChassisRuntime_Stop(fixture_feedback_time); }
int main(void)
{
  for(unsigned wrap=0;wrap<2;wrap++) for(int sign=-1;sign<=1;sign+=2) {
    const uint32_t base=wrap?0xffffe000U:10000U;
    Init(base);Calibrate(base);
    for(unsigned k=0;k<150;k++) {
      uint32_t now=base+3220+k*20;Heartbeat(now);FeedbackExcept(-1);
      ChassisYawFeedback_Observe(now,false,true,.04f+sign*.32f,now,322+k*2,1);
      assert(ChassisRuntime_SetVelocity(0,sign*.4f,now));
      ChassisRuntime_Process(now,false);
    }
    ChassisRuntimeStatus runtime=Runtime();ChassisControllerStatus controller;
    assert(ChassisController_GetStatus(&controller));
    assert(runtime.state==CHASSIS_RUNTIME_STATE_ACTIVE && controller.active);
    assert(fabsf(runtime.applied_command.angular_z_rad_s-sign*.4f)<.0001f);
    assert(sign*controller.command.angular_z_rad_s>.41f);
    assert(sign*controller.command.angular_z_rad_s<=.5001f);
    YawRateControlState yaw;assert(ChassisRuntime_GetYawControl(&yaw));
    assert(sign*yaw.correction>0 && yaw.active);
    uint32_t unhealthy=base+6220;Heartbeat(unhealthy);FeedbackExcept(MOTOR_DRIVER_WHEEL_LF);
    ChassisYawFeedback_Observe(unhealthy,false,true,.04f,unhealthy,622,1);
    assert(ChassisRuntime_SetVelocity(0,sign*.4f,unhealthy));
    ChassisRuntime_Process(unhealthy,false);
    /* The outer loop reads the previous inner-controller snapshot. The first
     * process call publishes the new bad wheel feedback; the next consumes it. */
    assert(ChassisRuntime_GetYawControl(&yaw));
    const float old_correction=fabsf(yaw.correction);
    unhealthy+=20;Heartbeat(unhealthy);FeedbackExcept(MOTOR_DRIVER_WHEEL_LF);
    ChassisYawFeedback_Observe(unhealthy,false,true,.04f,unhealthy,624,1);
    assert(ChassisRuntime_SetVelocity(0,sign*.4f,unhealthy));
    ChassisRuntime_Process(unhealthy,false);
    assert(ChassisRuntime_GetYawControl(&yaw));
    assert(yaw.inhibited && yaw.integral==0 && fabsf(yaw.correction)<old_correction);
#if CHASSIS_CONTROL_REVISION >= 204
    for(unsigned k=0;k<60;k++) {
      unhealthy+=20;Heartbeat(unhealthy);FeedbackExcept(MOTOR_DRIVER_WHEEL_LF);
      ChassisYawFeedback_Observe(unhealthy,false,true,.04f+sign*.5f,unhealthy,626+k*2,1);
      assert(ChassisRuntime_SetVelocity(0,sign*.4f,unhealthy));
      ChassisRuntime_Process(unhealthy,false);
    }
    assert(ChassisRuntime_GetYawControl(&yaw));
    assert(yaw.inhibited && yaw.integral==0 && sign*yaw.correction<0);
    assert(ChassisController_GetStatus(&controller));
    assert(sign*controller.command.angular_z_rad_s<.4f);
    assert(sign*controller.command.angular_z_rad_s>=.2999f);
#endif
    fixture_exit_hook=YawStopHook;
    ChassisRuntime_Process(unhealthy+20,false);
    AssertDisabled();assert(ChassisRuntime_GetYawControl(&yaw));
    assert(yaw.correction==0 && yaw.integral==0 && !yaw.active);
    /* A new session must not inherit an angular correction. */
    Heartbeat(unhealthy+40);
    assert(ChassisRuntime_SetVelocity(.04f,0,unhealthy+40));
    ChassisRuntime_Process(unhealthy+40,false);
    assert(ChassisController_GetStatus(&controller));
    assert(controller.command.angular_z_rad_s==0);
    ChassisRuntime_StopForTransportLoss(unhealthy+41);AssertDisabled();
    assert(ChassisRuntime_GetYawControl(&yaw) && yaw.correction==0);
  }
  Init(100000);
  ChassisRuntime_LatchHealthFault(100001);
  ChassisRuntimeEvent events[CHASSIS_RUNTIME_EVENT_CAPACITY];uint32_t overwritten;
  const uint32_t count=ChassisRuntime_GetEvents(events,&overwritten);
  assert(count==1 && events[0].kind==CHASSIS_EVENT_HEALTH_FAULT);
  for(unsigned k=0;k<1000;k++) ChassisRuntime_LatchHealthFault(100002+k*10);
  assert(ChassisRuntime_GetEvents(events,&overwritten)==count && overwritten==0);
  AssertDisabled();
  puts("PASS: real runtime applies bounded yaw correction in both directions, preserves nominal profile, Stop invalidates computation, no correction leaks after restart/transport loss, including wrap");
  return 0;
}
