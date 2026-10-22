/* Diagnostic only: real controller/fault code, prescribed zero feedback.
 * Does not model motor physics, access ports, or change the firmware image. */
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include "wheel_speed_controller.h"
#include "fault_manager.h"
#include "debug_log.h"

static unsigned depth;
void Test_EnterCritical(void) { ++depth; }
void Test_ExitCritical(void) { assert(depth); --depth; }
bool DebugLog_Printf(DebugLog_Level level, const char *format, ...)
{ (void)level; (void)format; assert(!depth); return true; }

static void Run(MotorDriverWheel wheel, float target)
{
  WheelSpeedController_Init();
  FaultManager_Init(0);
  FaultManager_MarkSystemInitComplete(0);
  unsigned first_fault = 0;
  printf("CASE wheel=%u target=%.1f feedback=0 dt=20ms\n", (unsigned)wheel, target);
  for (unsigned t=20; t<=6000; t+=20)
  {
    int16_t pwm=0;
    WheelSpeedControllerState c;
    WheelSpeedFrontPivotState f;
    FaultManagerStatus status;
    float targets[4]={0}, speeds[4]={0};
    int16_t outputs[4]={0};
    assert(WheelSpeedController_UpdateWithMode(wheel,target,0,0,
      WHEEL_SPEED_CONTROL_MODE_PIVOT,20,&pwm));
    assert(WheelSpeedController_GetState(wheel,&c));
    assert(WheelSpeedController_GetFrontPivotState(wheel,&f));
    targets[wheel]=target; outputs[wheel]=pwm;
    for(unsigned k=0;k<FAULT_MANAGER_TASK_COUNT;k++)
      FaultManager_Heartbeat((FaultManagerTask)k,t);
    FaultManager_Process(t);
    FaultManager_ObserveMotion(t,true,targets,speeds,outputs);
    assert(FaultManager_GetStatus(&status));
    if(status.latched_faults && !first_fault) first_fault=t;
    if(t==20 || t==40 || t==60 || t==100 || t==200 || t==300 ||
       t==360 || t==500 || t==1000 || t==1800 || t==6000)
      printf("t=%u pwm=%d ff=%.3f p=%.3f i=%.3f load=%.3f recovery=%.3f "
        "recovering=%u exhausted=%u still_ms=%u faults=%u\n",t,pwm,
        c.feedforward_permille,c.proportional_permille,c.integrator_permille,
        f.load_permille,f.recovery_permille,f.recovering,f.exhausted,
        f.still_ms,(unsigned)status.latched_faults);
  }
  printf("first_fault_ms=%u\n",first_fault);
  /* Isolated controller output above is printed after faults for diagnostics.
   * The production runtime inhibits drive on a latched fault; this is not a
   * claim that a real chassis continues to drive after that fault. */
}

int main(void)
{
  Run(MOTOR_DRIVER_WHEEL_RF,-160.0f);
  Run(MOTOR_DRIVER_WHEEL_RF,-199.0f);
  Run(MOTOR_DRIVER_WHEEL_RF,-200.0f);
  Run(MOTOR_DRIVER_WHEEL_RF,-280.0f);
  Run(MOTOR_DRIVER_WHEEL_LF,160.0f);
  return 0;
}
