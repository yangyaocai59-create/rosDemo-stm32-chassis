/* Actual read-only production guard; peripheral/control snapshots mocked.
 * Missing motor-write implementations intentionally make any write fail link. */
#include <assert.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>
#include "imu_calibration_guard.h"
#include "chassis_runtime.h"
#include "encoder_service.h"
#include "wheel_speed_estimator.h"
#include "motor_driver.h"
#include "emergency_stop.h"
#include "fault_manager.h"
#include "motor_open_loop_test.h"
#include "chassis_runtime_test.h"

static ChassisRuntimeStatus runtime;
static MotorDriverStatus motor;
static EncoderState encoders[4];
static WheelSpeedState speeds[4];
static uint32_t now_ms;
static unsigned depth, failed_query;
static bool controller, estop_ok, faulted, debug_motion, runtime_test;
void Test_EnterCritical(void) { ++depth; }
void Test_ExitCritical(void) { assert(depth); --depth; }
uint32_t HAL_GetTick(void) { return now_ms; }
bool ChassisRuntime_GetStatus(ChassisRuntimeStatus *out)
{ *out=runtime; return failed_query!=1U; }
bool MotorDriver_GetStatus(MotorDriverStatus *out)
{ *out=motor; return failed_query!=2U; }
bool EncoderService_GetSnapshot(EncoderState out[4])
{ memcpy(out,encoders,sizeof(encoders)); return failed_query!=3U; }
bool WheelSpeedEstimator_GetSnapshot(WheelSpeedState out[4])
{ memcpy(out,speeds,sizeof(speeds)); return failed_query!=4U; }
bool ChassisController_IsActive(void) { return controller; }
bool EmergencyStop_CanEnableMotor(void) { return estop_ok; }
bool FaultManager_HasCriticalFault(void) { return faulted; }
bool MotorOpenLoopTest_IsActive(void) { return debug_motion; }
bool ChassisRuntimeTest_IsActive(void) { return runtime_test; }
static void Reset(void)
{
  memset(&runtime,0,sizeof(runtime)); runtime.state=CHASSIS_RUNTIME_STATE_IDLE;
  memset(&motor,0,sizeof(motor)); motor.initialized=true;
  memset(encoders,0,sizeof(encoders)); memset(speeds,0,sizeof(speeds));
  controller=faulted=debug_motion=runtime_test=false; estop_ok=true;
  failed_query=0U; now_ms=1000U;
  for(unsigned i=0;i<4U;++i)
  { speeds[i].valid=speeds[i].stopped=true; speeds[i].sample_time_ms=1000U;
    encoders[i].accumulated_count=(int64_t)i*10000000000LL; }
}
int main(void)
{
  int64_t counts[4]; Reset(); assert(!ImuCalibrationGuard_Read(NULL,false));
  assert(ImuCalibrationGuard_Read(counts,false));
  for(unsigned i=0;i<4U;++i) assert(counts[i]==encoders[i].accumulated_count);
  for(unsigned query=1U;query<=4U;++query)
  { Reset(); failed_query=query; assert(!ImuCalibrationGuard_Read(counts,false)); }
  for(unsigned reason=0U;reason<11U;++reason)
  {
    Reset();
    switch(reason)
    {
      case 0:runtime.state=CHASSIS_RUNTIME_STATE_ACTIVE;break;
      case 1:runtime.fault=CHASSIS_RUNTIME_FAULT_HEALTH_MANAGER;break;
      case 2:runtime.command_valid=true;break;
      case 3:runtime.diagnostics_active=true;break;
      case 4:controller=true;break;
      case 5:estop_ok=false;break;
      case 6:faulted=true;break;
      case 7:debug_motion=true;break;
      case 8:runtime_test=true;break;
      case 9:motor.initialized=false;break;
      default:motor.standby_enabled=true;break;
    }
    assert(!ImuCalibrationGuard_Read(counts,false) && depth==0U);
  }
  for(unsigned wheel=0U;wheel<4U;++wheel)
  {
    for(unsigned reason=0U;reason<5U;++reason)
    {
      Reset();
      switch(reason)
      {
        case 0:motor.command_permille[wheel]=1;break;
        case 1:encoders[wheel].delta=1;break;
        case 2:speeds[wheel].valid=false;break;
        case 3:speeds[wheel].stopped=false;break;
        default:speeds[wheel].sample_time_ms=969U;break;
      }
      assert(!ImuCalibrationGuard_Read(counts,true) && depth==0U);
    }
  }
  Reset(); runtime.diagnostics_active=true;
  assert(!ImuCalibrationGuard_Read(counts,false));
  assert(ImuCalibrationGuard_Read(counts,true)); /* Own calibration inhibits motors. */
  debug_motion=true; assert(!ImuCalibrationGuard_Read(counts,true));
  debug_motion=false; runtime_test=true; assert(!ImuCalibrationGuard_Read(counts,true));
  Reset(); now_ms=1030U; assert(ImuCalibrationGuard_Read(counts,false));
  now_ms=1031U; assert(!ImuCalibrationGuard_Read(counts,false));
  Reset(); now_ms=15U;
  for(unsigned i=0;i<4U;++i) speeds[i].sample_time_ms=UINT32_MAX-14U;
  assert(ImuCalibrationGuard_Read(counts,false) && depth==0U);
  puts("PASS: production calibration guard, all four wheels, stale snapshots, active command/controller/diagnostic/fault/ESTOP/STBY rejection and tick wrap; no motor writes");
  return 0;
}
