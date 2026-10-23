/* Actual trace recorder, mocked producers: retention and STOP interleaving. */
#include <assert.h>
#include <stdio.h>
#include <stdarg.h>
#include "motion_trace.c"

static unsigned depth;
static bool injected_stop;
static ChassisRuntimeStatus live_runtime;
void Test_EnterCritical(void) { ++depth; }
void Test_ExitCritical(void) { assert(depth); --depth; }
bool ChassisRuntime_GetStatus(ChassisRuntimeStatus *s) { *s=live_runtime;return true; }
bool ChassisRuntime_IsActive(void) { return live_runtime.state==CHASSIS_RUNTIME_STATE_ACTIVE; }
bool ChassisController_IsActive(void) { return ChassisRuntime_IsActive(); }
bool ChassisController_GetStatus(ChassisControllerStatus *s)
{ memset(s,0,sizeof *s);s->active=ChassisRuntime_IsActive();return true; }
bool MotorDriver_GetStatus(MotorDriverStatus *s)
{
  if(injected_stop) live_runtime.state=CHASSIS_RUNTIME_STATE_IDLE;
  memset(s,0,sizeof *s);s->initialized=true;s->standby_enabled=ChassisRuntime_IsActive();return true;
}
bool WheelSpeedEstimator_GetSnapshot(WheelSpeedState s[ENCODER_WHEEL_COUNT])
{ memset(s,0,sizeof(*s)*ENCODER_WHEEL_COUNT);for(unsigned w=0;w<4;w++)s[w].sample_period_ms=20;return true; }
bool WheelSpeedController_GetState(MotorDriverWheel w,WheelSpeedControllerState *s)
{ (void)w;memset(s,0,sizeof *s);return true; }
bool DebugLog_Printf(DebugLog_Level level,const char *format,...)
{ (void)level;(void)format;assert(!depth);return true; }

int main(void)
{
  live_runtime.state=CHASSIS_RUNTIME_STATE_ACTIVE;
  for(unsigned t=0;t<6000;t+=20) MotionTrace_Record(t);
  live_runtime.state=CHASSIS_RUNTIME_STATE_IDLE;MotionTrace_Record(6000);
  assert(s_count==128);
  unsigned oldest=(s_head+MOTION_TRACE_DEPTH-s_count)%MOTION_TRACE_DEPTH;
  assert(s_ring[oldest].time_ms==3460);
  assert(s_ring[(s_head+MOTION_TRACE_DEPTH-1)%MOTION_TRACE_DEPTH].time_ms==6000);
  for(unsigned k=0;k<s_count;k++)
    assert(s_ring[(oldest+k)%MOTION_TRACE_DEPTH].time_ms!=2500);
  printf("TRACE_CAPACITY,%u,%zu,%zu,%u,%u\n",s_count,sizeof(TraceSample),sizeof s_ring,
    s_ring[oldest].time_ms,s_ring[(s_head+MOTION_TRACE_DEPTH-1)%MOTION_TRACE_DEPTH].time_ms);
  puts("PASS: six-second run retains only 128 samples, 2540ms endpoint span; midpoint event overwritten");
  /* Runtime snapshot then a modeled safety STOP at the motor-status producer.
   * Each getter can be coherent individually; the combined sample is not atomic. */
  live_runtime.state=CHASSIS_RUNTIME_STATE_ACTIVE;injected_stop=true;
  MotionTrace_Record(7000);
  TraceSample last=s_ring[(s_head+MOTION_TRACE_DEPTH-1)%MOTION_TRACE_DEPTH];
  assert(last.state==CHASSIS_RUNTIME_STATE_ACTIVE && last.standby==0);
  assert(live_runtime.state==CHASSIS_RUNTIME_STATE_IDLE);
  assert(!depth);
  puts("PASS: actual recorder permits mixed pre-STOP runtime and post-STOP output snapshots; not a torque/waveform measurement");
  return 0;
}
