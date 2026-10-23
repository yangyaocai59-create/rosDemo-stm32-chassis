/* Actual recorder, prescribed producers. No hardware or motor commands. */
#include <assert.h>
#include <stdarg.h>
#include <stdio.h>
#include "motion_trace.c"

static unsigned depth, lines;
static unsigned fail_line;
static bool latch, inject_estop, pending_stop, schedule_stop, emit;
static ChassisRuntimeStatus runtime;
static ChassisControllerStatus controller;
static MotorDriverStatus motor;
static MotorDriverDiagnostics hardware;
static WheelSpeedState speeds[4];
static WheelSpeedControllerState controls[4];

static void Stop(void)
{ runtime.state=CHASSIS_RUNTIME_STATE_IDLE; controller.active=false;
  motor.standby_enabled=false; hardware.standby_pin=0; }
void Test_EnterCritical(void) { ++depth; }
void Test_ExitCritical(void)
{ assert(depth); --depth; if(!depth && pending_stop) {pending_stop=false; Stop();} }
bool EmergencyStop_IsLatched(void) { return latch; }
bool ChassisRuntime_GetStatus(ChassisRuntimeStatus *s)
{ assert(depth); *s=runtime; return true; }
bool ChassisController_GetStatus(ChassisControllerStatus *s)
{ assert(depth); *s=controller; return true; }
bool ChassisRuntime_IsActive(void) { return runtime.state==CHASSIS_RUNTIME_STATE_ACTIVE; }
bool ChassisController_IsActive(void) { return controller.active; }
bool MotorDriver_GetStatus(MotorDriverStatus *s)
{
  assert(depth);
  if(schedule_stop) {pending_stop=true; schedule_stop=false;}
  if(inject_estop) {latch=true; inject_estop=false; Stop();}
  *s=motor; return true;
}
bool MotorDriver_GetDiagnostics(MotorDriverDiagnostics *s)
{ assert(depth); *s=hardware; return true; }
bool WheelSpeedEstimator_GetSnapshot(WheelSpeedState s[4])
{ assert(depth); memcpy(s,speeds,sizeof speeds); return true; }
bool EncoderDiagnostics_Read(EncoderDiagnosticWheel s[4])
{ assert(depth); memset(s,0,sizeof(*s)*4); return true; }
bool WheelSpeedController_GetState(MotorDriverWheel w,WheelSpeedControllerState *s)
{ assert(depth); *s=controls[w]; return true; }
bool DebugLog_Printf(DebugLog_Level level,const char *format,...)
{
  (void)level; assert(!depth); ++lines;
  if (fail_line && lines==fail_line) return false;
  va_list args; va_start(args,format);
  if(emit) {vprintf(format,args); putchar('\n');}
  va_end(args); return true;
}
static TraceSample *Last(void)
{ return &s_ring[(s_head+MOTION_TRACE_DEPTH-1U)%MOTION_TRACE_DEPTH]; }
static unsigned Oldest(void)
{ return (s_head+MOTION_TRACE_DEPTH-s_count)%MOTION_TRACE_DEPTH; }
static void Reset(void)
{
  assert(!depth); s_head=s_count=s_dump_index=s_dump_remaining=0;
  s_was_active=s_dumping=s_frozen=s_triggered=s_dump_header=false;
  s_last_record_ms=s_start_ms=s_sequence=s_trigger_ms=s_trigger_sequence=s_last_speed_ms=0;
  s_reason=s_trigger_mask=s_post_remaining=0; memset(s_low_ms,0,sizeof s_low_ms);
  latch=inject_estop=pending_stop=schedule_stop=false; lines=0; fail_line=0;
  runtime=(ChassisRuntimeStatus){.state=CHASSIS_RUNTIME_STATE_ACTIVE};
  controller=(ChassisControllerStatus){.active=true};
  motor=(MotorDriverStatus){.standby_enabled=true};
  hardware=(MotorDriverDiagnostics){.standby_pin=1,.auto_reload=8399,.bdtr=32768,.ccer=4369};
  for(unsigned w=0;w<4;w++)
  {
    speeds[w]=(WheelSpeedState){.valid=true,.sample_period_ms=20};
    controls[w]=(WheelSpeedControllerState){.target_counts_per_second=240,
      .feedforward_permille=100,.integrator_permille=50,.command_permille=150};
    controller.targets.counts_per_second[w]=240;
    motor.command_permille[w]=150; hardware.requested_permille[w]=150;
    hardware.compare[w]=1260;
  }
}
static void Record(uint32_t t,int low_w)
{
  controller.last_control_ms=t;
  for(unsigned w=0;w<4;w++)
  {
    const float speed=(int)w==low_w ? 0 : controls[w].target_counts_per_second;
    speeds[w].sample_time_ms=t; speeds[w].sample_period_ms=20;
    speeds[w].raw_counts_per_second=speeds[w].filtered_counts_per_second=speed;
    speeds[w].delta_counts=speed==0 ? 0 : (speed>0 ? 5:-5);
    speeds[w].accumulated_count+=speeds[w].delta_counts;
    controls[w].measured_counts_per_second=speed;
  }
  MotionTrace_Record(t);
}
int main(int argc,char **argv)
{
  (void)argv;
  for(unsigned w=0;w<4;w++) for(int sign=-1;sign<=1;sign+=2)
  {
    Reset();
    controls[w].target_counts_per_second=controller.targets.counts_per_second[w]=sign*240;
    for(unsigned t=0;t<6000;t+=20) Record(t,t>=2500?(int)w:-1);
    assert(s_frozen && s_reason==TRACE_UNDERSPEED && s_trigger_mask==(1U<<w));
    assert(s_trigger_ms==2580 && Last()->time_ms==3080 && s_count==128);
    assert(s_ring[Oldest()].time_ms==540);
    assert(s_post_remaining==0);
    const TraceSample preserved=*Last(); Record(8000,-1); Stop(); Record(8020,-1);
    assert(memcmp(&preserved,Last(),sizeof preserved)==0);
    assert(lines==0);
  }
  puts("PASS: all four wheels/both directions preserve midpoint anomaly, 25 post frames, no live log");
  Reset(); for(unsigned t=0;t<30000;t+=20) Record(t,-1);
  assert(!s_triggered); Stop(); Record(30000,-1);
  assert(s_frozen && s_reason==TRACE_STOP && Last()->time_ms==30000);
  assert(s_ring[Oldest()].time_ms==27460);
  puts("PASS: healthy 30-second run and normal STOP, bounded rolling retention");
  Reset(); for(unsigned t=0;t<1100;t+=20) Record(t,t==1040?0:-1);
  assert(!s_triggered);
  for(unsigned t=1100;t<1180;t+=20) Record(t,0);
  assert(!s_triggered);
  speeds[0].sample_period_ms=100; MotionTrace_Record(1200);
  assert(!s_triggered);
  puts("PASS: isolated low sample and stale/missing sample cannot qualify sustained low feedback");
  Reset(); for(unsigned t=0;t<=2580;t+=20) Record(t,t>=2500?2:-1);
  Stop(); Record(2600,-1);
  assert(s_frozen && s_reason==TRACE_UNDERSPEED && Last()->time_ms==2600 && s_post_remaining==24);
  const unsigned count=s_count;
  runtime.state=CHASSIS_RUNTIME_STATE_ACTIVE; controller.active=true;
  motor.standby_enabled=true; hardware.standby_pin=1;
  Record(4000,-1); assert(s_count==count && Last()->time_ms==2600);
  assert(!MotionTrace_RequestDump()); Stop(); assert(MotionTrace_RequestDump());
  assert(!MotionTrace_RequestDump());
  runtime.state=CHASSIS_RUNTIME_STATE_ACTIVE;
  assert(MotionTrace_ProcessDump() && lines==0 && s_dump_remaining==count);
  Stop(); unsigned drained=0; while(MotionTrace_ProcessDump()) ++drained;
  assert(drained==count && !s_frozen && !s_dumping && s_count==0);
  runtime.state=CHASSIS_RUNTIME_STATE_ACTIVE; controller.active=true;
  motor.standby_enabled=true; hardware.standby_pin=1; Record(5000,-1);
  assert(s_count==1 && !s_frozen);
  puts("PASS: early STOP closes capture, later motion cannot overwrite; dump/rearm and motion pause");
  Reset(); Record(0,-1); schedule_stop=true; Record(20,-1);
  assert(Last()->state==CHASSIS_RUNTIME_STATE_ACTIVE && Last()->standby==1);
  assert(runtime.state==CHASSIS_RUNTIME_STATE_IDLE); Record(40,-1);
  assert(Last()->state==CHASSIS_RUNTIME_STATE_IDLE && Last()->standby==0);
  Reset(); Record(0,-1); inject_estop=true; Record(20,-1);
  assert((Last()->flags & (TRACE_ESTOP_PRESENT|TRACE_ESTOP_BOUNDARY))==3U);
  assert(s_frozen && s_reason==TRACE_ESTOP);
  puts("PASS: task STOP is deferred outside snapshot; asynchronous E-stop boundary explicitly marked");
  Reset(); const uint32_t base=0xfffff800U;
  for(unsigned t=0;t<5000;t+=20) Record(base+t,t>=2500?3:-1);
  assert(s_trigger_ms==base+2580 && Last()->time_ms==base+3080);
  puts("PASS: timestamp wrap does not break cadence or trigger/post window");
  Reset(); Record(0,-1); runtime.state=CHASSIS_RUNTIME_STATE_FAULT;
  runtime.fault=CHASSIS_RUNTIME_FAULT_HEALTH_MANAGER; controller.active=false;
  motor.standby_enabled=false; hardware.standby_pin=0; Record(20,-1);
  assert(s_reason==TRACE_FAULT && s_frozen);
  puts("PASS: fault transition freezes immediately without modifying fault/control");
  Reset(); Record(0,-1); Stop(); Record(20,-1);
  assert(MotionTrace_RequestDump()); fail_line=3;
  assert(MotionTrace_ProcessDump());
  assert(lines==3 && s_frozen && !s_dumping && s_count==2);
  fail_line=0; assert(MotionTrace_RequestDump());
  while(MotionTrace_ProcessDump()) {}
  assert(!s_frozen && s_count==0);
  puts("PASS: UART failure aborts after first failed write and preserves whole capture for retry");
  Reset();
  for(unsigned w=0;w<4;w++) speeds[w].accumulated_count=-(INT64_C(1)<<40)-w;
  Record(0,-1); Stop(); Record(20,-1);
  assert(Last()->wheel[3].total==-(INT64_C(1)<<40)-3+10);
  emit=argc>1; assert(MotionTrace_RequestDump());
  while(MotionTrace_ProcessDump()) {}
  printf("PASS: signed 64-bit totals preserved; frame=%zu ring=%zu bytes\n",sizeof(TraceSample),sizeof s_ring);
  return 0;
}

uint32_t ChassisRuntime_GetEvents(ChassisRuntimeEvent events[CHASSIS_RUNTIME_EVENT_CAPACITY],
                                 uint32_t *overwritten)
{ (void)events; *overwritten=0U; return 0U; }
