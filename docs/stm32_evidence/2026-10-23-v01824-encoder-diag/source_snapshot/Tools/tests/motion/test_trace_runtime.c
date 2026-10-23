/* Real runtime + PI + fault manager with/without recorder: same prescribed
 * inputs must produce identical commands and fault timing. No motor model. */
#define main UnusedLegacyMain
#define WheelSpeedEstimator_GetSnapshot LegacySnapshot
#include "test_front_recovery_safety.c"
#undef WheelSpeedEstimator_GetSnapshot
#undef main
bool WheelSpeedEstimator_GetSnapshot(WheelSpeedState states[4]);
#include "motion_trace.c"

static uint32_t source_time;
static int64_t totals[4];
bool EncoderDiagnostics_Read(EncoderDiagnosticWheel s[4])
{ memset(s,0,sizeof(*s)*4); return true; }
bool WheelSpeedEstimator_GetSnapshot(WheelSpeedState states[4])
{
  for(unsigned w=0;w<4;w++) states[w]=(WheelSpeedState){
    .valid=true,.sample_period_ms=20,.sample_time_ms=source_time,
    .accumulated_count=totals[w],.delta_counts=(int32_t)(feedback[w]*.02f),
    .filtered_counts_per_second=feedback[w],.raw_counts_per_second=feedback[w]};
  return true;
}
bool MotorDriver_GetDiagnostics(MotorDriverDiagnostics *out)
{
  *out=(MotorDriverDiagnostics){.standby_pin=motor.standby_enabled,.auto_reload=8399};
  for(unsigned w=0;w<4;w++)
  { out->requested_permille[w]=motor.command_permille[w];
    out->compare[w]=(uint32_t)(abs(motor.command_permille[w])*8400/1000); }
  return true;
}
int main(int argc,char **argv)
{
  (void)argv; const bool tracing=argc>1;
  for(unsigned wheel=0;wheel<4;wheel++) for(int sign=-1;sign<=1;sign+=2)
  {
    Init(0); memset(totals,0,sizeof totals);
    s_dumping=s_frozen=s_was_active=s_triggered=false; s_count=s_head=0;
    uint32_t trip=0;
    for(unsigned t=0;t<=6000;t+=20)
    {
      Heartbeat(t);
      FeedbackExcept(((t>=2500 && t<2800) || t>=3500)?(int)wheel:-1);
      source_time=t;
      for(unsigned w=0;w<4;w++) totals[w]+=(int32_t)(feedback[w]*.02f);
      assert(ChassisRuntime_SetVelocity(0,sign*.3f,t));
      ChassisRuntime_Process(t,false); Safety(t);
      if(tracing) MotionTrace_Record(t);
      printf("%u,%d,%u,%u,%d,%d,%d,%d\n",wheel,sign,t,(unsigned)Runtime().state,
        motor.command_permille[0],motor.command_permille[1],
        motor.command_permille[2],motor.command_permille[3]);
      if(Runtime().state==CHASSIS_RUNTIME_STATE_FAULT) {trip=t; break;}
    }
    assert(trip>3500); AssertDisabled();
    if(tracing)
    {
      assert(s_triggered && s_frozen && s_reason==TRACE_UNDERSPEED);
      assert(s_trigger_mask==(1U<<wheel) && s_trigger_ms==2580);
      const unsigned oldest=(s_head+MOTION_TRACE_DEPTH-s_count)%MOTION_TRACE_DEPTH;
      for(unsigned n=0;n<s_count;n++)
        assert(s_ring[(oldest+n)%MOTION_TRACE_DEPTH].flags==0);
    }
  }
  return 0;
}
