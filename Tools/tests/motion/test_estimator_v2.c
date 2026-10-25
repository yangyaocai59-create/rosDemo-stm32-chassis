#include <assert.h>
#include <limits.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "wheel_speed_estimator.h"
#include "control_timing.h"

static EncoderState encoder[4];
static unsigned depth;
void Test_EnterCritical(void) { ++depth; }
void Test_ExitCritical(void) { assert(depth); --depth; }
bool EncoderService_GetSnapshot(EncoderState states[4])
{ memcpy(states,encoder,sizeof encoder); return true; }
static void Read(WheelSpeedState s[4]) { assert(WheelSpeedEstimator_GetSnapshot(s)); }

int main(void)
{
  WheelSpeedState s[4];
  for(unsigned wrap=0;wrap<2;wrap++) {
    const uint32_t base=wrap?0xfffffff0U:1000U;
    memset(encoder,0,sizeof encoder);
    assert(WheelSpeedEstimator_Init(base));
    WheelSpeedEstimator_Update(base);Read(s);assert(!s[0].valid);
    for(unsigned w=0;w<4;w++) encoder[w].accumulated_count=(w%2?-1:1)*12;
    WheelSpeedEstimator_Update(base+20);Read(s);
    for(unsigned w=0;w<4;w++) {
      const float sign=w%2?-1.0f:1.0f;
      assert(s[w].valid&&s[w].quality_flags==0&&s[w].sample_sequence==1);
      assert(s[w].sample_time_ms==base+20&&s[w].sample_period_ms==20);
      assert(s[w].raw_counts_per_second==sign*600);
      assert(fabsf(s[w].filtered_counts_per_second-sign*(600.0f*20/70))<.001f);
    }
    WheelSpeedEstimator_Update(base+20);Read(s);assert(s[0].sample_sequence==1);
    for(unsigned t=40;t<=220;t+=20) WheelSpeedEstimator_Update(base+t);
    Read(s);assert(s[0].valid&&s[0].stopped&&s[0].filtered_counts_per_second==0);
    encoder[0].accumulated_count++;
    WheelSpeedEstimator_Update(base+321);Read(s);
    assert(!s[0].valid&&(s[0].quality_flags&WHEEL_SPEED_QUALITY_INTERVAL));
    assert(s[0].accumulated_count==13); /* Position never fabricated or discarded. */
    encoder[0].accumulated_count++;
    WheelSpeedEstimator_Update(base+341);Read(s);
    assert(s[0].valid&&s[0].delta_counts==1&&s[0].raw_counts_per_second==50);
  }
  puts("PASS: real estimator preserves count units/filter, same-tick idempotence, coherent sequence, stop timeout and wrap; bad interval marked invalid then recovers");
  memset(encoder,0,sizeof encoder);assert(WheelSpeedEstimator_Init(0));
  encoder[0].accumulated_count=400;WheelSpeedEstimator_Update(20);Read(s);
  assert(s[0].valid&&s[0].raw_counts_per_second==20000);
  encoder[0].accumulated_count+=401;WheelSpeedEstimator_Update(40);Read(s);
  assert(!s[0].valid&&(s[0].quality_flags&WHEEL_SPEED_QUALITY_COUNT_RATE));
  assert(s[0].accumulated_count==801&&s[0].filtered_counts_per_second==0);
  encoder[0].accumulated_count+=(int64_t)INT32_MAX+100;
  WheelSpeedEstimator_Update(60);Read(s);
  assert(!s[0].valid&&(s[0].quality_flags&WHEEL_SPEED_QUALITY_COUNT_RATE));
  puts("PASS: 20000/20050cps boundary and wide count jumps reject control feedback while preserving original cumulative count");
  ControlTiming_RecordStart(CONTROL_TIMING_MOTOR,0xfffffff0U);
  ControlTiming_RecordStart(CONTROL_TIMING_MOTOR,4U);
  ControlTiming_RecordExecution(CONTROL_TIMING_MOTOR,16800,168000000);
  ControlTiming_RecordExecution(CONTROL_TIMING_MOTOR,33600,168000000);
  ControlTiming_RecordExecution(CONTROL_TIMING_MOTOR,1,0);
  ControlTimingStatus timing;assert(ControlTiming_GetStatus(CONTROL_TIMING_MOTOR,&timing));
  assert(timing.period_min_ms==20&&timing.period_max_ms==20);
  assert(timing.execution_max_us==200&&timing.execution_last_us==200&&timing.execution_samples==2);
  const uint32_t elapsed_cycles=100U-0xfffffff0U;
  ControlTiming_RecordSection(CONTROL_SECTION_TRACE,elapsed_cycles,1000000);
  ControlSectionTiming section;assert(ControlTiming_GetSection(CONTROL_SECTION_TRACE,&section));
  assert(section.max_us==116&&section.samples==1);
  assert(!ControlTiming_GetSection(CONTROL_SECTION_COUNT,&section));
  assert(!depth);
  puts("PASS: execution/section timing conversions, invalid clock and cycle/tick wrap accounting");
  return 0;
}
