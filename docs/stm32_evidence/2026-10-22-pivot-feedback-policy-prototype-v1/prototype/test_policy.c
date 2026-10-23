#include "pivot_feedback_policy.h"
#include <assert.h>
#include <limits.h>
#include <math.h>
#include <stdio.h>

typedef struct {
  PfpState state;
  PfpInput input;
  uint32_t base;
  int sign;
} Fixture;

static void Init(Fixture *f, int sign, uint32_t base)
{
  *f = (Fixture){0};
  Pfp_Init(&f->state);
  f->base=base; f->sign=sign;
  f->input=(PfpInput){.sample_period_ms=20,.active=true,.sample_valid=true,
    .target_cps=sign*240.0f,.applied_pwm=(int16_t)(sign*150)};
}

static PfpResult Tick(Fixture *f,uint32_t elapsed,int delta,float speed)
{
  f->input.now_ms=f->base+elapsed;
  f->input.measured_cps=f->sign*speed;
  if (elapsed%20U==0U)
  {
    f->input.sample_ms=f->input.now_ms;
    f->input.delta=f->sign*delta;
    f->input.count+=f->input.delta;
  }
  return Pfp_Update(&f->state,&f->input);
}

static void Warm(Fixture *f)
{
  for(unsigned t=0;t<=2000;t+=10)
    assert(Tick(f,t,5,240)==PFP_NORMAL);
}

static void TestFourIndependentLanes(void)
{
  Fixture f[4];
  for(unsigned w=0;w<4;w++) { Init(&f[w],w%2 ? -1:1,0); Warm(&f[w]); }
  for(unsigned t=2010;t<=2750;t+=10)
    for(unsigned w=0;w<4;w++)
    {
      PfpResult r=Tick(&f[w],t,w==2?0:5,w==2?0:240);
      if(w!=2) assert(r==PFP_NORMAL);
      else assert(t==2750 ? r==PFP_FAULT_NO_PROGRESS : !Pfp_IsFault(r));
    }
  puts("PASS: independent four-wheel state; one failed lane cannot mutate others");
}

static void TestShortGaps(void)
{
  for(unsigned w=0;w<4;w++)
    for(int sign=-1;sign<=1;sign+=2)
    {
      Fixture f; Init(&f,sign,0); Warm(&f);
      bool warned=false;
      for(unsigned t=2010;t<=4000;t+=10)
      {
        bool gap=t<2400;
        PfpResult r=Tick(&f,t,gap?0:5,gap?0:240);
        assert(!Pfp_IsFault(r));
        warned|=r==PFP_WARNING;
      }
      assert(warned && f.state.result==PFP_NORMAL);
    }
  puts("PASS: 400ms feedback gap warns then recovers in all 4 lanes/both signs");
}

static void TestNoProgress(void)
{
  for(int sign=-1;sign<=1;sign+=2)
    for(unsigned low_pwm=0;low_pwm<2;low_pwm++)
    {
      Fixture f; Init(&f,sign,0); Warm(&f);
      for(unsigned t=2010;t<=2750;t+=10)
      {
        f.input.applied_pwm=(int16_t)(sign*(low_pwm?1:180));
        PfpResult r=Tick(&f,t,0,0);
        assert(t==2750 ? r==PFP_FAULT_NO_PROGRESS : !Pfp_IsFault(r));
      }
      f.input.active=false;
      assert(Pfp_Update(&f.state,&f.input)==PFP_FAULT_NO_PROGRESS);
    }
  puts("PASS: exact 750ms no-progress bound, duplicate samples, faded PWM and sticky fault");
}

static void TestStartupAndWrap(void)
{
  for(unsigned wrap=0;wrap<2;wrap++)
  {
    Fixture f; Init(&f,1,wrap?0xfffffc00U:0U);
    for(unsigned t=0;t<=1750;t+=10)
    {
      PfpResult r=Tick(&f,t,0,0);
      assert(t==1750?r==PFP_FAULT_NO_PROGRESS:!Pfp_IsFault(r));
    }
  }
  puts("PASS: 1000ms startup grace plus bounded no-progress deadline, uint32 tick wrap");
}

static void TestSparseOrChatteringCounts(void)
{
  for(unsigned chatter=0;chatter<2;chatter++)
  {
    Fixture f; Init(&f,-1,0); Warm(&f);
    for(unsigned t=2010;t<=3510;t+=10)
    {
      int delta=chatter ? ((t/20)%2 ? 1:-1) : (t%100==0 ? 1:0);
      PfpResult r=Tick(&f,t,delta,10);
      assert(t==3510?r==PFP_FAULT_SEVERE:!Pfp_IsFault(r));
    }
  }
  puts("PASS: sparse pulses and zero-net chatter cannot hide severe failure beyond 1500ms");
}

static void TestBriefSpeedSpikes(void)
{
  Fixture f; Init(&f,1,0); Warm(&f);
  for(unsigned t=2010;t<=3510;t+=10)
  {
    bool spike=t%200==0;
    PfpResult r=Tick(&f,t,t%100==0?1:0,spike?240:10);
    assert(t==3510?r==PFP_FAULT_SEVERE:!Pfp_IsFault(r));
  }
  puts("PASS: isolated healthy-speed spikes do not reset continuous severe episode");
}

static void TestEvidenceFailures(void)
{
  for(unsigned kind=0;kind<9;kind++)
  {
    Fixture f; Init(&f,1,0); Warm(&f);
    switch(kind)
    {
      case 0: f.input.now_ms+=101; break;
      case 1: f.input.count++; break; /* duplicate stamp, changed count */
      case 2: f.input.sample_ms-=20; break;
      case 3: f.input.sample_valid=false; break;
      case 4: f.input.sample_period_ms=0; break;
      case 5: f.input.target_cps=NAN; break;
      case 6: f.input.measured_cps=INFINITY; break;
      case 7: f.input.sample_ms+=21; break;
      case 8: f.input.applied_pwm=301; break;
    }
    assert(Pfp_Update(&f.state,&f.input)==PFP_FAULT_EVIDENCE);
  }
  puts("PASS: stale/backwards/inconsistent/invalid evidence fails boundedly, not as progress");
}

static void TestFutureAndCounterOverflow(void)
{
  Fixture f; Init(&f,1,0); Warm(&f);
  f.input.now_ms=2010;
  f.input.sample_ms=2020;
  f.input.count+=5;
  assert(Pfp_Update(&f.state,&f.input)==PFP_NORMAL);
  assert(f.state.last_progress_ms==2010);
  Init(&f,1,0);
  f.input.count=INT64_MAX;
  assert(Pfp_Update(&f.state,&f.input)==PFP_NORMAL);
  f.input.now_ms=20; f.input.sample_ms=20; f.input.delta=1;
  f.input.count=INT64_MIN;
  assert(Pfp_Update(&f.state,&f.input)==PFP_FAULT_EVIDENCE);
  puts("PASS: newer-producer/older-caller tick and signed position overflow rejection");
}

static void TestOutOfScopeAndSignChange(void)
{
  Fixture f; Init(&f,1,0); Warm(&f);
  f.input.target_cps=199.9f;
  assert(Pfp_Update(&f.state,&f.input)==PFP_NOT_SUPERVISED);
  Init(&f,1,0); Warm(&f);
  f.input.target_cps=-240;
  f.input.now_ms=2010;
  assert(Pfp_Update(&f.state,&f.input)==PFP_NORMAL);
  assert(f.state.started_ms==2010 && f.state.sign==-1);
  puts("PASS: lower-target path explicitly out of scope, direction change starts a new phase");
}

static void TestSkippedProducerSamples(void)
{
  Fixture f; Init(&f,1,0); Warm(&f);
  f.input.now_ms=2040; f.input.sample_ms=2040;
  f.input.delta=5; f.input.count+=10;
  assert(Pfp_Update(&f.state,&f.input)==PFP_NORMAL);
  assert(f.state.last_progress_ms==2040);
  f.input.now_ms=2050;
  assert(Pfp_Update(&f.state,&f.input)==PFP_NORMAL);
  assert(f.state.last_progress_ms==2040);
  f.input.now_ms=2100; f.input.sample_ms=2100;
  f.input.delta=5; f.input.count+=15;
  assert(Pfp_Update(&f.state,&f.input)==PFP_NORMAL);
  f.input.now_ms=2240; f.input.sample_ms=2240; f.input.count+=35;
  assert(Pfp_Update(&f.state,&f.input)==PFP_FAULT_EVIDENCE);
  puts("PASS: skipped producer samples use cumulative progress; duplicate/stale gaps remain bounded");
}

int main(void)
{
  TestFourIndependentLanes(); TestShortGaps(); TestNoProgress();
  TestStartupAndWrap(); TestSparseOrChatteringCounts(); TestBriefSpeedSpikes();
  TestEvidenceFailures(); TestFutureAndCounterOverflow(); TestOutOfScopeAndSignChange();
  TestSkippedProducerSamples();
  return 0;
}
