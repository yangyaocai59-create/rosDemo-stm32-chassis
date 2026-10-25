#include <assert.h>
#include <math.h>
#include <stdio.h>
#include "pivot_feedback_guard.h"
int main(void)
{
  const float targets[]={1,2,5,19,20,240,1600};
  for(unsigned k=0;k<sizeof(targets)/sizeof(targets[0]);k++)
    for(int sign=-1;sign<=1;sign+=2) for(unsigned wrap=0;wrap<2;wrap++) {
      uint32_t base=wrap?0xfffffc00U:0U;
      PivotFeedbackGuard state={0};
      for(unsigned t=0;t<=3000;t+=20)
        assert(PivotFeedbackGuard_Update(&state,true,sign*targets[k],0,base+t)==(t==3000));
      assert(state.failed);
      state=(PivotFeedbackGuard){0};
      for(unsigned t=0;t<=5000;t+=20) {
        const float speed=t>=1200?sign*targets[k]:0;
        assert(!PivotFeedbackGuard_Update(&state,true,sign*targets[k],speed,base+t));
        if(t>=1300) assert(!state.low&&!state.warning);
      }
    }
  puts("PASS: guard covers 1..1600cps, both signs/wrap; persistent no-feedback latches at 3s and exact-target recovery is achievable at 1cps");
  for(int sign=-1;sign<=1;sign+=2) {
    PivotFeedbackGuard state={0};float filtered=0;
    unsigned last_count=0;
    for(unsigned t=0;t<=20000;t+=20) {
      bool pulse=t!=0&&t%1000==0;
      if(pulse)last_count=t;
      filtered+=(20.0f/70.0f)*((pulse?50:0)-filtered);
      if(t-last_count>=200)filtered=0;
      assert(!PivotFeedbackGuard_Update(&state,true,sign*1.0f,sign*filtered,t));
    }
  }
  puts("PASS: legitimate sparse one-count-per-second feedback with the actual LPF/stop rule does not falsely latch");
  return 0;
}
