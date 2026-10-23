#include <assert.h>
#include <stdio.h>
#include "pivot_feedback_guard.h"

int main(void)
{
  const float targets[]={20,80,160,240,280};
  for(unsigned w=0;w<4;w++) for(int sign=-1;sign<=1;sign+=2)
    for(unsigned m=0;m<5;m++) for(unsigned wrap=0;wrap<2;wrap++)
    {
      PivotFeedbackGuard s={0};const uint32_t base=wrap?0xfffffc00U:0;
      for(unsigned t=0;t<=3100;t+=10)
      {
        bool failed=PivotFeedbackGuard_Update(&s,true,sign*targets[m],0,base+t);
        assert(failed==(t>=3000));
        assert(s.warning==(t>=1250));
      }
      assert(PivotFeedbackGuard_Update(&s,true,sign*targets[m],sign*targets[m],base+3200));
    }
  puts("PASS: four independent wheels/both signs/targets/wrap, warning250, grace1000 + failure2000, sticky failure");
  for(unsigned length=100;length<=1800;length+=100)
  {
    PivotFeedbackGuard s={0};
    for(unsigned t=0;t<=6000;t+=10)
    {
      float speed=t>=1200 && t<1200+length?0:240;
      assert(!PivotFeedbackGuard_Update(&s,true,240,speed,t));
    }
    assert(!s.low && !s.warning);
  }
  puts("PASS: brief and recoverable low-feedback episodes warn and recover without latching");
  PivotFeedbackGuard s={0};
  for(unsigned t=0;t<=3100;t+=10)
  {
    /* 40ms of apparent normal speed must not reset two seconds of failure. */
    const float speed=t>=1700 && t<1740?240:0;
    const float target=240+(t%3); /* same-sign refresh cannot reset deadline */
    assert(PivotFeedbackGuard_Update(&s,true,target,speed,t)==(t>=3000));
  }
  s=(PivotFeedbackGuard){0};
  for(unsigned t=0;t<=3100;t+=10)
  {
    const float speed=t>=1500?45:0; /* between low36 and recovery60 */
    assert(PivotFeedbackGuard_Update(&s,true,240,speed,t)==(t>=3000));
  }
  puts("PASS: one short speed spike and hysteresis-band motion cannot endlessly reset a genuine failure");
  s=(PivotFeedbackGuard){0};
  for(unsigned t=0;t<=2800;t+=10) assert(!PivotFeedbackGuard_Update(&s,true,240,0,t));
  assert(!PivotFeedbackGuard_Update(&s,true,-240,0,2810) && !s.low && !s.warning);
  assert(!PivotFeedbackGuard_Update(&s,false,-240,0,2820) && !s.active);
  assert(!PivotFeedbackGuard_Update(&s,true,10,0,2830) && !s.active);
  puts("PASS: mode exit/zero-low target/direction change reset only their maneuver state");
  s=(PivotFeedbackGuard){0};
  for(unsigned t=0;t<=3010;t+=10)
    assert(PivotFeedbackGuard_Update(&s,true,160,21,t)==(t>=3000));
  puts("PASS: low target160 with speed21 is still bounded; legacy low-speed gap not retained");
}
