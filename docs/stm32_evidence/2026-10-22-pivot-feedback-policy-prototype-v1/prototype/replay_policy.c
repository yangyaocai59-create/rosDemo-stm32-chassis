/* Native offline stdin/stdout adapter. No firmware/ports/HAL dependencies. */
#include "pivot_feedback_policy.h"
#include <inttypes.h>
#include <stdio.h>
#include <string.h>

int main(int argc,char **argv)
{
  PfpState state;
  Pfp_Init(&state);
  bool first=true;
  PfpInput in;
  int pwm;
  unsigned active,valid;
  while (scanf("%"SCNu32" %"SCNu32" %"SCNu32" %"SCNd64" %"SCNd32" %f %f %d %u %u",
         &in.now_ms,&in.sample_ms,&in.sample_period_ms,&in.count,&in.delta,
         &in.target_cps,&in.measured_cps,&pwm,&active,&valid)==10)
  {
    if(pwm < -300 || pwm > 300) return 2;
    in.applied_pwm=(int16_t)pwm;
    in.active=active!=0; in.sample_valid=valid!=0;
    if(first && argc==2 && strcmp(argv[1],"--warm-start")==0)
    {
      /* Explicit replay assumption only: startup completed before this
       * truncated trace; preceding hidden policy state is NOT reconstructed. */
      state.started=true; state.watching=true;
      state.sign=in.target_cps>0?1:-1;
      state.started_ms=in.now_ms-PFP_STARTUP_MS;
      state.last_progress_ms=in.now_ms;
    }
    first=false;
    PfpResult result=Pfp_Update(&state,&in);
    printf("%"PRIu32",%u,%"PRIu32",%"PRIu32"\n",in.now_ms,
      (unsigned)result,state.last_progress_ms,state.severe_since_ms);
  }
  return ferror(stdin)?1:0;
}
