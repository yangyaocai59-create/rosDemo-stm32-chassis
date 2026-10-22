/* Conditional replay of a truncated post-state ring. NOT a plant model.
 * Quantized seeds and RF recovery=0 are hypotheses, not measured private state.
 * Native executable only; never opens serial, links firmware or accesses GPIO. */
#define main FrozenRuntimeRegressionMain
#ifdef BASELINE
#include "../2026-10-22-v01819-offline/baseline_0.18.18/test_runtime_motion.c"
#else
#include "source_snapshot/test_runtime_motion.c"
#endif
#undef main
#ifdef BASELINE
#include "../2026-10-22-v01819-offline/baseline_0.18.18/chassis_controller.c"
#include "../2026-10-22-v01819-offline/baseline_0.18.18/wheel_speed_controller.c"
#else
#include "source_snapshot/chassis_controller.c"
#include "source_snapshot/wheel_speed_controller.c"
#endif

static float midpoint(int printed,float unit,float shift)
{
  return ((float)printed+(printed>0 ? .5f : printed<0 ? -.5f : 0)+shift)*unit;
}
int main(int argc,char **argv)
{
  const float shift=argc>1 ? strtof(argv[1],NULL) : 0;
  const bool initial_restart=argc>2 && strtol(argv[2],NULL,10)!=0;
  assert(fabsf(shift)<=.49f);
  ChassisVelocityCommand command={0,.3f};
  ChassisWheelTargets targets;
  assert(ChassisKinematics_ToWheelTargets(&command,1600,&targets));
  WheelSpeedController_Init();
  for(unsigned w=0;w<4;w++) ChassisController_ClearPivotWheel((MotorDriverWheel)w);
  const MotorDriverWheel rr=MOTOR_DRIVER_WHEEL_RR,rf=MOTOR_DRIVER_WHEEL_RF;
  const float target_rr=targets.counts_per_second[rr],target_rf=targets.counts_per_second[rf];
  float measured_rr=0,measured_rf=0;
  unsigned t,dt,count=0,last_rr=0,last_rf=0;
  int dc_rr,cps_rr,ff_rr,ix_rr,pwm_rr,dc_rf,cps_rf,ff_rf,ix_rf,pwm_rf;
  puts("t,rr_target,rr_measured,rr_ff,rr_p,rr_i,rr_pwm,rr_bias,rr_steady_bias,rr_restart_active,rr_restart_age,rf_measured,rf_ff,rf_p,rf_i,rf_pwm,rf_load,rf_recovery,rf_overspeed_ms");
  while(scanf("%u %u %d %d %d %d %d %d %d %d %d %d",&t,&dt,
    &dc_rr,&cps_rr,&ff_rr,&ix_rr,&pwm_rr,&dc_rf,&cps_rf,&ff_rf,&ix_rf,&pwm_rf)==12)
  {
    assert(dt==20);
    if(count==0)
    {
      measured_rr=midpoint(cps_rr,1,shift); measured_rf=midpoint(cps_rf,1,shift);
      s_states[rr].integrator_permille=midpoint(ix_rr,.1f,shift);
      s_states[rf].integrator_permille=midpoint(ix_rf,.1f,shift);
      s_states[rr].feedforward_permille=WheelSpeedController_GetFeedforward(rr,target_rr);
      assert(fabsf(s_states[rr].feedforward_permille-ff_rr*.1f)<.11f);
      s_states[rf].feedforward_permille=midpoint(ff_rf,.1f,shift);
      s_states[rr].command_permille=(int16_t)pwm_rr;
      s_states[rf].command_permille=(int16_t)pwm_rf;
      s_previous_output[rr]=(float)pwm_rr+shift;
      s_previous_output[rf]=(float)pwm_rf+shift;
      s_previous_target[rr]=target_rr; s_previous_target[rf]=target_rf;
      s_pivot_breakaway_sign[rr]=1;
      s_pivot_has_moved[rr]=true;
      s_pivot_rear_restart[rr]=initial_restart;
      /* RR initial total legacy bias=0. Finished-restart vs no restart is
       * a sensitivity branch; neither reconstructs missing startup history. */
      s_front_pivot[rf]=(WheelSpeedFrontPivotState){.active=true,.sign=1,.moving_ms=40,
        .load_permille=s_states[rf].feedforward_permille-
          WheelSpeedController_GetFeedforward(rf,target_rf)};
      last_rr=last_rf=t;
    }
    else
    {
      const float alpha=(float)dt/(CHASSIS_SPEED_FILTER_TIME_CONSTANT_MS+dt);
      measured_rr+=alpha*(dc_rr*1000.0f/dt-measured_rr);
      measured_rf+=alpha*(dc_rf*1000.0f/dt-measured_rf);
      if(dc_rr) last_rr=t; else if(t-last_rr>=CHASSIS_SPEED_STOP_TIMEOUT_MS) measured_rr=0;
      if(dc_rf) last_rf=t; else if(t-last_rf>=CHASSIS_SPEED_STOP_TIMEOUT_MS) measured_rf=0;
      const float bias=ChassisController_GetFeedforwardBias(&command,rr,target_rr,measured_rr,t,dt);
      int16_t output;
      assert(WheelSpeedController_UpdateWithMode(rr,target_rr,measured_rr,bias,
        WHEEL_SPEED_CONTROL_MODE_PIVOT,dt,&output));
      assert(WheelSpeedController_UpdateWithMode(rf,target_rf,measured_rf,0,
        WHEEL_SPEED_CONTROL_MODE_PIVOT,dt,&output));
    }
    const WheelSpeedControllerState *r=&s_states[rr],*f=&s_states[rf];
    const WheelSpeedFrontPivotState *front=&s_front_pivot[rf];
    printf("%u,%.9f,%.9f,%.9f,%.9f,%.9f,%d,%.9f,%.9f,%d,%u,%.9f,%.9f,%.9f,%.9f,%d,%.9f,%.9f,%u\n",
      t,target_rr,measured_rr,r->feedforward_permille,r->proportional_permille,
      r->integrator_permille,r->command_permille,s_pivot_applied_bias[rr],
      s_pivot_steady_bias[rr],s_pivot_breakaway_active[rr],
      s_pivot_rear_restart[rr] ? t-s_pivot_breakaway_start_ms[rr] : 0,
      measured_rf,f->feedforward_permille,f->proportional_permille,f->integrator_permille,
      f->command_permille,front->load_permille,front->recovery_permille,front->load_overspeed_ms);
    ++count;
  }
  assert(count==127);
  return 0;
}
