/* Conditional replay of a TRUNCATED ring, NOT a motor/load simulation.
 * First post-state comes from midpoints of printed quantization intervals;
 * front recovery=0 is a stated hypothesis checked against the following rows.
 * No serial, motor commands, firmware link or flash. */
#define main ExistingRuntimeRegressionMain
#include "prototype/wheel_speed_control_config.h"
#include "prototype/wheel_speed_controller.h"
#include "../../../Tools/tests/motion/test_runtime_motion.c"
#undef main
#include "prototype/chassis_controller.c"
#include "prototype/wheel_speed_controller.c"

static float midpoint(int value, float unit)
{
  return ((float)value + (value > 0 ? 0.5f : (value < 0 ? -0.5f : 0.0f))) * unit;
}

int main(int argc,char **argv)
{
  (void)argc;
  const float seed_shift = argv[1] ? strtof(argv[1],NULL) : 0.0f;
  assert(fabsf(seed_shift)<=0.49f);
  unsigned t, count=0, last_lr=0, last_rf=0;
  int dc_lr,dc_rf,cps_lr,ix_lr,pwm_lr,cps_rf,ff_rf,ix_rf,pwm_rf;
  float measured_lr=0,measured_rf=0;
  ChassisVelocityCommand command={0,0.3f};
  ChassisWheelTargets targets;
  assert(ChassisKinematics_ToWheelTargets(&command,1600,&targets));
  WheelSpeedController_Init();
  for(unsigned w=0;w<4;w++) ChassisController_ClearPivotWheel((MotorDriverWheel)w);
  puts("t,lr_measured,lr_ff,lr_p,lr_i,lr_pwm,lr_bias,lr_restart,lr_restart_active,lr_restart_age,rf_measured,rf_ff,rf_p,rf_i,rf_pwm,rf_load,rf_recovery,rf_overspeed_ms,lr_enabled,lr_temporary,lr_under_ms,lr_episode_ms,lr_active,lr_spent");
  while(scanf("%u %d %d %d %d %d %d %d %d %d",&t,&dc_lr,&dc_rf,&cps_lr,
      &ix_lr,&pwm_lr,&cps_rf,&ff_rf,&ix_rf,&pwm_rf)==10)
  {
    const float target_lr=targets.counts_per_second[MOTOR_DRIVER_WHEEL_LR];
    const float target_rf=targets.counts_per_second[MOTOR_DRIVER_WHEEL_RF];
    if(count==0)
    {
      measured_lr=midpoint(cps_lr,1)+seed_shift;
      measured_rf=midpoint(cps_rf,1)+seed_shift;
      s_states[MOTOR_DRIVER_WHEEL_LR].integrator_permille=midpoint(ix_lr,.1f)+seed_shift*.1f;
      s_states[MOTOR_DRIVER_WHEEL_RF].integrator_permille=midpoint(ix_rf,.1f)+seed_shift*.1f;
      s_states[MOTOR_DRIVER_WHEEL_LR].feedforward_permille=
        WheelSpeedController_GetFeedforward(MOTOR_DRIVER_WHEEL_LR,target_lr);
      s_states[MOTOR_DRIVER_WHEEL_RF].feedforward_permille=midpoint(ff_rf,.1f)+seed_shift*.1f;
      s_states[MOTOR_DRIVER_WHEEL_LR].command_permille=(int16_t)pwm_lr;
      s_states[MOTOR_DRIVER_WHEEL_RF].command_permille=(int16_t)pwm_rf;
      s_previous_output[MOTOR_DRIVER_WHEEL_LR]=(float)pwm_lr;
      s_previous_output[MOTOR_DRIVER_WHEEL_RF]=(float)pwm_rf;
      s_previous_target[MOTOR_DRIVER_WHEEL_LR]=target_lr;
      s_previous_target[MOTOR_DRIVER_WHEEL_RF]=target_rf;
      /* Initial LR total bias is zero in the printed FF; old startup history
       * is not observable. Neither choice of a finished restart changes this
       * zero-bias pre-event branch. Subsequent near-rest entry is new. */
      s_pivot_breakaway_sign[MOTOR_DRIVER_WHEEL_LR]=-1;
      s_pivot_has_moved[MOTOR_DRIVER_WHEEL_LR]=true;
      s_lr_running.moving_ms=40;
      /* Seeded ring is already moving with zero legacy bias. Selection will
       * occur on its next update; no private startup timeline is invented. */
      s_front_pivot[MOTOR_DRIVER_WHEEL_RF]=(WheelSpeedFrontPivotState){
        .active=true,.sign=1,.moving_ms=40,
        .load_permille=s_states[MOTOR_DRIVER_WHEEL_RF].feedforward_permille-
          WheelSpeedController_GetFeedforward(MOTOR_DRIVER_WHEEL_RF,target_rf)};
      last_lr=last_rf=t;
    }
    else
    {
      measured_lr+=(20.0f/70.0f)*(dc_lr*50.0f-measured_lr);
      measured_rf+=(20.0f/70.0f)*(dc_rf*50.0f-measured_rf);
      if(dc_lr) last_lr=t; else if(t-last_lr>=200) measured_lr=0;
      if(dc_rf) last_rf=t; else if(t-last_rf>=200) measured_rf=0;
      float bias_lr=ChassisController_GetFeedforwardBias(&command,MOTOR_DRIVER_WHEEL_LR,
        target_lr,measured_lr,t,20);
      int16_t result;
      assert(WheelSpeedController_UpdateWithMode(MOTOR_DRIVER_WHEEL_LR,target_lr,
        measured_lr,bias_lr,WHEEL_SPEED_CONTROL_MODE_PIVOT,20,&result));
      assert(WheelSpeedController_UpdateWithMode(MOTOR_DRIVER_WHEEL_RF,target_rf,
        measured_rf,0,WHEEL_SPEED_CONTROL_MODE_PIVOT,20,&result));
    }
    const WheelSpeedControllerState *lr=&s_states[MOTOR_DRIVER_WHEEL_LR];
    const WheelSpeedControllerState *rf=&s_states[MOTOR_DRIVER_WHEEL_RF];
    const WheelSpeedFrontPivotState *front=&s_front_pivot[MOTOR_DRIVER_WHEEL_RF];
    printf("%u,%.9f,%.9f,%.9f,%.9f,%d,%.9f,%d,%d,%u,%.9f,%.9f,%.9f,%.9f,%d,%.9f,%.9f,%u,%d,%.9f,%u,%u,%d,%d\n",
      t,measured_lr,lr->feedforward_permille,lr->proportional_permille,
      lr->integrator_permille,lr->command_permille,
      s_pivot_applied_bias[MOTOR_DRIVER_WHEEL_LR],s_pivot_rear_restart[MOTOR_DRIVER_WHEEL_LR],
      s_pivot_breakaway_active[MOTOR_DRIVER_WHEEL_LR],
      s_pivot_rear_restart[MOTOR_DRIVER_WHEEL_LR] ? t-s_pivot_breakaway_start_ms[MOTOR_DRIVER_WHEEL_LR] : 0,
      measured_rf,rf->feedforward_permille,rf->proportional_permille,
      rf->integrator_permille,rf->command_permille,front->load_permille,
      front->recovery_permille,front->load_overspeed_ms,
      s_lr_running.enabled,s_lr_running.temporary_permille,s_lr_running.under_ms,
      s_lr_running.episode_ms,s_lr_running.active,s_lr_running.spent);
    ++count;
  }
  assert(count==127);
  return 0;
}
