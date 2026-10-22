/* Diagnostic only. Frozen production C is unchanged. Recorded feedback and
 * inferred anchor state are prescribed, not a motor/ground simulation. */
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include "source_snapshot/wheel_speed_controller.c"
#include "source_snapshot/chassis_config.h"

static void emit(unsigned t, MotorDriverWheel w, float measured)
{
  const WheelSpeedControllerState *c = &s_states[w];
  const WheelSpeedFrontPivotState *f = &s_front_pivot[w];
  printf("%u,%.9f,%.9f,%.9f,%.9f,%d,%.9f,%.9f,%u,%u,%u,%d,%d\n",
    t, measured, c->feedforward_permille, c->proportional_permille,
    c->integrator_permille, c->command_permille, f->load_permille,
    f->recovery_permille, f->still_ms, f->moving_ms,
    f->load_overspeed_ms, f->recovering, f->recovery_failed);
}

int main(void)
{
  unsigned index, t;
  float integral, measured, bias;
  assert(scanf("%u %u %f %f %f", &index, &t, &integral, &measured, &bias) == 5);
  assert(index == MOTOR_DRIVER_WHEEL_RF || index == MOTOR_DRIVER_WHEEL_RR);
  MotorDriverWheel w = (MotorDriverWheel)index;
  const float target = (0.3f * (CHASSIS_WHEEL_TRACK_M * 0.5f)) *
    ((float)CHASSIS_ENCODER_COUNTS_PER_WHEEL_REV / CHASSIS_WHEEL_CIRCUMFERENCE_M);
  WheelSpeedController_Init();
  if (index == MOTOR_DRIVER_WHEEL_RF)
  {
    s_front_pivot[w] = (WheelSpeedFrontPivotState){
      .active = true, .sign = 1, .moving_ms = 40, .load_permille = 0};
  }
  const float ff = WheelSpeedController_GetFeedforward(w, target) + bias;
  const float error = target - measured;
  const float p = (measured > target ? WHEEL_SPEED_CONTROL_PIVOT_OVERSPEED_KP :
    WHEEL_SPEED_CONTROL_KP_PERMILLE_PER_CPS) * error;
  s_states[w] = (WheelSpeedControllerState){
    .target_counts_per_second = target, .measured_counts_per_second = measured,
    .error_counts_per_second = error, .feedforward_permille = ff,
    .proportional_permille = p, .integrator_permille = integral,
    .command_permille = WheelSpeedController_RoundToInt16(ff + p + integral)};
  s_previous_target[w] = target;
  s_previous_output[w] = ff + p + integral;
  emit(t, w, measured);
  while (scanf("%u %f %f", &t, &measured, &bias) == 3)
  {
    int16_t command;
    assert(WheelSpeedController_UpdateWithMode(w, target, measured, bias,
      WHEEL_SPEED_CONTROL_MODE_PIVOT, 20, &command));
    assert(abs(command) <= 300 && fabsf(s_states[w].integrator_permille) <= 50.001f);
    emit(t, w, measured);
  }
  return 0;
}
