/* Diagnostic only. The frozen production .c is included UNMODIFIED so an
 * inferred initial state can be seeded in this translation unit. It is not
 * Flash readback, new private-state telemetry, a motor model or new firmware.
 * No GPIO/ports/RTOS code is linked. */
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "source_snapshot/wheel_speed_controller.c"

static void Emit(unsigned time, MotorDriverWheel wheel, float measured)
{
  const WheelSpeedControllerState *s = &s_states[wheel];
  const WheelSpeedFrontPivotState *f = &s_front_pivot[wheel];
  const int hold = f->still_ms >= WHEEL_SPEED_FRONT_RECOVERY_CONFIRM_MS ||
                  f->recovering || f->recovery_permille > 0;
  printf("%u,%.8f,%.8f,%.8f,%.8f,%d,%.8f,%.8f,%u,%u,%u,%u,%u,%d,%d,%d,%d\n",
    time, measured, s->feedforward_permille, s->proportional_permille,
    s->integrator_permille, s->command_permille, f->load_permille,
    f->recovery_permille, (unsigned)f->still_ms, (unsigned)f->moving_ms,
    (unsigned)f->load_overspeed_ms, (unsigned)f->recovery_ms,
    (unsigned)f->no_motion_ms, f->recovering, f->exhausted,
    f->recovery_failed, hold);
}

static void Step(unsigned time, MotorDriverWheel wheel, float target, float measured)
{
  int16_t command;
  assert(WheelSpeedController_UpdateWithMode(wheel, target, measured, 0,
    WHEEL_SPEED_CONTROL_MODE_PIVOT, 20, &command));
  assert(abs(command) <= 300);
  assert(fabsf(s_states[wheel].integrator_permille) <= 50.001f);
  Emit(time, wheel, measured);
}

static void Synthetic(void)
{
  const MotorDriverWheel wheels[] = {MOTOR_DRIVER_WHEEL_LF, MOTOR_DRIVER_WHEEL_RF};
  for (unsigned w = 0; w < 2; ++w)
    for (int sign = -1; sign <= 1; sign += 2)
    {
      const MotorDriverWheel wheel = wheels[w];
      const float target = (float)sign*160;
      WheelSpeedController_Init();
      int16_t command;
      for (unsigned k = 0; k < 20; ++k)
        assert(WheelSpeedController_UpdateWithMode(wheel, target, target, 0,
          WHEEL_SPEED_CONTROL_MODE_PIVOT, 20, &command));
      assert(fabsf(WheelSpeedController_ReleaseFeedforwardBias(wheel,(float)sign*20)) < 0.001f);
      assert(WheelSpeedController_UpdateWithMode(wheel, target, target, 0,
        WHEEL_SPEED_CONTROL_MODE_PIVOT, 20, &command));
      const float before = (float)sign*(s_states[wheel].feedforward_permille+s_states[wheel].integrator_permille);
      printf("CASE wheel=%u sign=%d target=160 preexisting_I=20 prescribed_feedback_only\n", (unsigned)wheel, sign);
      for (unsigned t=20; t<=160; t+=20) Step(t,wheel,target,0);
      const float peak = (float)sign*(s_states[wheel].feedforward_permille+s_states[wheel].integrator_permille);
      assert(s_front_pivot[wheel].recovery_permille == 50 && s_states[wheel].integrator_permille == 0);
      for (unsigned t=180; t<=320; t+=20) Step(t,wheel,target,(float)sign*100);
      const float final = (float)sign*(s_states[wheel].feedforward_permille+s_states[wheel].integrator_permille);
      assert(!s_front_pivot[wheel].recovering && s_front_pivot[wheel].recovery_permille == 0);
      assert(fabsf(s_states[wheel].integrator_permille-(float)sign*0.3f)<0.001f);
      assert(fabsf((peak-final)-49.7f)<0.002f && final<before);
      assert(s_front_pivot[wheel].load_overspeed_ms == 0 && !s_front_pivot[wheel].recovery_failed);
      printf("RESULT pre_near_rest_FF_plus_I=%.5f peak=%.5f after_160ms_accounted_moving_at_100cps=%.5f support_below_pre_rest=%.5f\n",
        before,peak,final,before-final);
    }
}

int main(int argc, char **argv)
{
  if (argc == 2 && strcmp(argv[1], "synthetic") == 0)
  { Synthetic(); return 0; }
  unsigned index, time;
  float target, load, integral, measured;
  assert(scanf("%u %u %f %f %f %f", &index, &time, &target, &load, &integral, &measured) == 6);
  assert(index == MOTOR_DRIVER_WHEEL_LF || index == MOTOR_DRIVER_WHEEL_RF);
  const MotorDriverWheel wheel = (MotorDriverWheel)index;
  const int sign = target > 0 ? 1 : -1;
  WheelSpeedController_Init();
  s_front_pivot[wheel] = (WheelSpeedFrontPivotState){.load_permille=load,
    .moving_ms=40, .sign=(int8_t)sign, .active=true};
  const float error = target-measured;
  const float ff = WheelSpeedController_GetFeedforward(wheel,target)+(float)sign*load;
  const float p = WHEEL_SPEED_CONTROL_KP_PERMILLE_PER_CPS*error;
  const float output = ff+p+integral;
  s_states[wheel] = (WheelSpeedControllerState){.target_counts_per_second=target,
    .measured_counts_per_second=measured, .error_counts_per_second=error,
    .feedforward_permille=ff, .proportional_permille=p, .integrator_permille=integral,
    .command_permille=WheelSpeedController_RoundToInt16(output)};
  s_previous_target[wheel]=target;
  s_previous_output[wheel]=output;
  Emit(time,wheel,measured);
  while (scanf("%u %f", &time, &measured) == 2) Step(time,wheel,target,measured);
  return 0;
}
