/* Production controller, prescribed feedback only; never opens hardware. */
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include "wheel_speed_controller.h"
#include "wheel_speed_control_config.h"

static WheelSpeedControllerState Step(MotorDriverWheel wheel, float target,
                                     float measured, uint32_t dt)
{
  int16_t pwm;
  WheelSpeedControllerState before, after;
  WheelSpeedFrontPivotState front;
  assert(WheelSpeedController_GetState(wheel, &before));
  assert(WheelSpeedController_UpdateWithMode(wheel, target, measured, 0,
    WHEEL_SPEED_CONTROL_MODE_PIVOT, dt, &pwm));
  assert(WheelSpeedController_GetState(wheel, &after));
  assert(WheelSpeedController_GetFrontPivotState(wheel, &front));
  assert(abs(pwm) <= 300 && fabsf(after.integrator_permille) <= 50.001f);
  assert(abs(pwm - before.command_permille) <= (int)dt + 1);
  assert(front.load_overspeed_ms <= 60 && front.no_motion_ms <= 1500);
  return after;
}

static float Setup(MotorDriverWheel wheel, float sign, float magnitude)
{
  assert(WheelSpeedController_Reset(wheel));
  for (unsigned k = 0; k < 20; ++k)
    (void)Step(wheel, sign * magnitude, sign * magnitude, 20);
  assert(fabsf(WheelSpeedController_ReleaseFeedforwardBias(wheel, sign * 10)) < 0.001f);
  return fabsf(WheelSpeedController_GetPivotLoadFeedforward(wheel, sign * magnitude));
}

static void TestPulsesAndSustained(MotorDriverWheel wheel, float sign)
{
  const float magnitudes[] = {20, 40, 160, 280, 1600};
  const uint32_t pulses[] = {1, 10, 20, 40, 59};
  const uint32_t periods[] = {1, 10, 20, 25, 59, 60, 100};
  for (unsigned m = 0; m < sizeof(magnitudes)/sizeof(magnitudes[0]); ++m)
  {
    const float magnitude = magnitudes[m];
    const float band = fmaxf(20, magnitude * 0.1f);
    const float fast = magnitude + band + 20;
    for (unsigned p = 0; p < sizeof(pulses)/sizeof(pulses[0]); ++p)
    {
      const float ceiling = Setup(wheel, sign, magnitude);
      WheelSpeedControllerState before;
      assert(WheelSpeedController_GetState(wheel, &before));
      const uint32_t dt = pulses[p];
      WheelSpeedControllerState state = Step(wheel, sign*magnitude, sign*fast, dt);
      WheelSpeedFrontPivotState front;
      assert(WheelSpeedController_GetFrontPivotState(wheel, &front));
      assert(front.load_overspeed_ms == dt && fabsf(front.load_permille - ceiling) < 0.001f);
      /* P damping and error I already act on the FIRST short speed excursion. */
      assert(fabsf(state.proportional_permille*sign + 0.15f*(fast-magnitude)) < 0.002f);
      assert(state.integrator_permille*sign < before.integrator_permille*sign);
      (void)Step(wheel, sign*magnitude, sign*magnitude, 20);
      assert(WheelSpeedController_GetFrontPivotState(wheel, &front));
      assert(front.load_overspeed_ms == 0 && fabsf(front.load_permille-ceiling) < 0.001f);
    }
    for (unsigned d = 0; d < sizeof(periods)/sizeof(periods[0]); ++d)
    {
      float expected = Setup(wheel, sign, magnitude);
      const uint32_t dt = periods[d];
      uint32_t elapsed = 0;
      while (elapsed < (wheel==MOTOR_DRIVER_WHEEL_LF ? 6000U : 1000U))
      {
        elapsed += dt;
        (void)Step(wheel, sign*magnitude, sign*fast, dt);
        WheelSpeedFrontPivotState front;
        assert(WheelSpeedController_GetFrontPivotState(wheel, &front));
        const float rate=wheel==MOTOR_DRIVER_WHEEL_LF ? 20.0f : 200.0f;
        if (elapsed >= 60) expected = fmaxf(0, expected - rate*(float)dt/1000.0f);
        assert(front.load_overspeed_ms == (elapsed < 60 ? elapsed : 60));
        assert(fabsf(front.load_permille-expected) < 0.003f);
        assert(!front.recovery_failed && !front.recovering && front.no_motion_ms == 0);
      }
      assert(expected == 0);
    }
  }
}

static void TestBrokenQualification(MotorDriverWheel wheel, float sign)
{
  const float ceiling = Setup(wheel, sign, 160);
  WheelSpeedFrontPivotState front;
  for (unsigned k = 0; k < 30; ++k)
  {
    (void)Step(wheel, sign*160, sign*200, 20);
    (void)Step(wheel, sign*160, sign*200, 20);
    assert(WheelSpeedController_GetFrontPivotState(wheel, &front));
    assert(front.load_overspeed_ms == 40);
    /* Equality is not overspeed: the >target+band comparison is strict. */
    (void)Step(wheel, sign*160, sign*180, 20);
    assert(WheelSpeedController_GetFrontPivotState(wheel, &front));
    assert(front.load_overspeed_ms == 0 && fabsf(front.load_permille-ceiling) < 0.001f);
  }
  WheelSpeedControllerState before, after;
  assert(WheelSpeedController_GetState(wheel, &before));
  after = Step(wheel, sign*160, sign*115, 20);
  assert(WheelSpeedController_GetFrontPivotState(wheel, &front));
  assert(front.load_overspeed_ms == 0);
  assert(fabsf((after.feedforward_permille+after.integrator_permille -
    before.feedforward_permille-before.integrator_permille)*sign - 0.225f) < 0.003f);
}

static void TestRecoveryStillEndsImmediately(MotorDriverWheel wheel, float sign)
{
  const float ceiling = Setup(wheel, sign, 160);
  for (unsigned k = 0; k < 5; ++k) (void)Step(wheel, sign*160, 0, 20);
  WheelSpeedFrontPivotState before, after;
  assert(WheelSpeedController_GetFrontPivotState(wheel, &before));
  assert(before.recovering && before.recovery_permille > 8);
  (void)Step(wheel, sign*160, sign*200, 20);
  assert(WheelSpeedController_GetFrontPivotState(wheel, &after));
  assert(!after.recovering && after.load_overspeed_ms == 20);
  assert(fabsf(after.load_permille-ceiling) < 0.001f);
  assert(fabsf(after.recovery_permille - (before.recovery_permille-8)) < 0.001f);
}

static void TestClearsAndInvalidDt(MotorDriverWheel wheel, float sign)
{
  const float ceiling = Setup(wheel, sign, 160);
  (void)Step(wheel, sign*160, sign*200, 20);
  (void)Step(wheel, sign*160, sign*200, 20);
  WheelSpeedFrontPivotState front;
  int16_t pwm = 1234;
  assert(!WheelSpeedController_UpdateWithMode(wheel, sign*160, sign*200, 0,
    WHEEL_SPEED_CONTROL_MODE_PIVOT, 0, &pwm) && pwm == 1234);
  assert(!WheelSpeedController_UpdateWithMode(wheel, sign*160, sign*200, 0,
    WHEEL_SPEED_CONTROL_MODE_PIVOT, 101, &pwm));
  assert(WheelSpeedController_GetFrontPivotState(wheel, &front));
  assert(front.load_overspeed_ms == 40 && fabsf(front.load_permille-ceiling) < 0.001f);
  (void)Step(wheel, -sign*160, -sign*200, 20);
  assert(WheelSpeedController_GetFrontPivotState(wheel, &front));
  assert(front.sign == (int)-sign && front.load_overspeed_ms == 20);
  (void)Step(wheel, -sign*19, -sign*60, 20);
  assert(WheelSpeedController_GetFrontPivotState(wheel, &front));
  assert(!front.active && front.load_overspeed_ms == 0);
  (void)Step(wheel, sign*160, sign*200, 20);
  assert(WheelSpeedController_Update(wheel, sign*160, sign*200, 20, &pwm));
  assert(WheelSpeedController_GetFrontPivotState(wheel, &front));
  assert(!front.active && front.load_overspeed_ms == 0);
  (void)Step(wheel, sign*160, sign*200, 20);
  (void)Step(wheel, 0, 0, 20);
  assert(WheelSpeedController_GetFrontPivotState(wheel, &front));
  assert(!front.active && front.load_overspeed_ms == 0);
  (void)Step(wheel, sign*160, sign*200, 20);
  assert(WheelSpeedController_Reset(wheel));
  assert(WheelSpeedController_GetFrontPivotState(wheel, &front));
  assert(!front.active && front.load_overspeed_ms == 0);
}

int main(void)
{
  const MotorDriverWheel wheels[] = {MOTOR_DRIVER_WHEEL_LF, MOTOR_DRIVER_WHEEL_RF};
  WheelSpeedController_Init();
  for (unsigned w = 0; w < 2; ++w)
    for (int sign = -1; sign <= 1; sign += 2)
    {
      TestPulsesAndSustained(wheels[w], (float)sign);
      TestBrokenQualification(wheels[w], (float)sign);
      TestRecoveryStillEndsImmediately(wheels[w], (float)sign);
      TestClearsAndInvalidDt(wheels[w], (float)sign);
    }
  puts("PASS: both fronts/signs at 20/40/160/280/1600 cps ignore 1..59ms load-withdrawal pulses while P damping and I unwind act immediately");
  puts("PASS: prototype LF error-scaled and original RF load withdrawal reaches zero after 60ms confirmation, for 1/10/20/25/59/60/100ms steps");
  puts("PASS: separated 40ms excursions never accumulate; exact target+band resets qualification, moving underspeed retains original net integral correction");
  puts("PASS: transient recovery terminates and fades on FIRST overspeed sample, without waiting for load confirmation");
  puts("PASS: sign/mode/low-target/zero/Reset clear qualification; invalid dt never advances state");
  return 0;
}
