/* Host-only probe of real controller code, prescribed feedback, no plant,
 * GPIO, ports, firmware build, safety-state clear, or hardware motion. */
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include "wheel_speed_controller.h"
#include "wheel_speed_control_config.h"

static WheelSpeedControllerState Step(MotorDriverWheel wheel, float sign,
                                      float measured_magnitude)
{
  int16_t pwm = 0;
  WheelSpeedControllerState state;
  assert(WheelSpeedController_UpdateWithMode(wheel, sign * 160.0f,
    sign * measured_magnitude, 0.0f, WHEEL_SPEED_CONTROL_MODE_PIVOT, 20, &pwm));
  assert(WheelSpeedController_GetState(wheel, &state));
  assert(abs(pwm) <= WHEEL_SPEED_CONTROL_OUTPUT_LIMIT_PERMILLE);
  return state;
}

static float Support(WheelSpeedControllerState state, float sign)
{
  return sign * (state.feedforward_permille + state.integrator_permille);
}

int main(void)
{
  const MotorDriverWheel fronts[] = {MOTOR_DRIVER_WHEEL_LF, MOTOR_DRIVER_WHEEL_RF};
  for (unsigned w = 0; w < 2; ++w)
  {
    for (int sign = -1; sign <= 1; sign += 2)
    {
      const MotorDriverWheel wheel = fronts[w];
      assert(WheelSpeedController_Reset(wheel));
      for (unsigned k = 0; k < 20; ++k) (void)Step(wheel, (float)sign, 160);
      assert(fabsf(WheelSpeedController_ReleaseFeedforwardBias(wheel,
        (float)sign * 10.0f)) < 0.0001f);
      WheelSpeedControllerState baseline = Step(wheel, (float)sign, 160);
      WheelSpeedControllerState fast = Step(wheel, (float)sign, 200);
      const float withdrawal = Support(fast, (float)sign) - Support(baseline, (float)sign);
      assert(fabsf(withdrawal + 4.2f) < 0.002f);
      WheelSpeedControllerState state = fast;
      unsigned restoration_ms = 0;
      for (unsigned k = 1; k <= 19; ++k)
      {
        WheelSpeedControllerState previous = state;
        state = Step(wheel, (float)sign, 115);
        assert(fabsf(Support(state, (float)sign) - Support(previous, (float)sign) - 0.225f) < 0.002f);
        if (!restoration_ms && Support(state, (float)sign) >= Support(baseline, (float)sign))
          restoration_ms = k * 20;
      }
      assert(restoration_ms == 380);
      WheelSpeedFrontPivotState front;
      assert(WheelSpeedController_GetFrontPivotState(wheel, &front));
      assert(!front.recovery_failed && !front.recovering && front.no_motion_ms == 0);
      printf("PASS wheel=%u sign=%d target=160 overspeed=200/20ms "
             "FF_plus_I_delta=%.3f underspeed=115 regain_rate=11.25permille/s "
             "regain_ms=%u no_physical_plant\n", (unsigned)wheel, sign, withdrawal, restoration_ms);
    }
  }
  return 0;
}
