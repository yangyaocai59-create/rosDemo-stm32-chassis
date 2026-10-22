/* Native stub fixture; no real GPIO/encoder/serial. */
#ifdef BASELINE
#include "baseline/wheel_speed_control_config.h"
#include "baseline/wheel_speed_controller.h"
#else
#include "prototype/wheel_speed_control_config.h"
#include "prototype/wheel_speed_controller.h"
#endif
#ifdef LIVE_FAST_STUB
#define WheelSpeedEstimator_GetSnapshot FrozenEstimator_GetSnapshot
#endif
#define main FrozenRuntimeRegressionMain
#include "baseline/test_runtime_motion.c"
#undef main
#ifdef LIVE_FAST_STUB
#undef WheelSpeedEstimator_GetSnapshot
static int32_t fast_delta;
static uint32_t fast_period=20;
static int32_t fast_age;
bool WheelSpeedEstimator_GetSnapshot(WheelSpeedState states[ENCODER_WHEEL_COUNT])
{
  if (!FrozenEstimator_GetSnapshot(states)) return false;
  states[3].delta_counts=fast_delta;
  states[3].sample_period_ms=fast_period;
  states[3].sample_time_ms=time_ms-(uint32_t)fast_age;
  return true;
}
#endif
#ifdef BASELINE
#include "baseline/chassis_controller.c"
#include "baseline/wheel_speed_controller.c"
#else
#include "prototype/chassis_controller.c"
#include "prototype/wheel_speed_controller.c"
#endif
