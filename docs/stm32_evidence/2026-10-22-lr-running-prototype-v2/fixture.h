/* Host peripheral stubs and existing runtime helpers; no hardware access. */
#ifdef CONTROL_V1
#include "../2026-10-22-lr-running-prototype-v1/fixture.h"
#else
#include "prototype/wheel_speed_control_config.h"
#include "prototype/wheel_speed_controller.h"
#define main ExistingRuntimeRegressionMain
#include "../../../Tools/tests/motion/test_runtime_motion.c"
#undef main
#include "prototype/chassis_controller.c"
#include "prototype/wheel_speed_controller.c"
#endif
