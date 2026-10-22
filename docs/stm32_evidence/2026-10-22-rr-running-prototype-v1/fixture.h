/* Native peripheral stubs only. No serial or hardware operations. */
#ifdef BASELINE
#include "baseline/wheel_speed_control_config.h"
#include "baseline/wheel_speed_controller.h"
#else
#include "prototype/wheel_speed_control_config.h"
#include "prototype/wheel_speed_controller.h"
#endif
#define main ExistingRuntimeRegressionMain
#include "baseline/test_runtime_motion.c"
#undef main
#ifdef BASELINE
#include "baseline/chassis_controller.c"
#include "baseline/wheel_speed_controller.c"
#else
#include "prototype/chassis_controller.c"
#include "prototype/wheel_speed_controller.c"
#endif
