/* Real production code with host peripherals. Never accesses hardware. */
#include "wheel_speed_control_config.h"
#include "wheel_speed_controller.h"
#define main ExistingRuntimeRegressionMain
#include "test_runtime_motion.c"
#undef main
#include "../../../Application/Chassis/chassis_controller.c"
#include "../../../Application/Chassis/wheel_speed_controller.c"
