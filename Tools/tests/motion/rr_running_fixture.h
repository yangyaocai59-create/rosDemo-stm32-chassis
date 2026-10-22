/* Real controller/runtime, mocked peripherals; never hardware. */
#ifdef REFERENCE
#include "../../../docs/stm32_evidence/2026-10-22-rr-running-prototype-v1/fixture.h"
#else
#ifdef BASELINE
#include "../../../docs/stm32_evidence/2026-10-22-rr-running-prototype-v1/baseline/wheel_speed_control_config.h"
#include "../../../docs/stm32_evidence/2026-10-22-rr-running-prototype-v1/baseline/wheel_speed_controller.h"
#else
#include "wheel_speed_control_config.h"
#include "wheel_speed_controller.h"
#endif
#define main ExistingRuntimeRegressionMain
#ifdef BASELINE
#include "../../../docs/stm32_evidence/2026-10-22-rr-running-prototype-v1/baseline/test_runtime_motion.c"
#else
#include "test_runtime_motion.c"
#endif
#undef main
#ifdef BASELINE
#include "../../../docs/stm32_evidence/2026-10-22-rr-running-prototype-v1/baseline/chassis_controller.c"
#include "../../../docs/stm32_evidence/2026-10-22-rr-running-prototype-v1/baseline/wheel_speed_controller.c"
#else
#include "../../../Application/Chassis/chassis_controller.c"
#include "../../../Application/Chassis/wheel_speed_controller.c"
#endif
#endif
