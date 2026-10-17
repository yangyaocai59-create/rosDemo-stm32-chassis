#ifndef APPLICATION_DIAGNOSTICS_IMU_CALIBRATION_GUARD_H_
#define APPLICATION_DIAGNOSTICS_IMU_CALIBRATION_GUARD_H_
#include <stdbool.h>
#include <stdint.h>

/* Four coherent wheel cumulative counts. No motor commands/writes here.
 * collecting=true permits only the aggregate diagnostic inhibit owned by
 * this calibration itself; actual motor diagnostics remain forbidden. */
bool ImuCalibrationGuard_Read(int64_t wheel_counts[4], bool collecting);
#endif
