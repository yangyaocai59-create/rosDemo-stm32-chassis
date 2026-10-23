/**
 * @file imu_static_test.h
 * @brief Stationary IMU baseline diagnostic shared by UART and USB.
 */

#ifndef APPLICATION_DIAGNOSTICS_IMU_STATIC_TEST_H_
#define APPLICATION_DIAGNOSTICS_IMU_STATIC_TEST_H_

#include <stdbool.h>
#include <stdint.h>

#include "imu_service.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
  IMU_STATIC_RESULT_NOT_RUN = 0,
  IMU_STATIC_RESULT_RUNNING,
  IMU_STATIC_RESULT_PASS,
  IMU_STATIC_RESULT_FAIL_MOVED,
  IMU_STATIC_RESULT_FAIL_BIAS_RANGE,
  IMU_STATIC_RESULT_CANCELLED,
  IMU_STATIC_RESULT_FAIL_DATA
} ImuStaticResult;

typedef enum
{
  IMU_STATIC_PERSIST_NOT_REQUESTED = 0,
  IMU_STATIC_PERSIST_PENDING,
  IMU_STATIC_PERSIST_SAVED,
  IMU_STATIC_PERSIST_FAILED
} ImuStaticPersistState;

typedef enum
{
  IMU_STATIC_SOURCE_NONE = 0,
  IMU_STATIC_SOURCE_UART,
  IMU_STATIC_SOURCE_USB
} ImuStaticSource;

typedef struct
{
  ImuStaticResult result;
  ImuStaticSource source;
  ImuStaticPersistState persistence;
  uint32_t collected_samples;
  uint32_t duration_ms;
} ImuStaticTestStatus;

void ImuStaticTest_Init(void);

/** Start a stationary sample collection unless another owner or motion is active. */
bool ImuStaticTest_Start(uint32_t now_ms,
                         bool motion_active,
                         ImuStaticSource source);

/** Consume at most one new coherent IMU sample. Called only by imuTask. */
void ImuStaticTest_ProcessSample(const ImuServiceSnapshot *snapshot,
                                 uint32_t now_ms);

/** Cancel an unfinished collection without changing IMU calibration. */
void ImuStaticTest_Cancel(void);

/** Cancel only when the unfinished collection belongs to the given source. */
bool ImuStaticTest_CancelIfSource(ImuStaticSource source);

bool ImuStaticTest_IsActive(void);

/** Copy the coherent result/source state used by USB telemetry. */
bool ImuStaticTest_GetStatus(ImuStaticTestStatus *status);

#ifdef __cplusplus
}
#endif

#endif /* APPLICATION_DIAGNOSTICS_IMU_STATIC_TEST_H_ */
