/** Low-frequency, identity-only SPI test; not a calibrated IMU backend. */
#ifndef APPLICATION_DIAGNOSTICS_IMU_SPI_IDENTITY_TEST_H_
#define APPLICATION_DIAGNOSTICS_IMU_SPI_IDENTITY_TEST_H_

#include <stdbool.h>
#include <stdint.h>

typedef struct
{
  bool initialized;
  bool identity_stable;
  uint8_t who_am_i;
  uint8_t consecutive_matches;
  uint32_t read_attempts;
  uint32_t read_errors;
  uint32_t unknown_reads;
  uint32_t sample_time_ms;
  uint32_t hal_status;
} ImuSpiIdentitySnapshot;

void ImuSpiIdentityTest_Init(uint32_t now_ms);
void ImuSpiIdentityTest_Update(uint32_t now_ms);
bool ImuSpiIdentityTest_GetSnapshot(ImuSpiIdentitySnapshot *snapshot);

#endif
