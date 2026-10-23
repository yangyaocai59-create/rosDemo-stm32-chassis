/**
 * @brief One read-only WHO_AM_I transaction per second from imuTask.
 * No sensor register writes, valid motion samples, bias restore or Flash save.
 */
#include "imu_spi_identity_test.h"

#include <stddef.h>
#include <string.h>
#include "FreeRTOS.h"
#include "task.h"
#include "debug_log.h"
#include "main.h"
#include "mpu_spi_probe.h"

#define IDENTITY_INITIAL_WAIT_MS 100U
#define IDENTITY_READ_PERIOD_MS 1000U
#define IDENTITY_LOG_PERIOD_MS 5000U
#define IDENTITY_CONFIRM_READS 3U

static ImuSpiIdentitySnapshot s_snapshot;
static uint32_t s_next_read_ms;
static uint32_t s_next_log_ms;

static bool TimeReached(uint32_t now_ms, uint32_t deadline_ms)
{
  return (int32_t)(now_ms - deadline_ms) >= 0;
}

void ImuSpiIdentityTest_Init(uint32_t now_ms)
{
  taskENTER_CRITICAL();
  memset(&s_snapshot, 0, sizeof(s_snapshot));
  s_snapshot.initialized = true;
  taskEXIT_CRITICAL();
  s_next_read_ms = now_ms + IDENTITY_INITIAL_WAIT_MS;
  s_next_log_ms = now_ms;
  DEBUG_LOG_WARN("SPI IMU identity-only test: I2C6050 inactive; no six-axis data, calibration or Flash writes");
  DEBUG_LOG_INFO("SPI IMU probe: SPI1 mode0 / 656kHz; PA4=CS PA5=SCK PB4=MISO PB5=MOSI PB0=INT");
}

void ImuSpiIdentityTest_Update(uint32_t now_ms)
{
  uint8_t who_am_i = 0U;
  HAL_StatusTypeDef status;
  ImuSpiIdentitySnapshot previous;
  ImuSpiIdentitySnapshot current;
  bool known;

  if (!s_snapshot.initialized || !TimeReached(now_ms, s_next_read_ms))
  {
    return;
  }
  /* Deadline based on this attempt: no burst catch-up after a task delay. */
  s_next_read_ms = now_ms + IDENTITY_READ_PERIOD_MS;
  (void)ImuSpiIdentityTest_GetSnapshot(&previous);
  status = MpuSpiProbe_ReadWhoAmI(&who_am_i);
  known = (status == HAL_OK) && MpuSpiProbe_IsKnownIdentity(who_am_i);
  current = previous;
  ++current.read_attempts;
  current.sample_time_ms = now_ms;
  current.hal_status = (uint32_t)status;
  current.who_am_i = who_am_i;
  if (status != HAL_OK)
  {
    ++current.read_errors;
  }
  else if (!known)
  {
    ++current.unknown_reads;
  }
  if (known)
  {
    if ((previous.hal_status == (uint32_t)HAL_OK) &&
        (previous.who_am_i == who_am_i) &&
        (previous.consecutive_matches != 0U))
    {
      if (current.consecutive_matches < IDENTITY_CONFIRM_READS)
      {
        ++current.consecutive_matches;
      }
    }
    else
    {
      current.consecutive_matches = 1U;
    }
  }
  else
  {
    current.consecutive_matches = 0U;
  }
  current.identity_stable =
    known && (current.consecutive_matches >= IDENTITY_CONFIRM_READS);
  taskENTER_CRITICAL();
  s_snapshot = current;
  taskEXIT_CRITICAL();

  if ((current.read_attempts <= IDENTITY_CONFIRM_READS) ||
      (previous.identity_stable != current.identity_stable) ||
      (previous.who_am_i != current.who_am_i) ||
      (previous.hal_status != current.hal_status) ||
      TimeReached(now_ms, s_next_log_ms))
  {
    DebugLog_Printf(current.identity_stable ? DEBUG_LOG_LEVEL_INFO : DEBUG_LOG_LEVEL_WARN,
      "SPI IMU ID: WHO_AM_I=0x%02X candidate=%s HAL=%s streak=%u/3 stable=%u reads=%lu errors=%lu unknown=%lu CS=%s INT=%u",
      (unsigned int)who_am_i, MpuSpiProbe_CandidateName(who_am_i),
      MpuSpiProbe_HalStatusName(status), (unsigned int)current.consecutive_matches,
      current.identity_stable ? 1U : 0U, (unsigned long)current.read_attempts,
      (unsigned long)current.read_errors, (unsigned long)current.unknown_reads,
      HAL_GPIO_ReadPin(MPU6500_CS_GPIO_Port, MPU6500_CS_Pin) == GPIO_PIN_SET ? "HIGH" : "LOW",
      HAL_GPIO_ReadPin(IMU_INT_GPIO_Port, IMU_INT_Pin) == GPIO_PIN_SET ? 1U : 0U);
    s_next_log_ms = now_ms + IDENTITY_LOG_PERIOD_MS;
  }
}

bool ImuSpiIdentityTest_GetSnapshot(ImuSpiIdentitySnapshot *snapshot)
{
  if (snapshot == NULL)
  {
    return false;
  }
  taskENTER_CRITICAL();
  *snapshot = s_snapshot;
  taskEXIT_CRITICAL();
  return true;
}
