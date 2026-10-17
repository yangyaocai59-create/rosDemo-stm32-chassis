/* Actual production probe, identity state machine and calibration gate.
 * HAL/RTOS/logging/legacy sensor/storage are mocked; no hardware is accessed. */
#include <assert.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include "main.h"
#include "spi.h"
#include "debug_log.h"
#include "mpu_spi_probe.h"
#include "imu_spi_identity_test.h"
#include "imu_service.h"
#include "imu_static_test.h"
#include "mpu6050.h"
#include "imu_calibration_guard.h"

GPIO_TypeDef test_gpio_a, test_gpio_b;
SPI_HandleTypeDef hspi1;
static GPIO_PinState cs = GPIO_PIN_SET;
static HAL_StatusTypeDef next_status = HAL_OK;
static uint8_t next_identity = 0x70U;
static uint32_t spi_calls, gpio_calls, legacy_calls, flash_calls;
static unsigned critical_depth, log_calls;
static DebugLog_Level last_level;
static char last_log[512];

void Test_EnterCritical(void) { ++critical_depth; }
void Test_ExitCritical(void) { assert(critical_depth); --critical_depth; }
uint32_t HAL_GetTick(void) { return 0U; }
bool ImuCalibrationGuard_Read(int64_t counts[4], bool collecting)
{ (void)collecting; (void)counts; assert(false); return false; }
void HAL_GPIO_WritePin(GPIO_TypeDef *port, uint16_t pin, GPIO_PinState value)
{
  assert(port == MPU6500_CS_GPIO_Port && pin == MPU6500_CS_Pin);
  assert(!critical_depth);
  assert((gpio_calls % 2U == 0U) ? value == GPIO_PIN_RESET : value == GPIO_PIN_SET);
  cs = value;
  ++gpio_calls;
}
GPIO_PinState HAL_GPIO_ReadPin(GPIO_TypeDef *port, uint16_t pin)
{
  assert(!critical_depth);
  if (port == MPU6500_CS_GPIO_Port && pin == MPU6500_CS_Pin) return cs;
  assert(port == IMU_INT_GPIO_Port && pin == IMU_INT_Pin);
  return GPIO_PIN_RESET;
}
HAL_StatusTypeDef HAL_SPI_TransmitReceive(SPI_HandleTypeDef *handle, uint8_t *tx,
                                        uint8_t *rx, uint16_t length, uint32_t timeout)
{
  assert(!critical_depth && cs == GPIO_PIN_RESET);
  assert(handle == &hspi1 && length == 2U && timeout == 5U);
  assert(tx[0] == 0xF5U && tx[1] == 0U); /* Read only, register 0x75. */
  rx[0] = 0xAAU; /* Command phase must be ignored. */
  rx[1] = next_identity;
  ++spi_calls;
  return next_status;
}
bool DebugLog_Printf(DebugLog_Level level, const char *format, ...)
{
  va_list args;
  assert(!critical_depth);
  last_level = level;
  va_start(args, format);
  (void)vsnprintf(last_log, sizeof(last_log), format, args);
  va_end(args);
  ++log_calls;
  return true;
}
/* Must never be used by identity-only operations or calibration rejection. */
bool Mpu6050_ReadWhoAmI(uint8_t *who) { (void)who; ++legacy_calls; return false; }
bool Mpu6050_BeginReset(void) { ++legacy_calls; return false; }
bool Mpu6050_Configure(void) { ++legacy_calls; return false; }
bool Mpu6050_VerifyConfiguration(void) { ++legacy_calls; return false; }
bool Mpu6050_ReadRaw(Mpu6050RawSample *raw) { (void)raw; ++legacy_calls; return false; }
bool ImuBiasStorage_SaveMilliRaw(int32_t x, int32_t y, int32_t z, int16_t temp)
{ (void)x; (void)y; (void)z; (void)temp; ++flash_calls; return false; }

static ImuSpiIdentitySnapshot Snapshot(void)
{
  ImuSpiIdentitySnapshot value;
  assert(ImuSpiIdentityTest_GetSnapshot(&value));
  assert(!critical_depth);
  return value;
}
static void VerifyNoDataOrCalibration(uint32_t now)
{
  ImuServiceSnapshot imu;
  assert(ImuService_GetSnapshot(&imu));
  assert(!imu.initialized && !imu.valid && !imu.gyro_bias_valid && !imu.sample_count);
  assert(!ImuStaticTest_Start(now, false, IMU_STATIC_SOURCE_USB));
  assert(!ImuStaticTest_Start(now, false, IMU_STATIC_SOURCE_UART));
  ImuStaticTest_ProcessSample(&imu, now);
  assert(!ImuStaticTest_IsActive() && !legacy_calls && !flash_calls);
}

int main(void)
{
  uint8_t identity = 0xFFU;
  assert(!ImuSpiIdentityTest_GetSnapshot(NULL));
  ImuSpiIdentityTest_Update(10000U);
  assert(!spi_calls);
  assert(MpuSpiProbe_ReadWhoAmI(NULL) == HAL_ERROR && !spi_calls && !gpio_calls);
  for (unsigned i = 0; i < 4U; ++i)
  {
    next_status = (HAL_StatusTypeDef)i;
    identity = 0xFFU;
    assert(MpuSpiProbe_ReadWhoAmI(&identity) == next_status);
    assert(cs == GPIO_PIN_SET && gpio_calls == 2U*spi_calls);
    assert(identity == (next_status == HAL_OK ? 0x70U : 0U));
  }
  assert(strcmp(MpuSpiProbe_CandidateName(0x70), "MPU6500") == 0);
  assert(strcmp(MpuSpiProbe_CandidateName(0x71), "MPU9250") == 0);
  assert(strcmp(MpuSpiProbe_CandidateName(0x73), "MPU9255") == 0);
  assert(!MpuSpiProbe_IsKnownIdentity(0) && !MpuSpiProbe_IsKnownIdentity(0xFF));
  assert(!MpuSpiProbe_IsKnownIdentity(0x68));
  ImuStaticTest_Init();
  next_status = HAL_OK;
  ImuSpiIdentityTest_Init(100U);
  unsigned before = spi_calls;
  ImuSpiIdentityTest_Update(199U);
  assert(spi_calls == before);
  ImuSpiIdentityTest_Update(200U);
  assert(Snapshot().consecutive_matches == 1 && !Snapshot().identity_stable);
  ImuSpiIdentityTest_Update(1199U);
  assert(spi_calls == before+1);
  ImuSpiIdentityTest_Update(1200U);
  assert(Snapshot().consecutive_matches == 2 && !Snapshot().identity_stable);
  ImuSpiIdentityTest_Update(2200U);
  assert(Snapshot().identity_stable && Snapshot().consecutive_matches == 3);
  assert(last_level == DEBUG_LOG_LEVEL_INFO && strstr(last_log,"stable=1"));
  VerifyNoDataOrCalibration(2200U);
  unsigned logs_before = log_calls;
  ImuSpiIdentityTest_Update(3200U);
  assert(log_calls == logs_before && Snapshot().consecutive_matches == 3);
  next_status = HAL_TIMEOUT;
  ImuSpiIdentityTest_Update(4200U);
  assert(!Snapshot().identity_stable && Snapshot().read_errors == 1);
  assert(Snapshot().consecutive_matches == 0 && cs == GPIO_PIN_SET);
  next_status = HAL_OK;
  ImuSpiIdentityTest_Update(5200U);
  assert(Snapshot().consecutive_matches == 1 && !Snapshot().identity_stable);
  next_identity = 0x71U;
  ImuSpiIdentityTest_Update(6200U);
  assert(Snapshot().consecutive_matches == 1 && !Snapshot().identity_stable);
  ImuSpiIdentityTest_Update(7200U);
  ImuSpiIdentityTest_Update(8200U);
  assert(Snapshot().identity_stable && Snapshot().who_am_i == 0x71);
  next_identity = 0xFFU;
  ImuSpiIdentityTest_Update(9200U);
  assert(!Snapshot().identity_stable && Snapshot().unknown_reads == 1);
  next_identity = 0U;
  ImuSpiIdentityTest_Update(10200U);
  assert(!Snapshot().identity_stable && Snapshot().unknown_reads == 2);
  next_identity = 0x73U;
  ImuSpiIdentityTest_Update(11200U);
  ImuSpiIdentityTest_Update(12200U);
  ImuSpiIdentityTest_Update(13200U);
  assert(Snapshot().identity_stable && Snapshot().who_am_i == 0x73);
  VerifyNoDataOrCalibration(13200U);
  /* Wrap-safe deadlines and no catch-up bursts after late scheduling. */
  ImuSpiIdentityTest_Init(0xFFFFFFD0U);
  before = spi_calls;
  ImuSpiIdentityTest_Update(0x33U);
  assert(spi_calls == before);
  ImuSpiIdentityTest_Update(0x34U);
  assert(spi_calls == before+1);
  ImuSpiIdentityTest_Update(50000U);
  assert(spi_calls == before+2);
  ImuSpiIdentityTest_Update(50000U);
  assert(spi_calls == before+2);
  assert(!legacy_calls && !flash_calls && !critical_depth && cs == GPIO_PIN_SET);
  puts("PASS: production SPI read command, CS ownership/release, HAL OK/ERROR/BUSY/TIMEOUT");
  puts("PASS: 3 consecutive identities, model change, unknown/00/FF rejection, loss/recovery, tick wrap and bounded retry");
  puts("PASS: production IMU snapshot stays invalid; USB/UART calibration rejected, no I2C/bias Flash calls");
  return 0;
}
