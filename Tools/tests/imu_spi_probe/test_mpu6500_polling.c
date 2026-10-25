/* Actual MPU6500 driver, common IMU service and calibration rejection.
 * Register-map HAL/RTOS mocks only; never opens a port or writes real Flash. */
#include <assert.h>
#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include "main.h"
#include "spi.h"
#include "debug_log.h"
#include "mpu6500.h"
#include "imu_service.h"
#include "imu_static_test.h"
#include "imu_calibration_guard.h"
#include "FreeRTOS.h"

GPIO_TypeDef test_gpio_a, test_gpio_b;
SPI_HandleTypeDef hspi1;
static uint8_t regs[128];
static GPIO_PinState cs = GPIO_PIN_SET;
static uint32_t transactions, writes, reads, gpio_calls, flash_calls, clock_ms;
static uint32_t fail_at;
static HAL_StatusTypeDef fail_status = HAL_TIMEOUT;
static unsigned critical_depth;
static bool test_auto_drdy = true;
static bool test_race_during_burst;
static bool test_motion_allowed = true;
static int64_t test_wheel_counts[4];
static unsigned notifications;
static bool test_notify_on_clock;
static bool test_restart_on_guard;
static bool test_user_control_auto_clear;
static uint8_t written_regs[32], written_values[32];

void Test_EnterCritical(void) { ++critical_depth; }
void Test_ExitCritical(void) { assert(critical_depth); --critical_depth; }
TaskHandle_t xTaskGetCurrentTaskHandle(void) { return (void *)(uintptr_t)1U; }
void vTaskNotifyGiveFromISR(TaskHandle_t handle, BaseType_t *woken)
{ assert(handle == (void *)(uintptr_t)1U); ++notifications; *woken = pdFALSE; }
bool ImuCalibrationGuard_Read(int64_t counts[4], bool collecting)
{
  (void)collecting;
  if (test_restart_on_guard)
  {
    test_restart_on_guard=false;
    ImuStaticTest_Cancel();
    assert(ImuStaticTest_Start(clock_ms,false,IMU_STATIC_SOURCE_USB));
  }
  memcpy(counts, test_wheel_counts, sizeof(test_wheel_counts)); return test_motion_allowed;
}
uint32_t HAL_GetTick(void)
{
  assert(!critical_depth);
  if (test_notify_on_clock)
  {
    test_notify_on_clock=false; regs[0x3A]=1U;
    ImuService_OnDataReadyFromISR(clock_ms);
  }
  return clock_ms+1U;
}
void HAL_GPIO_WritePin(GPIO_TypeDef *port, uint16_t pin, GPIO_PinState value)
{
  assert(!critical_depth && port == MPU6500_CS_GPIO_Port && pin == MPU6500_CS_Pin);
  assert(value == (gpio_calls % 2U ? GPIO_PIN_SET : GPIO_PIN_RESET));
  cs = value;
  ++gpio_calls;
}
GPIO_PinState HAL_GPIO_ReadPin(GPIO_TypeDef *port, uint16_t pin)
{
  assert(port == MPU6500_CS_GPIO_Port && pin == MPU6500_CS_Pin);
  return cs;
}
HAL_StatusTypeDef HAL_SPI_TransmitReceive(SPI_HandleTypeDef *handle, uint8_t *tx,
                                        uint8_t *rx, uint16_t length, uint32_t timeout)
{
  assert(!critical_depth && handle == &hspi1 && cs == GPIO_PIN_RESET);
  assert(timeout == 5U && length >= 2U && length <= 15U);
  ++transactions;
  memset(rx, 0xAA, length); /* Command response is not sensor data. */
  if (fail_at && transactions == fail_at) return fail_status;
  const uint8_t reg = tx[0] & 0x7FU;
  if (tx[0] & 0x80U)
  {
    ++reads;
    for (uint16_t i = 1U; i < length; ++i)
    {
      assert(tx[i] == 0U && reg+i-1U < sizeof(regs));
      rx[i] = regs[reg+i-1U];
    }
    if (reg == 0x3AU) { regs[0x3A] = 0U; }
    if (reg == 0x3BU && test_race_during_burst)
    {
      test_race_during_burst = false;
      regs[0x3A] = 1U;
      ImuService_OnDataReadyFromISR(clock_ms + 1U);
    }
  }
  else
  {
    assert(length == 2U);
    if (writes < sizeof(written_regs))
    {
      written_regs[writes] = reg;
      written_values[writes] = tx[1];
    }
    ++writes;
    if (reg == 0x6BU && tx[1] == 0x80U)
    {
      memset(regs, 0, sizeof(regs));
      regs[0x75] = 0x70U;
      regs[0x6B] = 0x40U;
    }
    else regs[reg] = (reg==0x6AU && test_user_control_auto_clear) ?
                      (uint8_t)(tx[1] & ~0x10U) : tx[1];
  }
  return HAL_OK;
}
bool DebugLog_Printf(DebugLog_Level level, const char *format, ...)
{
  (void)level;
  (void)format;
  assert(!critical_depth);
  return true;
}
#ifndef MPU6500_CALIBRATION_INTEGRATION_TEST
bool ImuBiasStorage_SaveMilliRaw(int32_t x, int32_t y, int32_t z, int16_t temp)
{ (void)x; (void)y; (void)z; (void)temp; ++flash_calls; return false; }
#endif

static ImuServiceSnapshot Snapshot(void)
{
  ImuServiceSnapshot value;
  assert(ImuService_GetSnapshot(&value));
  return value;
}
static void ResetMock(void)
{
  memset(regs, 0, sizeof(regs));
  regs[0x75] = 0x70U;
  regs[0x6B] = 0x40U;
  assert(cs == GPIO_PIN_SET && !critical_depth);
  transactions = writes = reads = gpio_calls = fail_at = 0;
  test_auto_drdy = true;
  test_race_during_burst = false;
  test_notify_on_clock = test_restart_on_guard = false;
  test_motion_allowed = true;
  test_user_control_auto_clear=false;
  memset(test_wheel_counts, 0, sizeof(test_wheel_counts));
}
static void Update(uint32_t now)
{
  clock_ms = now;
  if (test_auto_drdy && Snapshot().state == IMU_SERVICE_STATE_READY)
  {
    regs[0x3A] = 1U;
    ImuService_OnDataReadyFromISR(now);
  }
  ImuService_Update(now);
  assert(!critical_depth && cs == GPIO_PIN_SET && gpio_calls == 2U*transactions);
}
static void Put16(uint8_t address, int16_t value)
{
  regs[address] = (uint8_t)((uint16_t)value >> 8);
  regs[address+1] = (uint8_t)value;
}
static void PopulateSample(void)
{
  regs[0x3A] = 1U;
  Put16(0x3B, 8192); Put16(0x3D, -8192); Put16(0x3F, INT16_MAX);
  Put16(0x41, 3339); Put16(0x43, 65); Put16(0x45, -65); Put16(0x47, INT16_MIN);
}
static void StartReady(uint32_t now)
{
  ImuService_Init(now);
  unsigned before = transactions;
  Update(now+99);
  assert(transactions == before && !Snapshot().valid);
  Update(now+100); /* Identity + reset only. */
  assert(Snapshot().state == IMU_SERVICE_STATE_INITIALIZING);
  Update(now+199);
  assert(writes == 1U);
  Update(now+200); /* Configuration writes. */
  assert(writes == 13U);
  Update(now+299);
  assert(!Snapshot().valid);
  Update(now+300); /* Readback, then READY but no sample yet. */
  assert(Snapshot().state == IMU_SERVICE_STATE_READY && !Snapshot().valid);
}
static void TestDriver(void)
{
  ResetMock();
  assert(!Mpu6500_ReadRaw(NULL) && !transactions);
  assert(Mpu6500_BeginReset() && Mpu6500_Configure());
  const uint8_t expected_regs[] = {0x6B,0x6A,0x38,0x37,0x23,0x6B,0x6C,0x1A,0x1B,0x1C,0x1D,0x19,0x69};
  const uint8_t expected_values[] = {0x80,0x10,1,0,0,1,0,3,8,8,3,9,0};
  assert(writes == sizeof(expected_regs));
  assert(memcmp(written_regs, expected_regs, sizeof(expected_regs)) == 0);
  assert(memcmp(written_values, expected_values, sizeof(expected_values)) == 0);
  assert(Mpu6500_VerifyConfiguration());
  for (uint32_t i = 1; i < sizeof(expected_regs); ++i)
  {
    uint8_t saved = regs[expected_regs[i]];
    regs[expected_regs[i]] = saved ^ (i == 12 ? 0x80U : 0x01U);
    /* Select a documented field rather than a reserved bit. */
    if (i == 3) regs[0x37] = 0x80;
    if (i == 1) {
#if defined(CHASSIS_CONTROL_REVISION) && (CHASSIS_CONTROL_REVISION >= 205)
      regs[0x6A] = saved ^ 0x20; /* Stable I2C master bit must still fail. */
#else
      regs[0x6A] = saved ^ 0x10;
#endif
    }
    if (i == 8 || i == 9) regs[expected_regs[i]] = saved ^ 0x08;
    assert(!Mpu6500_VerifyConfiguration());
    regs[expected_regs[i]] = saved;
  }
  PopulateSample();
  Mpu6500RawSample raw;
  assert(Mpu6500_ReadRaw(&raw));
  assert(raw.accel_x == 8192 && raw.accel_y == -8192 && raw.accel_z == INT16_MAX);
  assert(raw.temperature == 3339 && raw.gyro_x == 65 && raw.gyro_y == -65 && raw.gyro_z == INT16_MIN);
  assert(fabsf(Mpu6500_TemperatureC(0)-21.0f) < 0.001f);
  assert(fabsf(Mpu6500_TemperatureC(3339)-31.0f) < 0.001f);
  assert(fabsf(Mpu6500_TemperatureC(-3339)-11.0f) < 0.001f);
  for (unsigned status = HAL_ERROR; status <= HAL_TIMEOUT; ++status)
  {
    for (unsigned stage = 1; stage <= 4; ++stage)
    {
      regs[0x3A] = 1U;
      memset(&raw, 0x55, sizeof(raw));
      Mpu6500RawSample saved = raw;
      fail_status = (HAL_StatusTypeDef)status;
      fail_at = transactions+stage; /* identity, power, ready-status, burst */
      assert(!Mpu6500_ReadRaw(&raw));
      assert(memcmp(&raw, &saved, sizeof(raw)) == 0 && cs == GPIO_PIN_SET);
    }
  }
  fail_at = 0;
  for (unsigned byte = 0; byte < 256; ++byte)
  {
    if (byte == 0x70) continue;
    regs[0x75] = (uint8_t)byte;
    assert(!Mpu6500_ReadRaw(&raw));
  }
  regs[0x75] = 0x70;
  regs[0x6B] = 0x40; /* Sensor-only brownout/reset must not look like a sample. */
  assert(!Mpu6500_ReadRaw(&raw));
  for (unsigned step = 1; step <= 12; ++step)
  {
    fail_at = transactions+step;
    assert(!Mpu6500_Configure());
    assert(cs == GPIO_PIN_SET);
  }
  fail_at = 0;
#if defined(CHASSIS_CONTROL_REVISION) && (CHASSIS_CONTROL_REVISION >= 205)
  /* The expected fingerprint comes from the archived 0.20.4 settings. */
  assert(Mpu6500_ConfigurationSignature()==0x212b53a9U);
  ResetMock();test_user_control_auto_clear=true;
  StartReady(0);assert(regs[0x6A]==0U);
  PopulateSample();Update(310);
  assert(Snapshot().valid && Snapshot().sample_count==1U);
  assert(Mpu6500_ConfigurationSignature()==0x212b53a9U);
  for(unsigned bit=5;bit<=6;bit++) {
    regs[0x6A]=(uint8_t)(1U<<bit);
    assert(!Mpu6500_VerifyConfiguration());
  }
  regs[0x6A]=0x10U;assert(Mpu6500_VerifyConfiguration());
  regs[0x6A]=0U;assert(Mpu6500_VerifyConfiguration());
  regs[0x1C]=0U;assert(!Mpu6500_VerifyConfiguration());
  regs[0x1C]=0x08U;
  fail_at=transactions+2U;assert(!Mpu6500_VerifyConfiguration());fail_at=0;
  puts("PASS: self-clearing USER_CTRL command permits actual service READY/sample; stable USER_CTRL/scale/read failures remain rejected; archived calibration signature unchanged");
#else
  ResetMock();test_user_control_auto_clear=true;
  ImuService_Init(0);Update(100);Update(200);Update(300);
  assert(regs[0x6A]==0U && Snapshot().state==IMU_SERVICE_STATE_IO_FAULT);
  assert(!Snapshot().valid && Snapshot().sample_count==0U);
  puts("PASS: reproducer confirms legacy readback rejects a self-cleared command before first sample");
#endif
}
static void TestService(void)
{
  ResetMock();
  ImuStaticTest_Init();
  StartReady(0);
  PopulateSample();
  Update(310);
  ImuServiceSnapshot sample = Snapshot();
  assert(sample.valid && sample.initialized && sample.who_am_i == 0x70);
  assert(sample.sample_count == 1 && sample.sample_time_ms == 310 && !sample.gyro_bias_valid);
  assert(fabsf(sample.accel_x_m_s2-9.80665f) < 0.001f);
  assert(fabsf(sample.accel_y_m_s2+9.80665f) < 0.001f);
  assert(fabsf(sample.angular_velocity_x_rad_s-(65.0f/65.5f)*0.01745329252f) < 0.000001f);
  assert(sample.angular_velocity_y_rad_s < 0 && sample.angular_velocity_z_rad_s < -8.7f);
  assert(fabsf(sample.temperature_c-31.0f) < 0.001f);
  assert(!ImuService_SetGyroBiasRaw(1,2,3));
  assert(!ImuStaticTest_Start(310,false,IMU_STATIC_SOURCE_USB));
  assert(!ImuStaticTest_Start(310,false,IMU_STATIC_SOURCE_UART));
  for (unsigned i = 0; i < 600; ++i) ImuStaticTest_ProcessSample(&sample,320+i*10);
  assert(!flash_calls && !ImuStaticTest_IsActive() && !Snapshot().gyro_bias_valid);
  fail_at = transactions+1;
  Update(320);
  assert(!Snapshot().valid && Snapshot().sample_count == 1 && Snapshot().read_error_count == 1);
  fail_at = 0;
  Update(330);
  assert(Snapshot().valid && Snapshot().sample_count == 2 && Snapshot().consecutive_errors == 0);
  regs[0x6B] = 0x40;
  Update(340); Update(350); Update(360);
  assert(!Snapshot().valid && Snapshot().state == IMU_SERVICE_STATE_IO_FAULT);
  unsigned before = transactions;
  Update(1359); assert(transactions == before);
  Update(1360); Update(1460); Update(1560);
  assert(Snapshot().state == IMU_SERVICE_STATE_READY && !Snapshot().valid);
  PopulateSample(); Update(1570);
  assert(Snapshot().valid && Snapshot().sample_count == 3 && Snapshot().read_error_count == 4);
  regs[0x1C] = 0x00; /* Scale mismatch must be caught by the periodic health check. */
  Update(2560);
  assert(!Snapshot().valid && Snapshot().state == IMU_SERVICE_STATE_IO_FAULT);
  assert(Snapshot().sample_count == 3 && Snapshot().read_error_count == 5);
  ResetMock(); regs[0x75] = 0x71;
  ImuService_Init(0); Update(100);
  assert(Snapshot().state == IMU_SERVICE_STATE_DISCONNECTED && !writes);
  before = transactions; Update(1099); assert(transactions == before);
  ResetMock();
  StartReady(0xFFFFFF00U); /* Initialization deadlines cross uint32 wrap. */
  PopulateSample(); Update(0xFFFFFF00U+310U);
  assert(Snapshot().valid && Snapshot().sample_time_ms == (uint32_t)(0xFFFFFF00U+310U));
}
static void TestDataReady(void)
{
  ResetMock(); StartReady(0); PopulateSample(); test_auto_drdy = false;
  const unsigned before = transactions;
  Update(310); /* A task wake without a hardware edge must not read/publish. */
  assert(transactions == before && Snapshot().sample_count == 0U);
  ImuService_OnDataReadyFromISR(315); Update(314); /* stale caller now */
  assert(Snapshot().valid && Snapshot().sample_time_ms == 315U);
  assert(Snapshot().sample_count == 1U);
  const unsigned sampled = transactions;
  Update(320); assert(transactions == sampled && Snapshot().sample_count == 1U);
  regs[0x3A] = 1U;
  ImuService_OnDataReadyFromISR(325); ImuService_OnDataReadyFromISR(335);
  Update(335); assert(Snapshot().sample_count == 2U && Snapshot().missed_samples == 1U);
  regs[0x3A] = 1U; ImuService_OnDataReadyFromISR(345);
  test_race_during_burst = true; Update(345);
  assert(!Snapshot().valid && Snapshot().read_races == 1U && Snapshot().sample_count == 2U);
  Update(346); assert(Snapshot().valid && Snapshot().sample_time_ms == 346U);
  ImuService_OnDataReadyFromISR(355); Update(374); /* too old: do not read registers */
  assert(!Snapshot().valid && Snapshot().missed_samples == 2U);
  Update(390);
  assert(Snapshot().state == IMU_SERVICE_STATE_IO_FAULT && Snapshot().data_ready_timeouts == 1U);
  assert(Snapshot().read_error_count == 1U);
  assert(notifications > 0U);
  ResetMock(); StartReady(0); PopulateSample(); test_auto_drdy=false;
  test_notify_on_clock=true; Update(335); /* ISR AFTER mailbox copy, before timeout. */
  assert(Snapshot().state==IMU_SERVICE_STATE_READY && Snapshot().data_ready_timeouts==0U);
  Update(336); assert(Snapshot().valid && Snapshot().sample_time_ms==335U);
  puts("PASS: DRDY-only acquisition, ISR timestamp, stale caller clock, coalesced edges, read race, stale edge and missing INT timeout");
}
int main(void)
{
  TestDriver(); TestService(); TestDataReady();
  assert(!critical_depth && !flash_calls && cs == GPIO_PIN_SET);
  puts("PASS: MPU6500 reset/configuration/readback, 14-byte signed decode, nominal die temperature, CS and HAL error paths");
  puts("PASS: actual service startup/wait/recovery, identity/reset/scale mismatch rejection, timestamp/units, tick wrap");
  puts("PASS: valid six-axis data cannot enable legacy USB/UART calibration or Flash writes");
  return 0;
}
