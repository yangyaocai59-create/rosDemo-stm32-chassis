/** MPU6500 register access on the existing CubeMX SPI1 and PA4 CS. */
#include "mpu6500.h"

#include <stddef.h>
#include <string.h>
#include "main.h"
#include "spi.h"
#include "mpu_spi_probe.h"
#if defined(CHASSIS_CONTROL_REVISION) && (CHASSIS_CONTROL_REVISION >= 205)
#include "debug_log.h"
#define MPU6500_READBACK_FIX 1
static bool s_verification_reported;
#else
#define MPU6500_READBACK_FIX 0
#endif

#define MPU6500_REG_PWR_MGMT_1 0x6BU
#define MPU6500_REG_ACCEL_XOUT_H 0x3BU
#define MPU6500_SPI_TIMEOUT_MS 5U
#define MPU6500_MAX_READ_SIZE 14U

typedef struct
{
  uint8_t address, value, verify_mask;
} Mpu6500Setting;

/* RM-MPU-6500A-00: gyro FCHOICE_B=0/DLPF=3 -> 1kHz, divider 9
 * -> 100Hz; +/-500dps; accel +/-4g/A_DLPF_CFG=3. */
static const Mpu6500Setting s_settings[] = {
  { 0x6AU, 0x10U, 0x70U }, /* I2C_IF_DIS=1, FIFO/I2C master disabled. */
  { 0x38U, 0x01U, 0x59U }, /* RAW_DATA_RDY only; PB0 rising-edge EXTI. */
  { 0x37U, 0x00U, 0xFEU }, /* No latch/bypass/external FSYNC interrupt. */
  { 0x23U, 0x00U, 0xFFU }, /* No FIFO channels. */
  { 0x6BU, 0x01U, 0xFFU }, /* Awake, temperature enabled, auto PLL clock. */
  { 0x6CU, 0x00U, 0x3FU }, /* All six axes enabled. */
  { 0x1AU, 0x03U, 0x7FU }, /* Gyro DLPF 41Hz, external sync disabled. */
  { 0x1BU, 0x08U, 0xFBU }, /* +/-500dps, FCHOICE_B=0, self-tests off. */
  { 0x1CU, 0x08U, 0xF8U }, /* +/-4g, self-tests off. */
  { 0x1DU, 0x03U, 0x0FU }, /* Accel DLPF 41Hz, bypass off. */
  { 0x19U, 0x09U, 0xFFU }, /* 1kHz/(1+9) = 100Hz. */
  { 0x69U, 0x00U, 0xC0U }  /* Wake-on-motion logic disabled. */
};

static bool ReadRegisters(uint8_t reg, uint8_t *data, uint16_t length)
{
  uint8_t tx[MPU6500_MAX_READ_SIZE+1U] = { 0U };
  uint8_t rx[MPU6500_MAX_READ_SIZE+1U] = { 0U };
  HAL_StatusTypeDef status;
  if ((data == NULL) || (length == 0U) ||
      (length > MPU6500_MAX_READ_SIZE) || (reg > 0x7FU))
  {
    return false;
  }
  tx[0] = reg | 0x80U;
  HAL_GPIO_WritePin(MPU6500_CS_GPIO_Port, MPU6500_CS_Pin, GPIO_PIN_RESET);
  status = HAL_SPI_TransmitReceive(&hspi1, tx, rx, length+1U, MPU6500_SPI_TIMEOUT_MS);
  HAL_GPIO_WritePin(MPU6500_CS_GPIO_Port, MPU6500_CS_Pin, GPIO_PIN_SET);
  if (status != HAL_OK)
  {
    return false;
  }
  memcpy(data, &rx[1], length);
  return true;
}

static bool WriteRegister(uint8_t reg, uint8_t value)
{
  uint8_t tx[2] = { reg, value };
  uint8_t rx[2] = { 0U, 0U };
  HAL_StatusTypeDef status;
  HAL_GPIO_WritePin(MPU6500_CS_GPIO_Port, MPU6500_CS_Pin, GPIO_PIN_RESET);
  status = HAL_SPI_TransmitReceive(&hspi1, tx, rx, 2U, MPU6500_SPI_TIMEOUT_MS);
  HAL_GPIO_WritePin(MPU6500_CS_GPIO_Port, MPU6500_CS_Pin, GPIO_PIN_SET);
  return status == HAL_OK;
}

static int16_t DecodeInt16(uint8_t high, uint8_t low)
{
  const uint32_t bits = ((uint32_t)high << 8U) | low;
  return (int16_t)(bits < 0x8000U ? (int32_t)bits : (int32_t)bits-65536L);
}

bool Mpu6500_ReadWhoAmI(uint8_t *who_am_i)
{
  return MpuSpiProbe_ReadWhoAmI(who_am_i) == HAL_OK;
}

bool Mpu6500_BeginReset(void)
{
#if MPU6500_READBACK_FIX
  s_verification_reported=false;
  const bool ok=WriteRegister(MPU6500_REG_PWR_MGMT_1, 0x80U);
  if (!ok) DEBUG_LOG_ERROR("MPU6500 init reset write failed reg=0x6B");
  return ok;
#else
  return WriteRegister(MPU6500_REG_PWR_MGMT_1, 0x80U);
#endif
}

bool Mpu6500_Configure(void)
{
  for (uint32_t i = 0U; i < sizeof(s_settings)/sizeof(s_settings[0]); ++i)
  {
    if (!WriteRegister(s_settings[i].address, s_settings[i].value))
    {
#if MPU6500_READBACK_FIX
      DEBUG_LOG_ERROR("MPU6500 init write failed reg=0x%02X expected=0x%02X",
        (unsigned)s_settings[i].address,(unsigned)s_settings[i].value);
#endif
      return false;
    }
  }
  return true;
}

bool Mpu6500_VerifyConfiguration(void)
{
  uint8_t value;
#if MPU6500_READBACK_FIX
  uint8_t user_control=0U;
#endif
  if (!Mpu6500_ReadWhoAmI(&value) || (value != MPU6500_WHO_AM_I_EXPECTED))
  {
#if MPU6500_READBACK_FIX
    DEBUG_LOG_ERROR("MPU6500 verify identity failed actual=0x%02X expected=0x70",(unsigned)value);
#endif
    return false;
  }
  for (uint32_t i = 0U; i < sizeof(s_settings)/sizeof(s_settings[0]); ++i)
  {
#if MPU6500_READBACK_FIX
    /* RM-MPU-6500A-00 rev 2.1 section 4.33: I2C_IF_DIS is a command
     * bit that auto-clears. Keep writing it, but verify stable bits only.
     * Retain the settings table/fingerprint for saved-bias compatibility. */
    const uint8_t mask=s_settings[i].address==0x6AU ?
      (uint8_t)(s_settings[i].verify_mask & ~0x10U) : s_settings[i].verify_mask;
    if (!ReadRegisters(s_settings[i].address, &value, 1U)) {
      DEBUG_LOG_ERROR("MPU6500 verify read failed reg=0x%02X",(unsigned)s_settings[i].address);
      return false;
    }
    if ((value & mask)!=(s_settings[i].value & mask)) {
      DEBUG_LOG_ERROR("MPU6500 verify mismatch reg=0x%02X actual=0x%02X expected=0x%02X mask=0x%02X",
        (unsigned)s_settings[i].address,(unsigned)value,
        (unsigned)(s_settings[i].value & mask),(unsigned)mask);
      return false;
    }
    if (s_settings[i].address==0x6AU) user_control=value;
#else
    if (!ReadRegisters(s_settings[i].address, &value, 1U) ||
        ((value & s_settings[i].verify_mask) != s_settings[i].value))
    {
      return false;
    }
#endif
  }
#if MPU6500_READBACK_FIX
  if (!s_verification_reported) {
    DEBUG_LOG_INFO("MPU6500 verify OK USER_CTRL=0x%02X stable_mask=0x60",(unsigned)user_control);
    s_verification_reported=true;
  }
#endif
  return true;
}

uint32_t Mpu6500_ConfigurationSignature(void)
{
  uint32_t hash = 2166136261UL;
  for (uint32_t i = 0U; i < sizeof(s_settings)/sizeof(s_settings[0]); ++i)
  {
    hash = (hash ^ s_settings[i].address) * 16777619UL;
    hash = (hash ^ s_settings[i].value) * 16777619UL;
    hash = (hash ^ s_settings[i].verify_mask) * 16777619UL;
  }
  return hash;
}

bool Mpu6500_ReadRaw(Mpu6500RawSample *sample)
{
  uint8_t identity, power, interrupt_status;
  uint8_t data[MPU6500_MAX_READ_SIZE];
  if ((sample == NULL) || !Mpu6500_ReadWhoAmI(&identity) ||
      (identity != MPU6500_WHO_AM_I_EXPECTED) ||
      !ReadRegisters(MPU6500_REG_PWR_MGMT_1, &power, 1U) || (power != 0x01U) ||
      !ReadRegisters(0x3AU, &interrupt_status, 1U) ||
      ((interrupt_status & 0x01U) == 0U) ||
      !ReadRegisters(MPU6500_REG_ACCEL_XOUT_H, data, sizeof(data)))
  {
    return false;
  }
  /* One 14-byte burst under one CS assertion: accel, temp, gyro. */
  sample->accel_x = DecodeInt16(data[0], data[1]);
  sample->accel_y = DecodeInt16(data[2], data[3]);
  sample->accel_z = DecodeInt16(data[4], data[5]);
  sample->temperature = DecodeInt16(data[6], data[7]);
  sample->gyro_x = DecodeInt16(data[8], data[9]);
  sample->gyro_y = DecodeInt16(data[10], data[11]);
  sample->gyro_z = DecodeInt16(data[12], data[13]);
  return true;
}

float Mpu6500_TemperatureC(int16_t raw)
{
  /* PS-MPU-6500A-01: nominal 333.87 LSB/C and zero offset at 21C.
   * This is die temperature, not a calibrated ambient thermometer. */
  return (float)raw / 333.87f + 21.0f;
}
