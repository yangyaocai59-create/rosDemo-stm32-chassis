/**
 * @file mpu6050.c
 * @brief MPU6050 register access using blocking HAL I2C transactions.
 */

#include "mpu6050.h"

#include "i2c.h"
#include "imu_config.h"

#define MPU6050_HAL_ADDRESS \
  ((uint16_t)(IMU_MPU6050_I2C_ADDRESS_7BIT << 1U))
#define MPU6050_I2C_TIMEOUT_MS 20U

#define MPU6050_REG_SMPLRT_DIV    0x19U
#define MPU6050_REG_CONFIG        0x1AU
#define MPU6050_REG_GYRO_CONFIG   0x1BU
#define MPU6050_REG_ACCEL_CONFIG  0x1CU
#define MPU6050_REG_INT_ENABLE    0x38U
#define MPU6050_REG_ACCEL_XOUT_H  0x3BU
#define MPU6050_REG_PWR_MGMT_1    0x6BU
#define MPU6050_REG_PWR_MGMT_2    0x6CU
#define MPU6050_REG_WHO_AM_I      0x75U

#define MPU6050_PWR_DEVICE_RESET  0x80U
#define MPU6050_CLOCK_PLL_XGYRO   0x01U
#define MPU6050_DLPF_44HZ         0x03U
#define MPU6050_GYRO_FS_500DPS    0x08U
#define MPU6050_ACCEL_FS_4G       0x08U
#define MPU6050_SAMPLE_DIV_100HZ  9U

static bool Mpu6050_WriteRegister(uint8_t reg, uint8_t value)
{
  return HAL_I2C_Mem_Write(&hi2c1,
                           MPU6050_HAL_ADDRESS,
                           reg,
                           I2C_MEMADD_SIZE_8BIT,
                           &value,
                           1U,
                           MPU6050_I2C_TIMEOUT_MS) == HAL_OK;
}

static bool Mpu6050_ReadRegisters(uint8_t reg, uint8_t *data, uint16_t length)
{
  if ((data == NULL) || (length == 0U))
  {
    return false;
  }

  return HAL_I2C_Mem_Read(&hi2c1,
                          MPU6050_HAL_ADDRESS,
                          reg,
                          I2C_MEMADD_SIZE_8BIT,
                          data,
                          length,
                          MPU6050_I2C_TIMEOUT_MS) == HAL_OK;
}

static int16_t Mpu6050_DecodeInt16(const uint8_t high, const uint8_t low)
{
  return (int16_t)(((uint16_t)high << 8U) | (uint16_t)low);
}

bool Mpu6050_ReadWhoAmI(uint8_t *who_am_i)
{
  return Mpu6050_ReadRegisters(MPU6050_REG_WHO_AM_I, who_am_i, 1U);
}

bool Mpu6050_BeginReset(void)
{
  return Mpu6050_WriteRegister(MPU6050_REG_PWR_MGMT_1,
                               MPU6050_PWR_DEVICE_RESET);
}

bool Mpu6050_Configure(void)
{
  /* 100 Hz raw sampling, 44 Hz DLPF, +/-4 g and +/-500 deg/s. */
  return Mpu6050_WriteRegister(MPU6050_REG_PWR_MGMT_1,
                               MPU6050_CLOCK_PLL_XGYRO) &&
         Mpu6050_WriteRegister(MPU6050_REG_PWR_MGMT_2, 0U) &&
         Mpu6050_WriteRegister(MPU6050_REG_CONFIG, MPU6050_DLPF_44HZ) &&
         Mpu6050_WriteRegister(MPU6050_REG_SMPLRT_DIV,
                               MPU6050_SAMPLE_DIV_100HZ) &&
         Mpu6050_WriteRegister(MPU6050_REG_GYRO_CONFIG,
                               MPU6050_GYRO_FS_500DPS) &&
         Mpu6050_WriteRegister(MPU6050_REG_ACCEL_CONFIG,
                               MPU6050_ACCEL_FS_4G) &&
         /* Stage 12A polls data; PB0/IMU_INT remains reserved for later. */
         Mpu6050_WriteRegister(MPU6050_REG_INT_ENABLE, 0U);
}

bool Mpu6050_VerifyConfiguration(void)
{
  uint8_t value = 0U;

  if (!Mpu6050_ReadRegisters(MPU6050_REG_PWR_MGMT_1, &value, 1U) ||
      (value != MPU6050_CLOCK_PLL_XGYRO))
  {
    return false;
  }
  if (!Mpu6050_ReadRegisters(MPU6050_REG_CONFIG, &value, 1U) ||
      ((value & 0x07U) != MPU6050_DLPF_44HZ))
  {
    return false;
  }
  if (!Mpu6050_ReadRegisters(MPU6050_REG_SMPLRT_DIV, &value, 1U) ||
      (value != MPU6050_SAMPLE_DIV_100HZ))
  {
    return false;
  }
  if (!Mpu6050_ReadRegisters(MPU6050_REG_GYRO_CONFIG, &value, 1U) ||
      ((value & 0x18U) != MPU6050_GYRO_FS_500DPS))
  {
    return false;
  }
  if (!Mpu6050_ReadRegisters(MPU6050_REG_ACCEL_CONFIG, &value, 1U) ||
      ((value & 0x18U) != MPU6050_ACCEL_FS_4G))
  {
    return false;
  }

  return true;
}

bool Mpu6050_ReadRaw(Mpu6050RawSample *sample)
{
  uint8_t data[14];

  if ((sample == NULL) ||
      !Mpu6050_ReadRegisters(MPU6050_REG_ACCEL_XOUT_H,
                             data,
                             (uint16_t)sizeof(data)))
  {
    return false;
  }

  sample->accel_x = Mpu6050_DecodeInt16(data[0], data[1]);
  sample->accel_y = Mpu6050_DecodeInt16(data[2], data[3]);
  sample->accel_z = Mpu6050_DecodeInt16(data[4], data[5]);
  sample->temperature = Mpu6050_DecodeInt16(data[6], data[7]);
  sample->gyro_x = Mpu6050_DecodeInt16(data[8], data[9]);
  sample->gyro_y = Mpu6050_DecodeInt16(data[10], data[11]);
  sample->gyro_z = Mpu6050_DecodeInt16(data[12], data[13]);

  return true;
}
