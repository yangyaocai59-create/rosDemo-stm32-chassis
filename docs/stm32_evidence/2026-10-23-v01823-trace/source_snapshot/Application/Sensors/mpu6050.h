/**
 * @file mpu6050.h
 * @brief Minimal polling driver for an MPU6050 on the CubeMX I2C1 handle.
 */

#ifndef APPLICATION_SENSORS_MPU6050_H_
#define APPLICATION_SENSORS_MPU6050_H_

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define MPU6050_WHO_AM_I_EXPECTED 0x68U

typedef struct
{
  int16_t accel_x;
  int16_t accel_y;
  int16_t accel_z;
  int16_t temperature;
  int16_t gyro_x;
  int16_t gyro_y;
  int16_t gyro_z;
} Mpu6050RawSample;

bool Mpu6050_ReadWhoAmI(uint8_t *who_am_i);
bool Mpu6050_BeginReset(void);
bool Mpu6050_Configure(void);
bool Mpu6050_VerifyConfiguration(void);
bool Mpu6050_ReadRaw(Mpu6050RawSample *sample);

#ifdef __cplusplus
}
#endif

#endif /* APPLICATION_SENSORS_MPU6050_H_ */
