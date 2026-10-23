/** Task-context blocking SPI1, DRDY-triggered MPU6500; no DMA, FIFO or DMP. */
#ifndef APPLICATION_SENSORS_MPU6500_H_
#define APPLICATION_SENSORS_MPU6500_H_
#include <stdbool.h>
#include <stdint.h>

#define MPU6500_WHO_AM_I_EXPECTED 0x70U

typedef struct
{
  int16_t accel_x, accel_y, accel_z;
  int16_t temperature;
  int16_t gyro_x, gyro_y, gyro_z;
} Mpu6500RawSample;

/* Task context, SPI1 exclusive ownership; never called from PB0 EXTI. */
bool Mpu6500_ReadWhoAmI(uint8_t *who_am_i);
bool Mpu6500_BeginReset(void);
bool Mpu6500_Configure(void);
bool Mpu6500_VerifyConfiguration(void);
/** Pure fingerprint of the actual register settings, for saved bias profiles. */
uint32_t Mpu6500_ConfigurationSignature(void);
/* Leaves output untouched if identity/power/read check fails. */
bool Mpu6500_ReadRaw(Mpu6500RawSample *sample);
float Mpu6500_TemperatureC(int16_t raw);

#endif
