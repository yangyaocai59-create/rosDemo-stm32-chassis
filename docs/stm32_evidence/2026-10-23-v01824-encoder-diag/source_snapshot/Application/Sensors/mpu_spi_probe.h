/** Read-only MPU-family SPI identity access. No reset/configuration writes. */
#ifndef APPLICATION_SENSORS_MPU_SPI_PROBE_H_
#define APPLICATION_SENSORS_MPU_SPI_PROBE_H_

#include <stdbool.h>
#include <stdint.h>
#include "stm32f4xx_hal.h"

/* Task-context only; SPI1 and PA4 CS are exclusively owned by this probe. */
HAL_StatusTypeDef MpuSpiProbe_ReadWhoAmI(uint8_t *who_am_i);
bool MpuSpiProbe_IsKnownIdentity(uint8_t who_am_i);
const char *MpuSpiProbe_CandidateName(uint8_t who_am_i);
const char *MpuSpiProbe_HalStatusName(HAL_StatusTypeDef status);

#endif
