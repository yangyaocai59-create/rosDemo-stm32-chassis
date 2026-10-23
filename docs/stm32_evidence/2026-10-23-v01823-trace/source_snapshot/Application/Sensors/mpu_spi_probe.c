/**
 * @brief Read register 0x75 over the existing CubeMX SPI1 / GPIO PA4 CS.
 * Identity bytes suggest a compatible model; they do not prove authenticity.
 */
#include "mpu_spi_probe.h"

#include <stddef.h>
#include "main.h"
#include "spi.h"

#define MPU_SPI_WHO_AM_I_REGISTER 0x75U
#define MPU_SPI_READ_BIT          0x80U
#define MPU_SPI_TIMEOUT_MS        5U

HAL_StatusTypeDef MpuSpiProbe_ReadWhoAmI(uint8_t *who_am_i)
{
  uint8_t tx[2] = { MPU_SPI_WHO_AM_I_REGISTER | MPU_SPI_READ_BIT, 0x00U };
  uint8_t rx[2] = { 0U, 0U };
  HAL_StatusTypeDef status;

  if (who_am_i == NULL)
  {
    return HAL_ERROR;
  }
  *who_am_i = 0U;
  /* CS stays asserted for both command and response, including scheduling
   * preemption. There is no SPI register write command in this module. */
  HAL_GPIO_WritePin(MPU6500_CS_GPIO_Port, MPU6500_CS_Pin, GPIO_PIN_RESET);
  status = HAL_SPI_TransmitReceive(&hspi1, tx, rx, 2U, MPU_SPI_TIMEOUT_MS);
  /* Always release CS, also for HAL_ERROR/HAL_BUSY/HAL_TIMEOUT. */
  HAL_GPIO_WritePin(MPU6500_CS_GPIO_Port, MPU6500_CS_Pin, GPIO_PIN_SET);
  if (status == HAL_OK)
  {
    *who_am_i = rx[1];
  }
  return status;
}

bool MpuSpiProbe_IsKnownIdentity(uint8_t who_am_i)
{
  return (who_am_i == 0x70U) || (who_am_i == 0x71U) ||
         (who_am_i == 0x73U);
}

const char *MpuSpiProbe_CandidateName(uint8_t who_am_i)
{
  switch (who_am_i)
  {
    case 0x70U: return "MPU6500";
    case 0x71U: return "MPU9250";
    case 0x73U: return "MPU9255";
    default: return "UNKNOWN";
  }
}

const char *MpuSpiProbe_HalStatusName(HAL_StatusTypeDef status)
{
  switch (status)
  {
    case HAL_OK: return "OK";
    case HAL_ERROR: return "ERROR";
    case HAL_BUSY: return "BUSY";
    case HAL_TIMEOUT: return "TIMEOUT";
    default: return "UNKNOWN";
  }
}
