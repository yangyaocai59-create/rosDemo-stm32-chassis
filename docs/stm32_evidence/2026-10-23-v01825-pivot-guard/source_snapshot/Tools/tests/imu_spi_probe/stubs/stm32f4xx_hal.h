#ifndef TEST_SPI_HAL_H_
#define TEST_SPI_HAL_H_
#include <stdint.h>
typedef enum { HAL_OK, HAL_ERROR, HAL_BUSY, HAL_TIMEOUT } HAL_StatusTypeDef;
typedef enum { GPIO_PIN_RESET, GPIO_PIN_SET } GPIO_PinState;
typedef struct { unsigned dummy; } GPIO_TypeDef;
typedef struct { unsigned dummy; } SPI_HandleTypeDef;
uint32_t HAL_GetTick(void);
void HAL_GPIO_WritePin(GPIO_TypeDef *, uint16_t, GPIO_PinState);
GPIO_PinState HAL_GPIO_ReadPin(GPIO_TypeDef *, uint16_t);
HAL_StatusTypeDef HAL_SPI_TransmitReceive(SPI_HandleTypeDef *, uint8_t *, uint8_t *, uint16_t, uint32_t);
#define FLASH_FLAG_EOP 1U
#define FLASH_FLAG_OPERR 2U
#define FLASH_FLAG_WRPERR 4U
#define FLASH_FLAG_PGAERR 8U
#define FLASH_FLAG_PGPERR 16U
#define FLASH_FLAG_PGSERR 32U
#define FLASH_TYPEPROGRAM_WORD 2U
void Test_FlashClearFlags(uint32_t);
#define __HAL_FLASH_CLEAR_FLAG(flags) Test_FlashClearFlags(flags)
HAL_StatusTypeDef HAL_FLASH_Unlock(void);
HAL_StatusTypeDef HAL_FLASH_Lock(void);
HAL_StatusTypeDef HAL_FLASH_Program(uint32_t, uint32_t, uint64_t);
#endif
