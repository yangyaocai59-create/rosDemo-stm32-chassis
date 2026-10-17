#ifndef TEST_LOG_HAL_H
#define TEST_LOG_HAL_H
#include <stdint.h>
typedef struct { unsigned dummy; } UART_HandleTypeDef;
#define HAL_OK 0
uint32_t HAL_GetTick(void);
uint32_t HAL_RCC_GetSysClockFreq(void);
unsigned __get_IPSR(void);
int HAL_UART_Transmit(UART_HandleTypeDef *uart, uint8_t *data, uint16_t length, uint32_t timeout);
#endif
