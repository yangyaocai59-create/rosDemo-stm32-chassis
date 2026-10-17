#ifndef TEST_SPI_MAIN_H_
#define TEST_SPI_MAIN_H_
#include "stm32f4xx_hal.h"
extern GPIO_TypeDef test_gpio_a, test_gpio_b;
#define MPU6500_CS_GPIO_Port (&test_gpio_a)
#define MPU6500_CS_Pin 16U
#define IMU_INT_GPIO_Port (&test_gpio_b)
#define IMU_INT_Pin 1U
#endif
