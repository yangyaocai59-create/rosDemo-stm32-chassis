#ifndef TEST_MOTION_MAIN_H_
#define TEST_MOTION_MAIN_H_
#include <stdbool.h>
#include <stdint.h>
typedef enum {GPIO_PIN_RESET, GPIO_PIN_SET} GPIO_PinState;
typedef struct { uint16_t pins; } GPIO_TypeDef;
typedef struct { uint32_t arr, preload[4], active[4]; } TIM_HandleTypeDef;
extern GPIO_TypeDef test_gpio;
extern TIM_HandleTypeDef htim8;
#define HAL_OK 0
uint32_t HAL_GetTick(void);
void HAL_GPIO_WritePin(GPIO_TypeDef *, uint16_t, GPIO_PinState);
int HAL_TIM_PWM_Start(TIM_HandleTypeDef *, uint32_t);
int HAL_TIM_PWM_Stop(TIM_HandleTypeDef *, uint32_t);
uint32_t __get_PRIMASK(void);
void __disable_irq(void);
void __enable_irq(void);
void Test_SetCompare(TIM_HandleTypeDef *, uint32_t, uint32_t);
#define __HAL_TIM_GET_AUTORELOAD(t) ((t)->arr)
#define __HAL_TIM_SET_COMPARE(t,c,v) Test_SetCompare(t,c,v)
#define LF_IN1_Pin (1U<<0)
#define LF_IN2_Pin (1U<<1)
#define LR_IN1_Pin (1U<<2)
#define LR_IN2_Pin (1U<<3)
#define RF_IN1_Pin (1U<<4)
#define RF_IN2_Pin (1U<<5)
#define RR_IN1_Pin (1U<<6)
#define RR_IN2_Pin (1U<<7)
#define MOTOR_STBY_1_Pin (1U<<10)
#define MOTOR_STBY_2_Pin (1U<<11)
#define LF_IN1_GPIO_Port (&test_gpio)
#define LF_IN2_GPIO_Port (&test_gpio)
#define LR_IN1_GPIO_Port (&test_gpio)
#define LR_IN2_GPIO_Port (&test_gpio)
#define RF_IN1_GPIO_Port (&test_gpio)
#define RF_IN2_GPIO_Port (&test_gpio)
#define RR_IN1_GPIO_Port (&test_gpio)
#define RR_IN2_GPIO_Port (&test_gpio)
#define MOTOR_STBY_1_GPIO_Port (&test_gpio)
#define MOTOR_STBY_2_GPIO_Port (&test_gpio)
#endif
