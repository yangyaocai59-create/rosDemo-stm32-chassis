#ifndef TEST_SPI_FREERTOS_H_
#define TEST_SPI_FREERTOS_H_
#include <stdint.h>
typedef int BaseType_t;
typedef void *TaskHandle_t;
#define pdFALSE 0
#define portYIELD_FROM_ISR(value) ((void)(value))
TaskHandle_t xTaskGetCurrentTaskHandle(void);
void vTaskNotifyGiveFromISR(TaskHandle_t handle, BaseType_t *woken);
void Test_EnterCritical(void);
void Test_ExitCritical(void);
#define taskENTER_CRITICAL() Test_EnterCritical()
#define taskEXIT_CRITICAL() Test_ExitCritical()
#endif
