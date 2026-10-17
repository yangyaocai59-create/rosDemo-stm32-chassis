#ifndef TEST_CMSIS_OS2_H
#define TEST_CMSIS_OS2_H
#include <stdint.h>
typedef void *osThreadId_t;
typedef void *osMutexId_t;
typedef enum { osKernelInactive=0, osKernelReady=1, osKernelRunning=2 } osKernelState_t;
typedef int osStatus_t;
typedef struct { const char *name; uint32_t attr_bits; void *cb_mem; uint32_t cb_size; } osMutexAttr_t;
#define osOK 0
osThreadId_t osThreadGetId(void);
osKernelState_t osKernelGetState(void);
osMutexId_t osMutexNew(const osMutexAttr_t *attributes);
osStatus_t osMutexAcquire(osMutexId_t mutex, uint32_t timeout);
osStatus_t osMutexRelease(osMutexId_t mutex);
#endif
