/* Includes actual logger with mock RTOS/HAL; no COM port is used. */
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "../../../Application/Services/debug_log.c"
UART_HandleTypeDef huart1;
static osKernelState_t kernel=osKernelReady;
static uintptr_t thread_id=1;
static unsigned depth, irq, uart_calls, lock_calls;
static int uart_result=HAL_OK, lock_result=osOK;
static uint32_t tick=1000;
static char last[256];
void Test_EnterCritical(void) { assert(kernel==osKernelRunning); ++depth; }
void Test_ExitCritical(void) { assert(depth); --depth; }
osThreadId_t osThreadGetId(void) { return (void *)thread_id; }
osKernelState_t osKernelGetState(void) { return kernel; }
osMutexId_t osMutexNew(const osMutexAttr_t *attributes) { assert(attributes); return (void *)1; }
osStatus_t osMutexAcquire(osMutexId_t mutex,uint32_t timeout)
{ (void)mutex; (void)timeout; assert(!depth); ++lock_calls; return lock_result; }
osStatus_t osMutexRelease(osMutexId_t mutex) { (void)mutex; assert(!depth); return osOK; }
uint32_t HAL_GetTick(void) { return tick; }
uint32_t HAL_RCC_GetSysClockFreq(void) { return 168000000; }
unsigned __get_IPSR(void) { return irq; }
int HAL_UART_Transmit(UART_HandleTypeDef *uart,uint8_t *data,uint16_t length,uint32_t timeout)
{
  (void)uart; (void)timeout; assert(!depth && length<sizeof(last));
  ++uart_calls; memcpy(last,data,length); last[length]=0; return uart_result;
}
SystemResetCause SystemStatus_GetResetCause(void) { return SYSTEM_RESET_CAUSE_PIN; }
void SystemStatus_ResetCauseToString(SystemResetCause cause,char *buffer,size_t size)
{ (void)cause; (void)snprintf(buffer,size,"PIN"); }
int main(void)
{
  assert(DebugLog_Init()); assert(!DebugLog_BeginDeferred());
  assert(DebugLog_Printf(DEBUG_LOG_LEVEL_INFO,"startup"));
  assert(uart_calls==1 && lock_calls==0); /* No pre-scheduler RTOS critical use. */
  kernel=osKernelRunning;
  assert(DebugLog_BeginDeferred()); assert(DebugLog_BeginDeferred());
  assert(DebugLog_Printf(DEBUG_LOG_LEVEL_INFO,"STOP %d",7));
  assert(uart_calls==1 && lock_calls==0);
  DebugLog_EndDeferred(); assert(DebugLog_Printf(DEBUG_LOG_LEVEL_DEBUG,"nested"));
  thread_id=2;
  assert(!DebugLog_BeginDeferred());
  assert(DebugLog_Printf(DEBUG_LOG_LEVEL_INFO,"other task remains synchronous"));
  assert(uart_calls==2 && lock_calls==1);
  thread_id=1; DebugLog_EndDeferred();
  tick=2000; thread_id=2; DebugLog_ProcessDeferred(1);
  assert(uart_calls==3 && strstr(last,"[0000001000]") && strstr(last,"STOP 7"));
  DebugLog_ProcessDeferred(1); assert(uart_calls==4 && strstr(last,"nested"));
  puts("PASS: pre-scheduler logging, task-local nesting, no USB UART/mutex wait, deferred original timestamp");

  thread_id=1; assert(DebugLog_BeginDeferred());
  char huge[600]; memset(huge,'x',599); huge[599]=0;
  for(unsigned i=0;i<DEBUG_LOG_DEFERRED_DEPTH-1;i++) { assert(DebugLog_Printf(DEBUG_LOG_LEVEL_INFO,"%s",huge)); }
  assert(!DebugLog_Printf(DEBUG_LOG_LEVEL_INFO,"overflow"));
  assert(DebugLog_GetDeferredDroppedCount()==1);
  DebugLog_EndDeferred(); thread_id=2;
  lock_result=1; DebugLog_ProcessDeferred(1); assert(s_deferred_head!=s_deferred_tail);
  lock_result=osOK; DebugLog_ProcessDeferred(1);
  assert(strlen(last)==255 && last[253]=='\r' && last[254]=='\n');
  uart_result=1; DebugLog_ProcessDeferred(1); assert(DebugLog_GetDeferredDroppedCount()==2);
  uart_result=HAL_OK; DebugLog_ProcessDeferred(DEBUG_LOG_DEFERRED_DEPTH); assert(s_deferred_head==s_deferred_tail);
  irq=1; assert(!DebugLog_Printf(DEBUG_LOG_LEVEL_INFO,"ISR rejected"));
  assert(!DebugLog_BeginDeferred()); irq=0; assert(depth==0);
  puts("PASS: finite log queue, truncation, full/UART-failure accounting, mutex retry and ISR rejection");
  const unsigned before_uart=uart_calls, before_lock=lock_calls;
  for(thread_id=3; thread_id<=5; ++thread_id)
  {
    assert(DebugLog_RegisterCurrentTaskNonBlocking());
    assert(DebugLog_RegisterCurrentTaskNonBlocking());
    assert(DebugLog_Printf(DEBUG_LOG_LEVEL_ERROR,"real-time task %u",(unsigned)thread_id));
  }
  assert(!DebugLog_RegisterCurrentTaskNonBlocking()); /* thread 6: bounded registry */
  assert(uart_calls==before_uart && lock_calls==before_lock);
  thread_id=2; DebugLog_ProcessDeferred(DEBUG_LOG_DEFERRED_DEPTH);
  assert(uart_calls==before_uart+3 && s_deferred_head==s_deferred_tail);
  puts("PASS: permanent motor/safety/IMU producers are independent and never acquire UART/mutex");
  return 0;
}
