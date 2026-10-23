/* Real production driver; mock explicitly models CCR preload and update. */
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "main.h"
#include "motor_driver.h"

GPIO_TypeDef test_gpio;
static TestTimerRegisters registers;
TIM_HandleTypeDef htim8 = {.arr=8399U, .Instance=&registers};
static uint32_t tick, primask;
static bool estop;
static unsigned compare_writes, zero_writes, stby_writes, direction_writes;
static bool assert_zero_before_direction;
uint32_t HAL_GetTick(void) { return tick; }
uint32_t __get_PRIMASK(void) { return primask; }
void __disable_irq(void) { primask = 1U; }
void __enable_irq(void) { primask = 0U; }
void HAL_GPIO_WritePin(GPIO_TypeDef *port, uint16_t pins, GPIO_PinState state)
{
  if (pins & (MOTOR_STBY_1_Pin | MOTOR_STBY_2_Pin)) { ++stby_writes; }
  else
  {
    ++direction_writes;
    if (assert_zero_before_direction && state == GPIO_PIN_SET)
      for (unsigned w=0; w<4; ++w)
        if (pins & (3U << (2U*w))) assert(htim8.active[w] == 0U);
  }
  if (state == GPIO_PIN_SET) port->pins |= pins;
  else port->pins &= (uint16_t)~pins;
}
void Test_SetCompare(TIM_HandleTypeDef *timer, uint32_t channel, uint32_t value)
{
  assert(channel < 4U && value <= 8400U);
  ++compare_writes; if (value == 0U) ++zero_writes;
  timer->preload[channel] = value; /* deliberately NOT active */
}
int HAL_TIM_PWM_Start(TIM_HandleTypeDef *timer, uint32_t channel)
{ (void)timer; (void)channel; return HAL_OK; }
int HAL_TIM_PWM_Stop(TIM_HandleTypeDef *timer, uint32_t channel)
{ (void)timer; (void)channel; return HAL_OK; }
bool EmergencyStop_CanEnableMotor(void) { return !estop; }
bool EmergencyStop_IsLatched(void) { return estop; }
bool EmergencyStop_IsInputActive(void) { return estop; }
void Board_EmergencyMotorShutdown(void)
{
  test_gpio.pins = 0U;
  memset(htim8.preload, 0, sizeof(htim8.preload));
  memset(htim8.active, 0, sizeof(htim8.active));
}
void Board_ForceMotorSafeState(void) { Board_EmergencyMotorShutdown(); }
static void UpdateEvent(void)
{ memcpy(htim8.active, htim8.preload, sizeof(htim8.active)); }
static int16_t Applied(MotorDriverWheel wheel)
{ MotorDriverStatus s; assert(MotorDriver_GetStatus(&s)); return s.command_permille[wheel]; }

int main(void)
{
  assert(MotorDriver_Init());
  assert(!MotorDriver_SetCommand(MOTOR_DRIVER_WHEEL_LF,100));
  assert(MotorDriver_Enable());
  assert(MotorDriver_SetCommand(MOTOR_DRIVER_WHEEL_LF,100));
  assert(Applied(MOTOR_DRIVER_WHEEL_LF)==0); /* fresh zero guard */
  tick=2; UpdateEvent(); assert_zero_before_direction=true;
  assert(MotorDriver_SetCommand(MOTOR_DRIVER_WHEEL_LF,100));
  UpdateEvent(); assert(Applied(MOTOR_DRIVER_WHEEL_LF)==100);
  MotorDriverDiagnostics diag;
  const unsigned diag_writes = compare_writes + direction_writes + stby_writes;
  registers.BDTR=0x8000U; registers.CCER=0x1111U;
  assert(MotorDriver_GetDiagnostics(&diag));
  assert(diag.compare[0]==840U && diag.auto_reload==8399U);
  assert(diag.bdtr==0x8000U && diag.ccer==0x1111U);
  assert(diag.standby_pin && (diag.direction_bits & 3U)==1U);
  assert(diag_writes==compare_writes + direction_writes + stby_writes);
  assert(!MotorDriver_GetDiagnostics(NULL));
  const unsigned before_zero=zero_writes, before_dir=direction_writes,
    before_stby=stby_writes, before_compare=compare_writes;
  for(unsigned i=0;i<1000;i++)
  {
    tick+=10; assert(MotorDriver_SetCommand(MOTOR_DRIVER_WHEEL_LF,100));
    UpdateEvent();
  }
  assert(zero_writes==before_zero && direction_writes==before_dir &&
    stby_writes==before_stby && compare_writes==before_compare+1000);
  puts("PASS: 1000 same-direction updates write only nonzero CCR, no GPIO/STBY/zero pulses");

  assert(MotorDriver_SetCommand(MOTOR_DRIVER_WHEEL_LF,-100));
  assert(Applied(MOTOR_DRIVER_WHEEL_LF)==0 && (test_gpio.pins&3U)==0U);
  assert(htim8.active[0]!=0U); /* preload has not latched; GPIO already coasts */
  assert(MotorDriver_SetCommand(MOTOR_DRIVER_WHEEL_LF,-100));
  assert(Applied(MOTOR_DRIVER_WHEEL_LF)==0);
  ++tick; UpdateEvent(); assert(MotorDriver_SetCommand(MOTOR_DRIVER_WHEEL_LF,-100));
  assert(Applied(MOTOR_DRIVER_WHEEL_LF)==0);
  ++tick; UpdateEvent(); assert(MotorDriver_SetCommand(MOTOR_DRIVER_WHEEL_LF,-100));
  assert(Applied(MOTOR_DRIVER_WHEEL_LF)==-100);
  UpdateEvent();
  MotorDriver_Disable(); assert((test_gpio.pins&MOTOR_STBY_1_Pin)==0U);
  assert(MotorDriver_Enable());
  assert(MotorDriver_SetCommand(MOTOR_DRIVER_WHEEL_LF,100));
  assert(Applied(MOTOR_DRIVER_WHEEL_LF)==0);
  tick+=2; UpdateEvent(); assert(MotorDriver_SetCommand(MOTOR_DRIVER_WHEEL_LF,100));
  assert(Applied(MOTOR_DRIVER_WHEEL_LF)==100);
  puts("PASS: immediate GPIO coast despite active preload, >=2-tick reversal/re-enable guard, no wait");

  MotorDriver_Disable(); tick=0xFFFFFFFEU;
  assert(MotorDriver_Enable()); UpdateEvent();
  /* Explicit coast starts a fresh guard at the wrap boundary. */
  tick+=2U; UpdateEvent();
  assert(MotorDriver_SetCommand(MOTOR_DRIVER_WHEEL_LF,100)); UpdateEvent();
  tick=0xFFFFFFFFU;
  assert(MotorDriver_SetCommand(MOTOR_DRIVER_WHEEL_LF,-100));
  tick=0U; UpdateEvent(); assert(MotorDriver_SetCommand(MOTOR_DRIVER_WHEEL_LF,-100));
  assert(Applied(MOTOR_DRIVER_WHEEL_LF)==0);
  tick=1U; UpdateEvent(); assert(MotorDriver_SetCommand(MOTOR_DRIVER_WHEEL_LF,-100));
  assert(Applied(MOTOR_DRIVER_WHEEL_LF)==-100);
  estop=true; assert(!MotorDriver_SetCommand(MOTOR_DRIVER_WHEEL_LF,100));
  assert(test_gpio.pins==0U && Applied(MOTOR_DRIVER_WHEEL_LF)==0 && primask==0U);
  puts("PASS: guard tick wrap, ESTOP rejects output and clears STBY immediately");
  return 0;
}
