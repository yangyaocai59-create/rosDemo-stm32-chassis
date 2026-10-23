/**
 * @file emergency_stop.c
 * @brief ISR-safe emergency-stop latch for the motor power stage.
 */

#include "emergency_stop.h"

#include <stdint.h>

#include "board_config.h"
#include "main.h"

static volatile bool s_latched;
static volatile bool s_trip_notice;

bool EmergencyStop_IsInputActive(void)
{
  return ((ESTOP_IN_GPIO_Port->IDR & ESTOP_IN_Pin) == 0U);
}

void EmergencyStop_Latch(void)
{
  if (!s_latched)
  {
    s_trip_notice = true;
  }
  s_latched = true;
  __DMB();
  Board_EmergencyMotorShutdown();
}

void EmergencyStop_Init(void)
{
  if (EmergencyStop_IsInputActive())
  {
    EmergencyStop_Latch();
  }
}

bool EmergencyStop_IsLatched(void)
{
  return s_latched;
}

bool EmergencyStop_CanEnableMotor(void)
{
  return ((!s_latched) && (!EmergencyStop_IsInputActive()));
}

bool EmergencyStop_TakeTripNotice(void)
{
  const uint32_t previous_primask = __get_PRIMASK();
  bool notice;

  __disable_irq();
  notice = s_trip_notice;
  s_trip_notice = false;
  __DMB();
  if (previous_primask == 0U)
  {
    __enable_irq();
  }

  return notice;
}
