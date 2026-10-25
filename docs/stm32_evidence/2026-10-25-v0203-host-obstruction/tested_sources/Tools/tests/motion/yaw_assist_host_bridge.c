/* Windows test bridge for the exact allocation-free firmware control core.
 * Not linked into MCU firmware. The Python runner owns transport and STOP. */
#include "yaw_rate_control_core.h"
#ifdef _WIN32
#define EXPORT __declspec(dllexport)
#else
#define EXPORT
#endif
static YawRateControlState s_state;
EXPORT void YawHost_Reset(void) { YawRateControl_Reset(&s_state); }
EXPORT float YawHost_Step(float reference,float measured,uint32_t elapsed_ms,bool valid)
{
  return YawRateControl_Step(&s_state,0.0f,reference,measured,valid,false,elapsed_ms);
}
