#ifndef ENCODER_DIAGNOSTICS_H_
#define ENCODER_DIAGNOSTICS_H_
#include <stdbool.h>
#include <stdint.h>
#include "encoder_service.h"

/* Read-only 20 ms observations, not pulse capture. sampled_cnt is from the
 * preceding EncoderService_Update; live_cnt/ab are later instantaneous reads.
 * bad bits: 1 CEN, 2 SMS, 4 CCMR1, 8 CCER, 16 ARR, 32 PSC/CKD,
 * 64 GPIO mode/AF/pull, 128 timer/GPIO clock. No writes or auto-repair. */
typedef struct
{
  uint32_t sampled_cnt, live_cnt;
  uint8_t bad, ab;
} EncoderDiagnosticWheel;
bool EncoderDiagnostics_Read(EncoderDiagnosticWheel wheels[ENCODER_WHEEL_COUNT]);
#endif
