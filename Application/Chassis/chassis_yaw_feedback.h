#ifndef CHASSIS_YAW_FEEDBACK_H_
#define CHASSIS_YAW_FEEDBACK_H_
#include <stdbool.h>
#include <stdint.h>

typedef struct {
  bool ready, valid;
  float bias_rad_s, rate_rad_s, baseline_std_rad_s;
  uint32_t sample_time_ms, sample_sequence, epoch, baseline_samples;
} ChassisYawFeedback;

void ChassisYawFeedback_Init(void);
/* Called only by motorTask. Stationary must include disabled motor outputs,
 * fresh zero-delta wheel feedback and a quiet accelerometer. RAM-only offset. */
void ChassisYawFeedback_Observe(uint32_t now_ms,bool stationary,bool valid,
  float rate_rad_s,uint32_t sample_ms,uint32_t sequence,uint32_t epoch);
bool ChassisYawFeedback_Get(ChassisYawFeedback *feedback);
#endif
