#ifndef PIVOT_FEEDBACK_POLICY_H
#define PIVOT_FEEDBACK_POLICY_H

#include <stdbool.h>
#include <stdint.h>

/* OFFLINE PROTOTYPE ONLY. Not linked into production firmware. These are
 * software experiment limits, NOT a validated electrical/thermal envelope. */
#define PFP_STARTUP_MS 1000U
#define PFP_NO_PROGRESS_MS 750U
#define PFP_SEVERE_MS 1500U
#define PFP_WARNING_MS 150U
#define PFP_RECOVER_MS 100U
#define PFP_MAX_AGE_MS 100U

typedef enum {
  PFP_NORMAL, PFP_WARNING, PFP_FAULT_NO_PROGRESS,
  PFP_FAULT_SEVERE, PFP_FAULT_EVIDENCE, PFP_NOT_SUPERVISED
} PfpResult;

typedef struct {
  uint32_t now_ms, sample_ms, sample_period_ms;
  int64_t count;
  int32_t delta;
  float target_cps, measured_cps;
  int16_t applied_pwm;
  bool active, sample_valid;
} PfpInput;

/* One independent state per wheel; no wheel/direction-specific branches. */
typedef struct {
  bool started, watching, have_sample, severe, recovering;
  int8_t sign;
  uint32_t started_ms, last_progress_ms, severe_since_ms, recovering_since_ms;
  uint32_t sample_ms, sample_period_ms;
  int64_t count;
  int32_t delta;
  PfpResult result;
} PfpState;

void Pfp_Init(PfpState *state);
PfpResult Pfp_Update(PfpState *state, const PfpInput *input);
bool Pfp_IsFault(PfpResult result);

#endif
