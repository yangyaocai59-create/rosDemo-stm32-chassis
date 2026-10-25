#include "chassis_yaw_feedback.h"
#include <stddef.h>
#include <math.h>
#include "FreeRTOS.h"
#include "task.h"

static ChassisYawFeedback s_status;
/* Only Observe writes this private accumulator; getters copy publication only. */
static uint32_t s_count,s_start,s_previous_time,s_previous_sequence,s_epoch;
static float s_mean,s_m2,s_previous_rate;
static bool s_previous;

void ChassisYawFeedback_Init(void)
{
  taskENTER_CRITICAL(); s_status=(ChassisYawFeedback){0}; taskEXIT_CRITICAL();
  s_count=s_start=s_previous_time=s_previous_sequence=s_epoch=0U;
  s_mean=s_m2=s_previous_rate=0.0f; s_previous=false;
}

void ChassisYawFeedback_Observe(uint32_t now_ms,bool stationary,bool valid,
  float rate,uint32_t sample_ms,uint32_t sequence,uint32_t epoch)
{
  ChassisYawFeedback next;
  taskENTER_CRITICAL(); next=s_status; taskEXIT_CRITICAL();
  const bool fresh=valid && isfinite(rate) && fabsf(rate)<=3.0f &&
                   (uint32_t)(now_ms-sample_ms)<=40U;
  const bool source_changed=s_previous && epoch!=s_epoch;
  const bool duplicate=s_previous && sample_ms==s_previous_time && sequence==s_previous_sequence;
  const bool conflicting_duplicate=duplicate && rate!=s_previous_rate;
  const bool advancing=!s_previous || ((int32_t)(sequence-s_previous_sequence)>0 &&
                      (uint32_t)(sequence-s_previous_sequence)<=4U &&
                      (uint32_t)(sample_ms-s_previous_time)<=40U && sample_ms!=s_previous_time);
  if (!fresh || source_changed || conflicting_duplicate || (!duplicate && !advancing)) {
    next.ready=false; next.valid=false; s_count=0U; s_mean=s_m2=0.0f;
  }
  if (fresh && !source_changed && !conflicting_duplicate && (advancing || duplicate)) {
    next.valid=true;
    if (!duplicate) {
      if (!stationary || fabsf(rate)>0.15f) {
        s_count=0U; s_mean=s_m2=0.0f;
      } else {
        if (s_count==0U) s_start=sample_ms;
        ++s_count;
        const float delta=rate-s_mean;
        s_mean+=delta/(float)s_count;
        s_m2+=delta*(rate-s_mean);
        if ((uint32_t)(sample_ms-s_start)>=3000U && s_count>=100U) {
          const float sd=sqrtf(fmaxf(0.0f,s_m2/(float)s_count));
          next.baseline_samples=s_count; next.baseline_std_rad_s=sd;
          next.ready=sd<=0.003f;
          if (next.ready) next.bias_rad_s=s_mean;
          s_count=0U; s_mean=s_m2=0.0f;
        }
      }
    }
  }
  next.rate_rad_s=next.ready && fresh ? rate-next.bias_rad_s : 0.0f;
  next.sample_time_ms=sample_ms; next.sample_sequence=sequence; next.epoch=epoch;
  s_previous=true; s_previous_time=sample_ms; s_previous_sequence=sequence; s_epoch=epoch;
  s_previous_rate=rate;
  taskENTER_CRITICAL(); s_status=next; taskEXIT_CRITICAL();
}

bool ChassisYawFeedback_Get(ChassisYawFeedback *feedback)
{
  if (!feedback) return false;
  taskENTER_CRITICAL(); *feedback=s_status; taskEXIT_CRITICAL(); return true;
}
