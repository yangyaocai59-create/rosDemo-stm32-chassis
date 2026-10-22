/**
 * @file wheel_speed_controller.c
 * @brief Per-wheel direction-aware feedforward plus PI speed control.
 */

#include "wheel_speed_controller.h"

#include <stddef.h>
#include <math.h>

#include "wheel_speed_control_config.h"

typedef struct
{
  float forward_static_permille;
  float forward_gain_per_cps;
  float reverse_static_permille;
  float reverse_gain_per_cps;
  float pivot_forward_load_permille;
  float pivot_reverse_load_permille;
} WheelSpeedFeedforwardConfig;

static const WheelSpeedFeedforwardConfig s_feedforward[MOTOR_DRIVER_WHEEL_COUNT] = {
  [MOTOR_DRIVER_WHEEL_LF] = {
    WHEEL_SPEED_FF_LF_FWD_STATIC_PERMILLE,
    WHEEL_SPEED_FF_LF_FWD_GAIN_PER_CPS,
    WHEEL_SPEED_FF_LF_REV_STATIC_PERMILLE,
    WHEEL_SPEED_FF_LF_REV_GAIN_PER_CPS,
    WHEEL_SPEED_PIVOT_LOAD_LF_FWD_PERMILLE,
    WHEEL_SPEED_PIVOT_LOAD_LF_REV_PERMILLE
  },
  [MOTOR_DRIVER_WHEEL_LR] = {
    WHEEL_SPEED_FF_LR_FWD_STATIC_PERMILLE,
    WHEEL_SPEED_FF_LR_FWD_GAIN_PER_CPS,
    WHEEL_SPEED_FF_LR_REV_STATIC_PERMILLE,
    WHEEL_SPEED_FF_LR_REV_GAIN_PER_CPS,
    WHEEL_SPEED_PIVOT_LOAD_LR_FWD_PERMILLE,
    WHEEL_SPEED_PIVOT_LOAD_LR_REV_PERMILLE
  },
  [MOTOR_DRIVER_WHEEL_RF] = {
    WHEEL_SPEED_FF_RF_FWD_STATIC_PERMILLE,
    WHEEL_SPEED_FF_RF_FWD_GAIN_PER_CPS,
    WHEEL_SPEED_FF_RF_REV_STATIC_PERMILLE,
    WHEEL_SPEED_FF_RF_REV_GAIN_PER_CPS,
    WHEEL_SPEED_PIVOT_LOAD_RF_FWD_PERMILLE,
    WHEEL_SPEED_PIVOT_LOAD_RF_REV_PERMILLE
  },
  [MOTOR_DRIVER_WHEEL_RR] = {
    WHEEL_SPEED_FF_RR_FWD_STATIC_PERMILLE,
    WHEEL_SPEED_FF_RR_FWD_GAIN_PER_CPS,
    WHEEL_SPEED_FF_RR_REV_STATIC_PERMILLE,
    WHEEL_SPEED_FF_RR_REV_GAIN_PER_CPS,
    WHEEL_SPEED_PIVOT_LOAD_RR_FWD_PERMILLE,
    WHEEL_SPEED_PIVOT_LOAD_RR_REV_PERMILLE
  }
};

static WheelSpeedControllerState s_states[MOTOR_DRIVER_WHEEL_COUNT];
static float s_previous_output[MOTOR_DRIVER_WHEEL_COUNT];
static float s_previous_target[MOTOR_DRIVER_WHEEL_COUNT];
static WheelSpeedFrontPivotState s_front_pivot[MOTOR_DRIVER_WHEEL_COUNT];

/* LF reverse first-start history. No public or wire layout changes. */
typedef struct
{
  uint32_t elapsed_ms;
  uint32_t full_hold_ms;
  bool initialized;
  bool has_moved;
  bool first_recovery;
  bool consumed;
} WheelSpeedFirstStartState;
static WheelSpeedFirstStartState s_lf_first_start;

/* Maneuver-local LR reverse running recovery, not first startup. */
typedef struct
{
  float temporary_permille;
  uint32_t moving_ms, under_ms, settled_ms, episode_ms;
  bool enabled, active, spent;
} WheelSpeedLrRunningState;
static WheelSpeedLrRunningState s_lr_running;

void WheelSpeedController_ClearLrRunningSupport(void)
{
  s_lr_running = (WheelSpeedLrRunningState){0};
}

bool WheelSpeedController_SelectLrRunningSupport(bool may_start)
{
  if (may_start &&
      (s_lr_running.moving_ms >= WHEEL_SPEED_LR_RUN_MOVING_CONFIRM_MS))
  {
    s_lr_running.enabled = true;
  }
  return s_lr_running.enabled;
}

bool WheelSpeedController_IsLrRunningSupportOwned(float observed_target_cps)
{
  return s_lr_running.enabled && isfinite(observed_target_cps) &&
    (observed_target_cps <= -WHEEL_SPEED_CONTROL_PIVOT_STATIONARY_CPS) &&
    (observed_target_cps ==
     s_states[MOTOR_DRIVER_WHEEL_LR].target_counts_per_second);
}

/* RR forward policy: RR forward running recovery, not first startup. */
typedef struct
{
  float temporary_permille;
  uint32_t moving_ms, under_ms, settled_ms, episode_ms;
  bool enabled, active, spent;
} WheelSpeedRrRunningState;
static WheelSpeedRrRunningState s_rr_running;

/* No public/wire layout change, no allocation or hardware access.
 * Evidence is consumed at most once by the RR-owned PI update. */
typedef struct
{
  int32_t pending_delta, previous_delta;
  uint32_t pending_period, pending_time, previous_period, previous_time;
  uint32_t last_observed_time, confirm_ms;
  float target;
  bool pending, observed, have_previous, exiting;
} WheelSpeedRrExitState;
static WheelSpeedRrExitState s_rr_exit;

static void WheelSpeedController_InvalidateRrExitEvidence(void)
{
  s_rr_exit.pending = false;
  s_rr_exit.have_previous = false;
  s_rr_exit.confirm_ms = 0U;
}

void WheelSpeedController_ObserveRrRecoveryCounts(int32_t delta_counts,
  uint32_t sample_period_ms, uint32_t sample_time_ms, uint32_t now_ms)
{
  WheelSpeedRrExitState *fast = &s_rr_exit;
  if ((sample_period_ms < WHEEL_SPEED_RR_EXIT_SAMPLE_MIN_MS) ||
      (sample_period_ms > WHEEL_SPEED_RR_EXIT_SAMPLE_MAX_MS) ||
      ((now_ms - sample_time_ms) > sample_period_ms) ||
      (fabsf((float)delta_counts * 1000.0f / (float)sample_period_ms) >
       2.0f * WHEEL_SPEED_CONTROL_TARGET_LIMIT_CPS))
  {
    WheelSpeedController_InvalidateRrExitEvidence();
    return;
  }
  const uint32_t gap = sample_time_ms - fast->last_observed_time;
  if (fast->observed && ((gap == 0U) || (gap > 0x7FFFFFFFU)))
  {
    WheelSpeedController_InvalidateRrExitEvidence();
    return;
  }
  if (fast->observed && (gap != sample_period_ms))
    WheelSpeedController_InvalidateRrExitEvidence();
  fast->observed = true;
  fast->last_observed_time = sample_time_ms;
  fast->pending_delta = delta_counts;
  fast->pending_period = sample_period_ms;
  fast->pending_time = sample_time_ms;
  fast->pending = true;
}

void WheelSpeedController_ClearRrRunningSupport(void)
{
  s_rr_running = (WheelSpeedRrRunningState){0};
  s_rr_exit = (WheelSpeedRrExitState){0};
}

bool WheelSpeedController_SelectRrRunningSupport(bool may_start)
{
  if (may_start &&
      (s_rr_running.moving_ms >= WHEEL_SPEED_RR_RUN_MOVING_CONFIRM_MS))
  {
    s_rr_running.enabled = true;
  }
  return s_rr_running.enabled;
}

bool WheelSpeedController_IsRrRunningSupportOwned(float observed_target_cps)
{
  return s_rr_running.enabled && isfinite(observed_target_cps) &&
    (observed_target_cps >= WHEEL_SPEED_CONTROL_PIVOT_STATIONARY_CPS) &&
    (observed_target_cps ==
     s_states[MOTOR_DRIVER_WHEEL_RR].target_counts_per_second);
}

static bool WheelSpeedController_IsFront(MotorDriverWheel wheel)
{
  return (wheel == MOTOR_DRIVER_WHEEL_LF) ||
         (wheel == MOTOR_DRIVER_WHEEL_RF);
}

static void WheelSpeedController_ClearFront(MotorDriverWheel wheel)
{
  s_front_pivot[wheel] = (WheelSpeedFrontPivotState){0};
  if (wheel == MOTOR_DRIVER_WHEEL_LF)
  {
    s_lf_first_start = (WheelSpeedFirstStartState){0};
  }
  if (wheel == MOTOR_DRIVER_WHEEL_LR)
  {
    WheelSpeedController_ClearLrRunningSupport();
  }
  if (wheel == MOTOR_DRIVER_WHEEL_RR)
  {
    WheelSpeedController_ClearRrRunningSupport();
  }
}

static bool WheelSpeedController_IsWheelValid(MotorDriverWheel wheel)
{
  return ((uint32_t)wheel < (uint32_t)MOTOR_DRIVER_WHEEL_COUNT);
}

static float WheelSpeedController_Abs(float value)
{
  return (value < 0.0f) ? -value : value;
}

static float WheelSpeedController_Clamp(float value,
                                        float minimum,
                                        float maximum)
{
  if (value < minimum)
  {
    return minimum;
  }
  if (value > maximum)
  {
    return maximum;
  }
  return value;
}

static int16_t WheelSpeedController_RoundToInt16(float value)
{
  return (int16_t)((value >= 0.0f) ? (value + 0.5f) : (value - 0.5f));
}

float WheelSpeedController_GetPivotLoadFeedforward(
  MotorDriverWheel wheel, float target_counts_per_second)
{
  if ((!WheelSpeedController_IsWheelValid(wheel)) ||
      (!isfinite(target_counts_per_second)))
  {
    return NAN;
  }
  const float magnitude = WheelSpeedController_Abs(target_counts_per_second);
  if (magnitude < WHEEL_SPEED_CONTROL_ZERO_TARGET_CPS) { return 0.0f; }
  const float scale = WheelSpeedController_Clamp(
    magnitude / WHEEL_SPEED_PIVOT_LOAD_FULL_CPS, 0.0f, 1.0f);
  return target_counts_per_second > 0.0f ?
    s_feedforward[wheel].pivot_forward_load_permille * scale :
    -s_feedforward[wheel].pivot_reverse_load_permille * scale;
}

static float WheelSpeedController_GetFeedforward(MotorDriverWheel wheel,
                                                  float target_cps)
{
  const WheelSpeedFeedforwardConfig *config = &s_feedforward[wheel];
  const float magnitude = WheelSpeedController_Abs(target_cps);

  if (target_cps > 0.0f)
  {
    return config->forward_static_permille +
           config->forward_gain_per_cps * magnitude;
  }

  return -(config->reverse_static_permille +
           config->reverse_gain_per_cps * magnitude);
}

static uint32_t WheelSpeedController_AddTime(uint32_t value,
                                            uint32_t elapsed_ms,
                                            uint32_t ceiling)
{
  if ((value >= ceiling) || (elapsed_ms >= ceiling - value))
  {
    return ceiling;
  }
  return value + elapsed_ms;
}

static float WheelSpeedController_FrontSupport(MotorDriverWheel wheel,
  float target, float measured, uint32_t elapsed_ms, bool *hold_integral)
{
  WheelSpeedFrontPivotState *front = &s_front_pivot[wheel];
  WheelSpeedControllerState *control = &s_states[wheel];
  const float magnitude = WheelSpeedController_Abs(target);
  const int8_t sign = target > 0.0f ? 1 : -1;
  const float seconds = (float)elapsed_ms / 1000.0f;
  const float ceiling = WheelSpeedController_Abs(
    WheelSpeedController_GetPivotLoadFeedforward(wheel, target));
  const float speed = measured * (float)sign;
  const float error = magnitude - speed;
  const float band = fmaxf(WHEEL_SPEED_FRONT_LOAD_ERROR_MIN_CPS,
    magnitude * WHEEL_SPEED_FRONT_LOAD_ERROR_RATIO);
  const bool overspeed = error < -band;
  float load_rise = 0.0f;
  bool restoring_moving_load = false;

  if ((!front->active) || (front->sign != sign))
  {
    WheelSpeedController_ClearFront(wheel);
    front->active = true;
    front->sign = sign;
    front->load_permille = ceiling;
  }
  front->load_permille = fminf(front->load_permille, ceiling);
  const bool first_start_scope = (wheel == MOTOR_DRIVER_WHEEL_LF) && (sign < 0);
  if (first_start_scope)
  {
    if (!s_lf_first_start.initialized)
    {
      s_lf_first_start.initialized = true;
      /* Supports diagnostic seeds with an already moving policy too. */
      s_lf_first_start.has_moved =
        front->moving_ms >= WHEEL_SPEED_FRONT_RECOVERY_MOVING_MS;
    }
    s_lf_first_start.elapsed_ms = WheelSpeedController_AddTime(
      s_lf_first_start.elapsed_ms, elapsed_ms, WHEEL_SPEED_LF_FIRST_START_TOTAL_MS);
  }
  front->load_overspeed_ms = overspeed ?
    WheelSpeedController_AddTime(front->load_overspeed_ms, elapsed_ms,
      WHEEL_SPEED_FRONT_LOAD_OVERSPEED_CONFIRM_MS) : 0U;
  const float stationary = fminf(WHEEL_SPEED_CONTROL_PIVOT_STATIONARY_CPS,
    magnitude * WHEEL_SPEED_FRONT_STILL_TARGET_RATIO);
  front->still_ms = WheelSpeedController_Abs(measured) <= stationary ?
    WheelSpeedController_AddTime(front->still_ms, elapsed_ms,
      WHEEL_SPEED_FRONT_RECOVERY_CONFIRM_MS) : 0U;
  front->moving_ms = speed >= WHEEL_SPEED_CONTROL_PIVOT_RELEASE_CPS ?
    WheelSpeedController_AddTime(front->moving_ms, elapsed_ms,
      WHEEL_SPEED_FRONT_RECOVERY_MOVING_MS) : 0U;
  if (first_start_scope &&
      (front->moving_ms >= WHEEL_SPEED_FRONT_RECOVERY_MOVING_MS))
  {
    s_lf_first_start.has_moved = true;
  }
  if (!front->recovery_failed)
  {
    front->no_motion_ms = WheelSpeedController_Abs(measured) <= stationary ?
      WheelSpeedController_AddTime(front->no_motion_ms, elapsed_ms,
        WHEEL_SPEED_FRONT_NO_MOTION_LIMIT_MS) : 0U;
  }

  if ((!front->recovery_failed) &&
      (front->moving_ms >= WHEEL_SPEED_FRONT_RECOVERY_MOVING_MS))
  {
    front->exhausted = false;
  }
  if ((front->still_ms >= WHEEL_SPEED_FRONT_RECOVERY_CONFIRM_MS) &&
      (!front->exhausted) && (!front->recovery_failed))
  {
    front->recovering = true;
    front->exhausted = true;
    front->recovery_ms = 0U;
    if (first_start_scope && !s_lf_first_start.has_moved &&
        !s_lf_first_start.consumed &&
        (s_lf_first_start.elapsed_ms + WHEEL_SPEED_FRONT_RECOVERY_MAX_MS <=
         WHEEL_SPEED_LF_FIRST_START_TOTAL_MS))
    {
      s_lf_first_start.first_recovery = true;
      s_lf_first_start.full_hold_ms = 0U;
    }
    /* A braking correction from an earlier overspeed is not a useful
     * preload at confirmed near-rest. Never preload positive integral. */
    if (control->integrator_permille * (float)sign < 0.0f)
    {
      control->integrator_permille = 0.0f;
    }
    /* Keep useful positive I. Recovery may borrow only the unused shared
     * headroom; existing steady support must not become a disposable kick. */
  }
  else if (front->recovering)
  {
    front->recovery_ms = WheelSpeedController_AddTime(front->recovery_ms,
      elapsed_ms, WHEEL_SPEED_FRONT_RECOVERY_MAX_MS);
  }

  if (overspeed)
  {
    /* Short speed excursions still receive the existing P damping and I
     * unwind below. Only the extra running-load withdrawal waits for
     * consecutive overspeed. A broken qualification starts from zero. */
    if (front->load_overspeed_ms >=
        WHEEL_SPEED_FRONT_LOAD_OVERSPEED_CONFIRM_MS)
    {
      const float rate = WheelSpeedController_Clamp(
        (-error - band) * WHEEL_SPEED_FRONT_COORD_LOAD_ERROR_GAIN,
        WHEEL_SPEED_FRONT_COORD_LOAD_WITHDRAW_MIN_PER_SECOND,
        WHEEL_SPEED_FRONT_LOAD_WITHDRAW_PER_SECOND);
      const float fading = fminf(front->recovery_permille,
        WHEEL_SPEED_FRONT_RECOVERY_DECAY_PER_SECOND * seconds);
      /* Both fronts share ONE aggregate fall budget with their temporary
       * term. RF now uses the same bounded policy as the unchanged LF. */
      const float budget = fminf(rate * seconds, fmaxf(0.0f,
        WHEEL_SPEED_FRONT_COORD_SUPPORT_FALL_PER_SECOND * seconds - fading));
      const float removed = fminf(front->load_permille, budget);
      front->load_permille -= removed;
      /* Preserve opposing-I debt handoff without a double drop in FF+I.
       * No positive integral is created by withdrawing running support. */
      const float opposing = -control->integrator_permille * (float)sign;
      if (opposing > 0.0f)
      {
        control->integrator_permille += (float)sign * fminf(removed, opposing);
      }
      else
      {
        /* After load is gone, the SAME withdrawal budget can remove positive
         * steady I inherited from recovery. Never leave a fixed I floor on
         * unload, and never spend a full budget independently per bucket. */
        control->integrator_permille -= (float)sign * fminf(
          control->integrator_permille * (float)sign, budget - removed);
      }
    }
  }
  else if (front->still_ms >= WHEEL_SPEED_FRONT_RECOVERY_CONFIRM_MS)
  {
    /* A wheel that loses ground motion can need its old load ceiling again.
     * Restore it at a bounded rate; do not rely on growing integral at rest. */
    load_rise = fminf(ceiling - front->load_permille,
      WHEEL_SPEED_FRONT_LOAD_RESTORE_PER_SECOND * seconds);
    front->load_permille += load_rise;
  }
  else if ((!front->recovering) && (front->recovery_permille == 0.0f) &&
           (speed >= WHEEL_SPEED_CONTROL_PIVOT_RELEASE_CPS) && (error > band))
  {
    const float support = control->integrator_permille * (float)sign;
    if ((front->moving_ms >= WHEEL_SPEED_FRONT_RECOVERY_MOVING_MS) &&
        (!front->recovery_failed) && (support >= 0.0f))
    {
      const float rate = fminf(WHEEL_SPEED_FRONT_LOAD_REBASE_PER_SECOND,
        (error - band) * WHEEL_SPEED_FRONT_COORD_LOAD_ERROR_GAIN);
      load_rise = fminf(ceiling - front->load_permille, rate * seconds);
      front->load_permille += load_rise;
      /* Bounded added support, not an I transfer: RF need not wait for
       * >10 permille of I before recovering its existing load ceiling.
       * No simultaneous positive PI integration on that update. */
      restoring_moving_load = load_rise > 0.0f;
    }
  }

  const bool first_recovery = first_start_scope && s_lf_first_start.first_recovery;
  const bool pulse_expired = first_recovery ?
    ((s_lf_first_start.elapsed_ms >= WHEEL_SPEED_LF_FIRST_START_TOTAL_MS) ||
     ((front->recovery_ms >= WHEEL_SPEED_FRONT_RECOVERY_MAX_MS) &&
      (s_lf_first_start.full_hold_ms >= WHEEL_SPEED_LF_FIRST_START_FULL_HOLD_MS))) :
    (front->recovery_ms >= WHEEL_SPEED_FRONT_RECOVERY_MAX_MS);
  if (front->recovering &&
      (pulse_expired ||
       (front->moving_ms >= WHEEL_SPEED_FRONT_RECOVERY_MOVING_MS) || overspeed))
  {
    front->recovering = false;
    if (first_recovery)
    {
      s_lf_first_start.first_recovery = false;
      s_lf_first_start.consumed = true;
    }
  }
  bool handed_off = false;
  if ((!front->recovering) && (!front->recovery_failed) &&
      (front->moving_ms >= WHEEL_SPEED_FRONT_RECOVERY_MOVING_MS) &&
      (error >= 0.0f) && (front->recovery_permille > 0.0f))
  {
    /* Reclassify support, do not add effort: FF+I is conserved. This path is
     * allowed only after real same-direction motion, never while stalled.
     * Suppress ordinary positive error integration on this very update too. */
    const float room = fmaxf(0.0f,
      WHEEL_SPEED_CONTROL_INTEGRATOR_LIMIT_PERMILLE -
      control->integrator_permille * (float)sign);
    const float transfer = fminf(front->recovery_permille, fminf(room,
      WHEEL_SPEED_FRONT_LOAD_REBASE_PER_SECOND * seconds));
    control->integrator_permille += (float)sign * transfer;
    front->recovery_permille -= transfer;
    handed_off = true;
  }
  const float scale = fminf(magnitude / WHEEL_SPEED_PIVOT_LOAD_FULL_CPS, 1.0f);
  const float authority = WHEEL_SPEED_CONTROL_INTEGRATOR_LIMIT_PERMILLE * scale;
  const float signed_i = control->integrator_permille * (float)sign;
  const float headroom = WheelSpeedController_Clamp(
    WHEEL_SPEED_CONTROL_INTEGRATOR_LIMIT_PERMILLE - signed_i, 0.0f, authority);
  if (front->recovering)
  {
    const float rise = fminf(
      WHEEL_SPEED_FRONT_RECOVERY_RISE_PER_SECOND * seconds, fmaxf(0.0f,
        WHEEL_SPEED_FRONT_COORD_SUPPORT_RISE_PER_SECOND * seconds - load_rise));
    front->recovery_permille = fminf(headroom, front->recovery_permille +
      rise);
  }
  else if (!handed_off)
  {
    front->recovery_permille = fmaxf(0.0f, front->recovery_permille -
      WHEEL_SPEED_FRONT_RECOVERY_DECAY_PER_SECOND * seconds);
    front->recovery_permille = fminf(front->recovery_permille, headroom);
  }
  /* Clamp after transfers too: round-off must not strand a tiny transient
   * when the shared integral budget has become exactly full. */
  front->recovery_permille = fminf(front->recovery_permille, headroom);
  if (first_start_scope && s_lf_first_start.first_recovery && front->recovering)
  {
    const bool full = (front->load_permille >= ceiling - 0.001f) &&
      (front->recovery_permille >= headroom - 0.001f);
    s_lf_first_start.full_hold_ms = full ? WheelSpeedController_AddTime(
      s_lf_first_start.full_hold_ms, elapsed_ms,
      WHEEL_SPEED_LF_FIRST_START_FULL_HOLD_MS) : 0U;
  }
  if ((front->no_motion_ms >= WHEEL_SPEED_FRONT_NO_MOTION_LIMIT_MS) &&
      front->exhausted && (!front->recovering) &&
      (front->recovery_permille == 0.0f))
  {
    /* Sticky until reset/sign/mode exit: a late sample of motion must not hide
     * a completed failure before safetyTask observes it. This is a failure
     * report, not a new retry or extra drive authority. FaultManager consumes
     * it independently of its old >=200 cps/output>=100 stall gate. */
    front->recovery_failed = true;
  }
  /* No retry train at rest. Recovery failure is consumed by safetyTask on its
   * next observation, then existing runtime shutdown clears all controllers. */
  *hold_integral = (front->still_ms >= WHEEL_SPEED_FRONT_RECOVERY_CONFIRM_MS) ||
    front->recovering || (front->recovery_permille > 0.0f) || handed_off ||
    restoring_moving_load;
  return (float)sign * (front->load_permille + front->recovery_permille);
}

static float WheelSpeedController_LrRunningSupport(float target, float measured,
  uint32_t elapsed_ms, bool *hold_integral)
{
  WheelSpeedLrRunningState *lr = &s_lr_running;
  WheelSpeedControllerState *control = &s_states[MOTOR_DRIVER_WHEEL_LR];
  const bool was_controlled = lr->active || (lr->temporary_permille > 0.0f);
  const float magnitude = -target;
  const float speed = -measured;
  const float error = magnitude - speed;
  const float band = fmaxf(20.0f, magnitude * 0.1f);
  const float stationary = fminf(20.0f, magnitude * 0.25f);
  const float seconds = (float)elapsed_ms / 1000.0f;
  const bool underspeed = (speed >= 0.0f) &&
    ((error > band) || (fabsf(measured) <= stationary));
  const bool overspeed = speed > magnitude;
  const bool settled = (speed >= 50.0f) && (fabsf(error) <= band);
  lr->under_ms = underspeed ? WheelSpeedController_AddTime(lr->under_ms,
    elapsed_ms, WHEEL_SPEED_LR_RUN_UNDER_CONFIRM_MS) : 0U;
  lr->settled_ms = settled ? WheelSpeedController_AddTime(lr->settled_ms,
    elapsed_ms, WHEEL_SPEED_LR_RUN_REARM_MS) : 0U;
  if (lr->spent)
  {
    lr->episode_ms = WheelSpeedController_AddTime(lr->episode_ms, elapsed_ms,
      WHEEL_SPEED_LR_RUN_EPISODE_MS);
  }
  if ((!lr->spent) && (lr->under_ms >= WHEEL_SPEED_LR_RUN_UNDER_CONFIRM_MS))
  {
    lr->active = true;
    lr->spent = true;
    lr->episode_ms = elapsed_ms;
  }
  if (overspeed || (speed < 0.0f) ||
      (lr->episode_ms >= WHEEL_SPEED_LR_RUN_EPISODE_MS) ||
      (lr->settled_ms >= WHEEL_SPEED_LR_RUN_MOVING_CONFIRM_MS))
  {
    lr->active = false;
  }
  const float authority = WHEEL_SPEED_CONTROL_PIVOT_BREAKAWAY_PERMILLE;
  const float headroom = fmaxf(0.0f,
    authority - fmaxf(0.0f, -control->integrator_permille));
  lr->temporary_permille = fminf(lr->temporary_permille, headroom);
  if (lr->active && underspeed)
  {
    const float rate = fminf(WHEEL_SPEED_LR_RUN_RISE_PER_SECOND,
      fmaxf(0.0f, error - band) * WHEEL_SPEED_LR_RUN_ERROR_GAIN);
    /* Small targets still receive finite gradual near-rest support. */
    const float rise = fabsf(measured) <= stationary ?
      WHEEL_SPEED_LR_RUN_RISE_PER_SECOND : rate;
    lr->temporary_permille = fminf(headroom,
      lr->temporary_permille + rise * seconds);
  }
  bool transferred = false;
  if ((!overspeed) && (speed >= 50.0f) &&
      (lr->moving_ms >= WHEEL_SPEED_LR_RUN_MOVING_CONFIRM_MS) &&
      (lr->episode_ms < WHEEL_SPEED_LR_RUN_EPISODE_MS) &&
      (lr->temporary_permille > 0.0f))
  {
    /* Reclassify, do not add: signed I + temporary is exactly conserved.
     * This can cancel opposing I but does not clear it in a single jump. */
    const float room = fmaxf(0.0f,
      WHEEL_SPEED_CONTROL_INTEGRATOR_LIMIT_PERMILLE + control->integrator_permille);
    const float transfer = fminf(lr->temporary_permille,
      fminf(room, WHEEL_SPEED_LR_RUN_HANDOFF_PER_SECOND * seconds));
    control->integrator_permille -= transfer;
    lr->temporary_permille -= transfer;
    transferred = transfer > 0.0f;
  }
  else if (!lr->active)
  {
    lr->temporary_permille = fmaxf(0.0f, lr->temporary_permille -
      WHEEL_SPEED_LR_RUN_FALL_PER_SECOND * seconds);
  }
  if ((lr->settled_ms >= WHEEL_SPEED_LR_RUN_REARM_MS) &&
      (lr->temporary_permille == 0.0f))
  {
    lr->spent = false;
    lr->episode_ms = 0U;
    lr->under_ms = 0U;
  }
  *hold_integral = was_controlled || lr->active ||
    (lr->temporary_permille > 0.0f) || transferred;
  return -lr->temporary_permille;
}

static bool WheelSpeedController_RrExitEvidence(float target, float measured,
  bool *accelerating)
{
  WheelSpeedRrExitState *fast = &s_rr_exit;
  *accelerating = false;
  if (target != fast->target)
  {
    fast->have_previous = false;
    fast->confirm_ms = 0U;
    fast->target = target;
  }
  if (!fast->pending)
  {
    WheelSpeedController_InvalidateRrExitEvidence();
    return false;
  }
  fast->pending = false;
  bool over = false;
  const float band = fmaxf(20.0f, target * 0.1f);
  if (fast->have_previous &&
      (fast->pending_time - fast->previous_time == fast->pending_period) &&
      (fast->previous_delta > 0) && (fast->pending_delta > 0))
  {
    const uint32_t span = fast->pending_period + fast->previous_period;
    const float window_speed =
      ((float)((int64_t)fast->previous_delta + fast->pending_delta) * 1000.0f) /
      (float)span;
    const float current_speed = (float)fast->pending_delta * 1000.0f /
      (float)fast->pending_period;
    /* A lone spike cannot qualify on the following low/zero sample:
     * BOTH the window and the newest independently timed sample must exceed
     * the margin. A rising near-target window may only PAUSE effort growth. */
    over = (window_speed > target + band) && (current_speed > target + band);
    *accelerating = (window_speed > measured + band) &&
      (window_speed >= target - band) && (current_speed >= target);
  }
  fast->confirm_ms = over ? WheelSpeedController_AddTime(fast->confirm_ms,
    fast->pending_period, WHEEL_SPEED_RR_EXIT_CONFIRM_MS) : 0U;
  fast->previous_delta = fast->pending_delta;
  fast->previous_period = fast->pending_period;
  fast->previous_time = fast->pending_time;
  fast->have_previous = true;
  return fast->confirm_ms >= WHEEL_SPEED_RR_EXIT_CONFIRM_MS;
}

static float WheelSpeedController_RrRunningSupport(float target, float measured,
  uint32_t elapsed_ms, bool *hold_integral)
{
  WheelSpeedRrRunningState *rr = &s_rr_running;
  WheelSpeedControllerState *control = &s_states[MOTOR_DRIVER_WHEEL_RR];
  const bool was_controlled = rr->active || (rr->temporary_permille > 0.0f);
  const float magnitude = target;
  const float speed = measured;
  const float error = magnitude - speed;
  const float band = fmaxf(20.0f, magnitude * 0.1f);
  const float stationary = fminf(20.0f, magnitude * 0.25f);
  const float seconds = (float)elapsed_ms / 1000.0f;
  const bool underspeed = (speed >= 0.0f) &&
    ((error > band) || (fabsf(measured) <= stationary));
  const bool overspeed = speed > magnitude;
  bool accelerating;
  const bool fast_over = WheelSpeedController_RrExitEvidence(target, measured,
    &accelerating);
  /* Do not rearm from an apparently settled LPF while fresh counts still
   * contradict it. Missing evidence retains the original filtered fallback. */
  const bool settled = (speed >= 50.0f) && (fabsf(error) <= band) &&
    !fast_over && !accelerating;
  if (fast_over && (rr->active || (rr->temporary_permille > 0.0f)))
    s_rr_exit.exiting = true;
  rr->under_ms = underspeed ? WheelSpeedController_AddTime(rr->under_ms,
    elapsed_ms, WHEEL_SPEED_RR_RUN_UNDER_CONFIRM_MS) : 0U;
  rr->settled_ms = settled ? WheelSpeedController_AddTime(rr->settled_ms,
    elapsed_ms, WHEEL_SPEED_RR_RUN_REARM_MS) : 0U;
  if (rr->spent)
  {
    rr->episode_ms = WheelSpeedController_AddTime(rr->episode_ms, elapsed_ms,
      WHEEL_SPEED_RR_RUN_EPISODE_MS);
  }
  if ((!rr->spent) && (rr->under_ms >= WHEEL_SPEED_RR_RUN_UNDER_CONFIRM_MS))
  {
    rr->active = true;
    rr->spent = true;
    rr->episode_ms = elapsed_ms;
  }
  if (s_rr_exit.exiting || overspeed || (speed < 0.0f) ||
      (rr->episode_ms >= WHEEL_SPEED_RR_RUN_EPISODE_MS) ||
      (rr->settled_ms >= WHEEL_SPEED_RR_RUN_MOVING_CONFIRM_MS))
  {
    rr->active = false;
  }
  const float authority = WHEEL_SPEED_CONTROL_PIVOT_BREAKAWAY_PERMILLE;
  const float headroom = fmaxf(0.0f,
    authority - fmaxf(0.0f, control->integrator_permille));
  rr->temporary_permille = fminf(rr->temporary_permille, headroom);
  if (rr->active && underspeed && !accelerating)
  {
    const float rate = fminf(WHEEL_SPEED_RR_RUN_RISE_PER_SECOND,
      fmaxf(0.0f, error - band) * WHEEL_SPEED_RR_RUN_ERROR_GAIN);
    /* Small targets still receive finite gradual near-rest support. */
    const float rise = fabsf(measured) <= stationary ?
      WHEEL_SPEED_RR_RUN_RISE_PER_SECOND : rate;
    rr->temporary_permille = fminf(headroom,
      rr->temporary_permille + rise * seconds);
  }
  bool transferred = false;
  if ((!s_rr_exit.exiting) && (!accelerating) && (!overspeed) && (speed >= 50.0f) &&
      (rr->moving_ms >= WHEEL_SPEED_RR_RUN_MOVING_CONFIRM_MS) &&
      (rr->episode_ms < WHEEL_SPEED_RR_RUN_EPISODE_MS) &&
      (rr->temporary_permille > 0.0f))
  {
    /* Reclassify, do not add: signed I + temporary is exactly conserved.
     * This can cancel opposing I but does not clear it in a single jump. */
    const float room = fmaxf(0.0f,
      WHEEL_SPEED_CONTROL_INTEGRATOR_LIMIT_PERMILLE - control->integrator_permille);
    const float transfer = fminf(rr->temporary_permille,
      fminf(room, WHEEL_SPEED_RR_RUN_HANDOFF_PER_SECOND * seconds));
    control->integrator_permille += transfer;
    rr->temporary_permille -= transfer;
    transferred = transfer > 0.0f;
  }
  else if (!rr->active)
  {
    rr->temporary_permille = fmaxf(0.0f, rr->temporary_permille -
      WHEEL_SPEED_RR_RUN_FALL_PER_SECOND * seconds);
  }
  if ((rr->settled_ms >= WHEEL_SPEED_RR_RUN_REARM_MS) &&
      (rr->temporary_permille == 0.0f))
  {
    rr->spent = false;
    rr->episode_ms = 0U;
    rr->under_ms = 0U;
    s_rr_exit.exiting = false;
    s_rr_exit.have_previous = false;
    s_rr_exit.confirm_ms = 0U;
  }
  *hold_integral = was_controlled || rr->active ||
    (rr->temporary_permille > 0.0f) || transferred ||
    (s_rr_exit.exiting && fast_over);
  return rr->temporary_permille;
}

void WheelSpeedController_Init(void)
{
  for (uint32_t wheel = 0U; wheel < MOTOR_DRIVER_WHEEL_COUNT; ++wheel)
  {
    (void)WheelSpeedController_Reset((MotorDriverWheel)wheel);
  }
}

bool WheelSpeedController_Reset(MotorDriverWheel wheel)
{
  if (!WheelSpeedController_IsWheelValid(wheel))
  {
    return false;
  }

  s_states[wheel].target_counts_per_second = 0.0f;
  s_states[wheel].measured_counts_per_second = 0.0f;
  s_states[wheel].error_counts_per_second = 0.0f;
  s_states[wheel].feedforward_permille = 0.0f;
  s_states[wheel].proportional_permille = 0.0f;
  s_states[wheel].integrator_permille = 0.0f;
  s_states[wheel].command_permille = 0;
  s_states[wheel].limited = false;
  s_previous_output[wheel] = 0.0f;
  s_previous_target[wheel] = 0.0f;
  WheelSpeedController_ClearFront(wheel);
  return true;
}

bool WheelSpeedController_PrimeOutput(MotorDriverWheel wheel,
                                      int16_t command_permille)
{
  float command;

  if (!WheelSpeedController_IsWheelValid(wheel))
  {
    return false;
  }

  command = WheelSpeedController_Clamp(
    (float)command_permille,
    -(float)WHEEL_SPEED_CONTROL_OUTPUT_LIMIT_PERMILLE,
    (float)WHEEL_SPEED_CONTROL_OUTPUT_LIMIT_PERMILLE);
  s_previous_output[wheel] = command;
  s_previous_target[wheel] = 0.0f;
  s_states[wheel].command_permille =
    WheelSpeedController_RoundToInt16(command);
  return true;
}

float WheelSpeedController_ReleaseFeedforwardBias(
  MotorDriverWheel wheel, float feedforward_bias_permille)
{
  if ((!WheelSpeedController_IsWheelValid(wheel)) ||
      (!isfinite(feedforward_bias_permille)) ||
      (WheelSpeedController_Abs(feedforward_bias_permille) >
       (float)WHEEL_SPEED_CONTROL_OUTPUT_LIMIT_PERMILLE))
  {
    return NAN;
  }

  WheelSpeedControllerState *state = &s_states[wheel];
  const float combined = state->integrator_permille +
    feedforward_bias_permille;
  state->integrator_permille = WheelSpeedController_Clamp(
    combined,
    -WHEEL_SPEED_CONTROL_INTEGRATOR_LIMIT_PERMILLE,
    WHEEL_SPEED_CONTROL_INTEGRATOR_LIMIT_PERMILLE);

  /* Keep any excess as maneuver-local load support rather than dropping it
   * when the PI integral reaches its limit. Normal PI error integration can
   * unwind that residual on following updates. It never survives Stop/Reset. */
  return combined - state->integrator_permille;
}

bool WheelSpeedController_UpdateWithMode(
  MotorDriverWheel wheel,
  float target_counts_per_second,
  float measured_counts_per_second,
  float feedforward_bias_permille,
  WheelSpeedControlMode mode,
  uint32_t elapsed_ms,
  int16_t *command_permille)
{
  WheelSpeedControllerState *state;
  float target;
  float error;
  float feedforward;
  float proportional;
  float proportional_gain;
  float previous_integrator;
  float candidate_integrator;
  float desired_output;
  float limited_output;
  float maximum_step;
  float minimum_output = -(float)WHEEL_SPEED_CONTROL_OUTPUT_LIMIT_PERMILLE;
  float maximum_output = (float)WHEEL_SPEED_CONTROL_OUTPUT_LIMIT_PERMILLE;
  bool magnitude_limited;
  bool slew_limited;
  bool front_hold_integral = false;

  if ((!WheelSpeedController_IsWheelValid(wheel)) ||
      (command_permille == NULL) ||
      ((mode != WHEEL_SPEED_CONTROL_MODE_NORMAL) &&
       (mode != WHEEL_SPEED_CONTROL_MODE_PIVOT)) ||
      (!isfinite(target_counts_per_second)) ||
      (!isfinite(measured_counts_per_second)) ||
      (feedforward_bias_permille != feedforward_bias_permille) ||
      (feedforward_bias_permille >
       (float)WHEEL_SPEED_CONTROL_OUTPUT_LIMIT_PERMILLE) ||
      (feedforward_bias_permille <
       -(float)WHEEL_SPEED_CONTROL_OUTPUT_LIMIT_PERMILLE) ||
      (elapsed_ms == 0U) || (elapsed_ms > 100U))
  {
    return false;
  }

  /* Explicit owner contract: never stack a caller's legacy bias on this
   * running policy. Reject before state changes, as with other bad inputs. */
  if ((wheel == MOTOR_DRIVER_WHEEL_LR) && s_lr_running.enabled &&
      (mode == WHEEL_SPEED_CONTROL_MODE_PIVOT) &&
      (target_counts_per_second <= -20.0f) && (feedforward_bias_permille != 0.0f))
  {
    return false;
  }

  state = &s_states[wheel];
  target = WheelSpeedController_Clamp(
    target_counts_per_second,
    -WHEEL_SPEED_CONTROL_TARGET_LIMIT_CPS,
    WHEEL_SPEED_CONTROL_TARGET_LIMIT_CPS);

  if ((wheel == MOTOR_DRIVER_WHEEL_RR) && s_rr_running.enabled &&
      (mode == WHEEL_SPEED_CONTROL_MODE_PIVOT) &&
      (target >= 20.0f) && (feedforward_bias_permille != 0.0f))
  {
    return false;
  }
  if (wheel == MOTOR_DRIVER_WHEEL_RR)
  {
    if ((mode != WHEEL_SPEED_CONTROL_MODE_PIVOT) ||
        (target < WHEEL_SPEED_CONTROL_PIVOT_STATIONARY_CPS))
    {
      WheelSpeedController_ClearRrRunningSupport();
    }
    else
    {
      s_rr_running.moving_ms = measured_counts_per_second >= 50.0f ?
        WheelSpeedController_AddTime(s_rr_running.moving_ms, elapsed_ms,
          WHEEL_SPEED_RR_RUN_MOVING_CONFIRM_MS) : 0U;
    }
  }

  if (wheel == MOTOR_DRIVER_WHEEL_LR)
  {
    if ((mode != WHEEL_SPEED_CONTROL_MODE_PIVOT) ||
        (target > -WHEEL_SPEED_CONTROL_PIVOT_STATIONARY_CPS))
    {
      WheelSpeedController_ClearLrRunningSupport();
    }
    else
    {
      s_lr_running.moving_ms = -measured_counts_per_second >= 50.0f ?
        WheelSpeedController_AddTime(s_lr_running.moving_ms, elapsed_ms,
          WHEEL_SPEED_LR_RUN_MOVING_CONFIRM_MS) : 0U;
    }
  }

  if ((mode != WHEEL_SPEED_CONTROL_MODE_PIVOT) ||
      (WheelSpeedController_Abs(target) <
       WHEEL_SPEED_CONTROL_PIVOT_STATIONARY_CPS))
  {
    WheelSpeedController_ClearFront(wheel);
  }

  if (WheelSpeedController_Abs(target) <
      WHEEL_SPEED_CONTROL_ZERO_TARGET_CPS)
  {
    /* Normal target crossing is not an emergency stop. Keep output continuity;
     * ChassisController_Stop/Reset still remove drive immediately. */
    maximum_step = WHEEL_SPEED_CONTROL_SLEW_PERMILLE_PER_SECOND *
      ((float)elapsed_ms / 1000.0f);
    limited_output = WheelSpeedController_Clamp(0.0f,
      s_previous_output[wheel] - maximum_step,
      s_previous_output[wheel] + maximum_step);
    state->target_counts_per_second = 0.0f;
    state->measured_counts_per_second = measured_counts_per_second;
    state->error_counts_per_second = -measured_counts_per_second;
    state->feedforward_permille = 0.0f;
    state->proportional_permille = 0.0f;
    state->integrator_permille = 0.0f;
    state->command_permille = WheelSpeedController_RoundToInt16(limited_output);
    state->limited = (limited_output != 0.0f);
    s_previous_output[wheel] = limited_output;
    s_previous_target[wheel] = 0.0f;
    *command_permille = state->command_permille;
    return true;
  }

  if (((s_previous_target[wheel] > 0.0f) && (target < 0.0f)) ||
      ((s_previous_target[wheel] < 0.0f) && (target > 0.0f)))
  {
    state->integrator_permille = 0.0f;
    /* Reset windup, not the actual output history: slew through zero. */
  }

  error = target - measured_counts_per_second;
  feedforward = WheelSpeedController_GetFeedforward(wheel, target) +
    feedforward_bias_permille;
  if (mode == WHEEL_SPEED_CONTROL_MODE_PIVOT)
  {
    if (WheelSpeedController_IsFront(wheel))
    {
      if (WheelSpeedController_Abs(target) >=
          WHEEL_SPEED_CONTROL_PIVOT_STATIONARY_CPS)
      {
        feedforward += WheelSpeedController_FrontSupport(wheel, target,
          measured_counts_per_second, elapsed_ms, &front_hold_integral);
      }
    }
    else
    {
      feedforward += WheelSpeedController_GetPivotLoadFeedforward(wheel, target);
      if ((wheel == MOTOR_DRIVER_WHEEL_LR) && (target <= -20.0f) &&
          s_lr_running.enabled)
      {
        feedforward += WheelSpeedController_LrRunningSupport(target,
          measured_counts_per_second, elapsed_ms, &front_hold_integral);
      }
      if ((wheel == MOTOR_DRIVER_WHEEL_RR) && (target >= 20.0f) &&
          s_rr_running.enabled)
      {
        feedforward += WheelSpeedController_RrRunningSupport(target,
          measured_counts_per_second, elapsed_ms, &front_hold_integral);
      }
    }
  }
  proportional_gain = WHEEL_SPEED_CONTROL_KP_PERMILLE_PER_CPS;
  if ((mode == WHEEL_SPEED_CONTROL_MODE_PIVOT) &&
      (((target > 0.0f) && (measured_counts_per_second > target)) ||
       ((target < 0.0f) && (measured_counts_per_second < target))))
  {
    /* Dampen wheels that are already too fast without increasing startup
     * force on a stuck wheel. The P term is continuous at zero error. */
    proportional_gain = WHEEL_SPEED_CONTROL_PIVOT_OVERSPEED_KP;
    /* Remove same-direction drive rather than command reverse torque just
     * to damp an overspeed. Deliberate reversals retain the normal guard. */
    if (target > 0.0f)
    {
      minimum_output = 0.0f;
    }
    else
    {
      maximum_output = 0.0f;
    }
  }
  proportional = proportional_gain * error;
  previous_integrator = state->integrator_permille;
  candidate_integrator = previous_integrator +
    WHEEL_SPEED_CONTROL_KI_PERMILLE_PER_CPS_SECOND * error *
    ((float)elapsed_ms / 1000.0f);
  if (front_hold_integral && (error * target > 0.0f))
  {
    /* Shared recovery/I budget: no double integration during handoff.
     * Opposing-error integration still unwinds normally on overspeed. */
    candidate_integrator = previous_integrator;
  }
  candidate_integrator = WheelSpeedController_Clamp(
    candidate_integrator,
    -WHEEL_SPEED_CONTROL_INTEGRATOR_LIMIT_PERMILLE,
    WHEEL_SPEED_CONTROL_INTEGRATOR_LIMIT_PERMILLE);

  desired_output = feedforward + proportional + candidate_integrator;
  magnitude_limited =
    (desired_output > maximum_output) || (desired_output < minimum_output);

  if (((desired_output > maximum_output) &&
       (error > 0.0f)) ||
      ((desired_output < minimum_output) &&
       (error < 0.0f)))
  {
    candidate_integrator = previous_integrator;
    desired_output = feedforward + proportional + candidate_integrator;
  }

  desired_output = WheelSpeedController_Clamp(
    desired_output,
    minimum_output, maximum_output);

  maximum_step = WHEEL_SPEED_CONTROL_SLEW_PERMILLE_PER_SECOND *
    ((float)elapsed_ms / 1000.0f);
  limited_output = WheelSpeedController_Clamp(
    desired_output,
    s_previous_output[wheel] - maximum_step,
    s_previous_output[wheel] + maximum_step);
  slew_limited = (limited_output != desired_output);

  /* Do not accumulate further in the same direction while slew-limited. */
  if (slew_limited &&
      (((desired_output > s_previous_output[wheel]) && (error > 0.0f)) ||
       ((desired_output < s_previous_output[wheel]) && (error < 0.0f))))
  {
    candidate_integrator = previous_integrator;
    desired_output = WheelSpeedController_Clamp(
      feedforward + proportional + candidate_integrator,
      minimum_output, maximum_output);
    limited_output = WheelSpeedController_Clamp(
      desired_output,
      s_previous_output[wheel] - maximum_step,
      s_previous_output[wheel] + maximum_step);
  }

  state->target_counts_per_second = target;
  state->measured_counts_per_second = measured_counts_per_second;
  state->error_counts_per_second = error;
  state->feedforward_permille = feedforward;
  state->proportional_permille = proportional;
  state->integrator_permille = candidate_integrator;
  state->command_permille =
    WheelSpeedController_RoundToInt16(limited_output);
  state->limited = magnitude_limited || slew_limited;
  s_previous_output[wheel] = limited_output;
  s_previous_target[wheel] = target;
  *command_permille = state->command_permille;
  return true;
}

bool WheelSpeedController_UpdateWithFeedforwardBias(
  MotorDriverWheel wheel,
  float target_counts_per_second,
  float measured_counts_per_second,
  float feedforward_bias_permille,
  uint32_t elapsed_ms,
  int16_t *command_permille)
{
  return WheelSpeedController_UpdateWithMode(
    wheel, target_counts_per_second, measured_counts_per_second,
    feedforward_bias_permille, WHEEL_SPEED_CONTROL_MODE_NORMAL,
    elapsed_ms, command_permille);
}

bool WheelSpeedController_Update(MotorDriverWheel wheel,
                                 float target_counts_per_second,
                                 float measured_counts_per_second,
                                 uint32_t elapsed_ms,
                                 int16_t *command_permille)
{
  return WheelSpeedController_UpdateWithFeedforwardBias(
    wheel,
    target_counts_per_second,
    measured_counts_per_second,
    0.0f,
    elapsed_ms,
    command_permille);
}

bool WheelSpeedController_GetState(MotorDriverWheel wheel,
                                   WheelSpeedControllerState *state)
{
  if ((!WheelSpeedController_IsWheelValid(wheel)) || (state == NULL))
  {
    return false;
  }

  *state = s_states[wheel];
  return true;
}

bool WheelSpeedController_GetFrontPivotState(
  MotorDriverWheel wheel, WheelSpeedFrontPivotState *state)
{
  if ((!WheelSpeedController_IsFront(wheel)) || (state == NULL))
  {
    return false;
  }
  *state = s_front_pivot[wheel];
  return true;
}
