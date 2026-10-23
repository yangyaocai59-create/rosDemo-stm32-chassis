/**
 * @file wheel_speed_controller.c
 * @brief Per-wheel direction-aware feedforward plus PI speed control.
 */

#include "wheel_speed_controller.h"

#include <stddef.h>
#include <math.h>

#include "wheel_speed_control_config.h"
#include "unified_pivot_control.h"

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
static UnifiedPivotState s_unified_pivot[MOTOR_DRIVER_WHEEL_COUNT];
static const float s_pivot_pi[MOTOR_DRIVER_WHEEL_COUNT][2] = {
  {WHEEL_SPEED_PIVOT_LF_KP, WHEEL_SPEED_PIVOT_LF_KI},
  {WHEEL_SPEED_PIVOT_LR_KP, WHEEL_SPEED_PIVOT_LR_KI},
  {WHEEL_SPEED_PIVOT_RF_KP, WHEEL_SPEED_PIVOT_RF_KI},
  {WHEEL_SPEED_PIVOT_RR_KP, WHEEL_SPEED_PIVOT_RR_KI}
};

bool WheelSpeedController_IsPivotMode(MotorDriverWheel wheel)
{
  return (uint32_t)wheel < MOTOR_DRIVER_WHEEL_COUNT && s_unified_pivot[wheel].active;
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
  s_unified_pivot[wheel]=(UnifiedPivotState){0};
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
  float previous_integrator;
  float candidate_integrator;
  float desired_output;
  float limited_output;
  float maximum_step;
  float minimum_output = -(float)WHEEL_SPEED_CONTROL_OUTPUT_LIMIT_PERMILLE;
  float maximum_output = (float)WHEEL_SPEED_CONTROL_OUTPUT_LIMIT_PERMILLE;
  bool magnitude_limited;
  bool slew_limited;

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

  if(mode==WHEEL_SPEED_CONTROL_MODE_PIVOT &&
     fabsf(target_counts_per_second)>=WHEEL_SPEED_CONTROL_ZERO_TARGET_CPS)
  {
    if(feedforward_bias_permille!=0) return false; /* no second startup owner */
    state=&s_states[wheel];
    target=WheelSpeedController_Clamp(target_counts_per_second,
      -WHEEL_SPEED_CONTROL_TARGET_LIMIT_CPS,WHEEL_SPEED_CONTROL_TARGET_LIMIT_CPS);
    float new_i=state->integrator_permille, ff,p,actual;
    bool new_limited;
    UnifiedPivotState next=s_unified_pivot[wheel];
    if(!UnifiedPivot_Update(&next,target,measured_counts_per_second,
       fabsf(WheelSpeedController_GetFeedforward(wheel,target)),
       fabsf(WheelSpeedController_GetPivotLoadFeedforward(wheel,target)),
       s_pivot_pi[wheel][0],s_pivot_pi[wheel][1],elapsed_ms,s_previous_output[wheel],
       &new_i,&ff,&p,&actual,&new_limited)) return false;
    state->target_counts_per_second=target;
    state->measured_counts_per_second=measured_counts_per_second;
    state->error_counts_per_second=target-measured_counts_per_second;
    state->feedforward_permille=ff;
    state->proportional_permille=p;
    state->integrator_permille=new_i;
    state->command_permille=WheelSpeedController_RoundToInt16(actual);
    state->limited=new_limited;
    s_unified_pivot[wheel]=next;
    s_previous_output[wheel]=actual;
    s_previous_target[wheel]=target;
    *command_permille=state->command_permille;
    return true;
  }
  if(s_unified_pivot[wheel].active)
  {
    s_unified_pivot[wheel]=(UnifiedPivotState){0};
    s_states[wheel].integrator_permille=0;
  }

  state = &s_states[wheel];
  target = WheelSpeedController_Clamp(target_counts_per_second,
    -WHEEL_SPEED_CONTROL_TARGET_LIMIT_CPS, WHEEL_SPEED_CONTROL_TARGET_LIMIT_CPS);

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
  proportional = WHEEL_SPEED_CONTROL_KP_PERMILLE_PER_CPS * error;
  previous_integrator = state->integrator_permille;
  candidate_integrator = previous_integrator +
    WHEEL_SPEED_CONTROL_KI_PERMILLE_PER_CPS_SECOND * error *
    ((float)elapsed_ms / 1000.0f);
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
