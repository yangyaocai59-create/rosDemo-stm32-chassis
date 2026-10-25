/* Compiled only for revision 200. The revision 191 implementation is retained
 * in wheel_speed_controller.c for exact historical regression and rebuilding. */
#include "wheel_speed_controller.h"
#include "wheel_speed_control_config.h"
#include <stddef.h>
#include <string.h>

typedef struct {
  float static_forward, velocity_forward, static_reverse, velocity_reverse;
  float load_forward, load_reverse, pivot_kp, pivot_ki;
} WheelCalibration;
#define WHEEL_CALIBRATION(W) { WHEEL_SPEED_FF_##W##_FWD_STATIC_PERMILLE, \
  WHEEL_SPEED_FF_##W##_FWD_GAIN_PER_CPS, WHEEL_SPEED_FF_##W##_REV_STATIC_PERMILLE, \
  WHEEL_SPEED_FF_##W##_REV_GAIN_PER_CPS, WHEEL_SPEED_PIVOT_LOAD_##W##_FWD_PERMILLE, \
  WHEEL_SPEED_PIVOT_LOAD_##W##_REV_PERMILLE, WHEEL_SPEED_PIVOT_##W##_KP, \
  WHEEL_SPEED_PIVOT_##W##_KI }
static const WheelCalibration s_calibration[MOTOR_DRIVER_WHEEL_COUNT] = {
  [MOTOR_DRIVER_WHEEL_LF] = WHEEL_CALIBRATION(LF),
  [MOTOR_DRIVER_WHEEL_LR] = WHEEL_CALIBRATION(LR),
  [MOTOR_DRIVER_WHEEL_RF] = WHEEL_CALIBRATION(RF),
  [MOTOR_DRIVER_WHEEL_RR] = WHEEL_CALIBRATION(RR)
};
#undef WHEEL_CALIBRATION
static WheelSpeedTransaction s_wheels;

static bool WheelValid(MotorDriverWheel wheel)
{ return (uint32_t)wheel < MOTOR_DRIVER_WHEEL_COUNT; }
static int16_t RoundOutput(float output)
{ return (int16_t)(output >= 0 ? output+0.5f : output-0.5f); }

float WheelSpeedController_GetFeedforwardTargetLimit(MotorDriverWheel wheel,
  bool forward,WheelSpeedControlMode mode)
{
  if (!WheelValid(wheel) ||
      (mode!=WHEEL_SPEED_CONTROL_MODE_NORMAL && mode!=WHEEL_SPEED_CONTROL_MODE_PIVOT))
    return NAN;
  const WheelCalibration *c=&s_calibration[wheel];
  const float ks=forward?c->static_forward:c->static_reverse;
  const float kv=forward?c->velocity_forward:c->velocity_reverse;
  const float load=mode==WHEEL_SPEED_CONTROL_MODE_PIVOT ?
    (forward?c->load_forward:c->load_reverse):0;
  const float ceiling=WHEEL_SPEED_CONTROL_OUTPUT_LIMIT_PERMILLE;
  const float knee=WHEEL_SPEED_PIVOT_LOAD_FULL_CPS;
  if (kv<=0 || knee<=0 || ks<0 || load<0 || ks>=ceiling) return 0;
  /* Exactly invert the existing piecewise target-ramped load model. */
  const float bound=(ks+kv*knee+load>=ceiling) ?
    (ceiling-ks)/(kv+load/knee):(ceiling-ks-load)/kv;
  return WheelControl_Clamp(bound,0,WHEEL_SPEED_CONTROL_TARGET_LIMIT_CPS);
}

float WheelSpeedController_GetPivotLoadFeedforward(MotorDriverWheel wheel, float target)
{
  if (!WheelValid(wheel) || !isfinite(target)) return NAN;
  if (fabsf(target) < WHEEL_SPEED_CONTROL_ZERO_TARGET_CPS) return 0;
  const float scale = WheelControl_Clamp(fabsf(target)/WHEEL_SPEED_PIVOT_LOAD_FULL_CPS,0,1);
  return target > 0 ? s_calibration[wheel].load_forward*scale :
                    -s_calibration[wheel].load_reverse*scale;
}

bool WheelSpeedController_Reset(MotorDriverWheel wheel)
{
  if (!WheelValid(wheel)) return false;
  s_wheels.memory[wheel] = (WheelControlMemory){0};
  s_wheels.state[wheel] = (WheelSpeedControllerState){0};
  ++s_wheels.revision;
  return true;
}
void WheelSpeedController_Init(void)
{
  /* Never reuse a transaction revision across reinitialization. */
  for (uint32_t w=0; w<MOTOR_DRIVER_WHEEL_COUNT; ++w)
    (void)WheelSpeedController_Reset((MotorDriverWheel)w);
}
bool WheelSpeedController_IsPivotMode(MotorDriverWheel wheel)
{ return WheelValid(wheel) && s_wheels.memory[wheel].active &&
         s_wheels.memory[wheel].mode == WHEEL_SPEED_CONTROL_MODE_PIVOT; }
bool WheelSpeedController_PrimeOutput(MotorDriverWheel wheel, int16_t command)
{
  if (!WheelValid(wheel)) return false;
  float output = WheelControl_Clamp((float)command,
    -WHEEL_SPEED_CONTROL_OUTPUT_LIMIT_PERMILLE,WHEEL_SPEED_CONTROL_OUTPUT_LIMIT_PERMILLE);
  /* Applied-output feedback must not reset the target/direction/I history. */
  s_wheels.memory[wheel].output = output;
  s_wheels.state[wheel].command_permille = RoundOutput(output);
  s_wheels.state[wheel].limited = true;
  s_wheels.state[wheel].limit_reasons |= WHEEL_LIMIT_ACTUATOR;
  ++s_wheels.revision;
  return true;
}
bool WheelSpeedController_Begin(WheelSpeedTransaction *transaction)
{
  if (!transaction) return false;
  *transaction = s_wheels;
  return true;
}
bool WheelSpeedController_Step(WheelSpeedTransaction *transaction,
  MotorDriverWheel wheel, float target, float measured, float bias,
  WheelSpeedControlMode mode, uint32_t elapsed_ms, int16_t *command)
{
  if (!transaction || !WheelValid(wheel) || !command || !isfinite(target) ||
      !isfinite(bias) || fabsf(bias)>WHEEL_SPEED_CONTROL_OUTPUT_LIMIT_PERMILLE ||
      (mode!=WHEEL_SPEED_CONTROL_MODE_NORMAL && mode!=WHEEL_SPEED_CONTROL_MODE_PIVOT) ||
      (mode==WHEEL_SPEED_CONTROL_MODE_PIVOT && bias!=0)) return false;
  const WheelCalibration *c=&s_calibration[wheel];
  const bool pivot=mode==WHEEL_SPEED_CONTROL_MODE_PIVOT;
  const float bounded=WheelControl_Clamp(target,-WHEEL_SPEED_CONTROL_TARGET_LIMIT_CPS,
                                               WHEEL_SPEED_CONTROL_TARGET_LIMIT_CPS);
  const float sign=bounded>=0?1.0f:-1.0f;
  const float base=sign*(bounded>=0?c->static_forward+c->velocity_forward*fabsf(bounded):
                                   c->static_reverse+c->velocity_reverse*fabsf(bounded));
  const float ff=base+bias+(pivot?WheelSpeedController_GetPivotLoadFeedforward(wheel,bounded):0);
  const WheelControlParameters parameters={
    pivot?c->pivot_kp:WHEEL_SPEED_CONTROL_KP_PERMILLE_PER_CPS,
    pivot?c->pivot_ki:WHEEL_SPEED_CONTROL_KI_PERMILLE_PER_CPS_SECOND,
    WHEEL_SPEED_CONTROL_INTEGRATOR_LIMIT_PERMILLE,WHEEL_SPEED_CONTROL_OUTPUT_LIMIT_PERMILLE,
    WHEEL_SPEED_CONTROL_SLEW_PERMILLE_PER_SECOND,WHEEL_SPEED_CONTROL_TARGET_LIMIT_CPS,
    WHEEL_SPEED_CONTROL_ZERO_TARGET_CPS};
  WheelControlResult next;
  if (!WheelControl_Step(&transaction->memory[wheel],&parameters,target,measured,
                         ff,(uint8_t)mode,elapsed_ms,&next)) return false;
  WheelSpeedControllerState state={0};
  state.target_counts_per_second=next.target;
  state.measured_counts_per_second=measured;
  state.error_counts_per_second=next.error;
  state.feedforward_permille=next.feedforward;
  state.proportional_permille=next.proportional;
  state.integrator_permille=next.memory.integral;
  state.command_permille=RoundOutput(next.memory.output);
  state.limit_reasons=next.reasons;
  state.limited=(next.reasons!=0);
  transaction->memory[wheel]=next.memory;
  transaction->state[wheel]=state;
  *command=state.command_permille;
  return true;
}
bool WheelSpeedController_Commit(const WheelSpeedTransaction *transaction)
{
  if (!transaction || transaction->revision!=s_wheels.revision) return false;
  s_wheels=*transaction;
  ++s_wheels.revision;
  return true;
}
bool WheelSpeedController_UpdateWithMode(MotorDriverWheel wheel,float target,float measured,
  float bias,WheelSpeedControlMode mode,uint32_t elapsed_ms,int16_t *command)
{
  if (!command) return false;
  WheelSpeedTransaction transaction;
  int16_t next;
  if (!WheelSpeedController_Begin(&transaction) ||
      !WheelSpeedController_Step(&transaction,wheel,target,measured,bias,mode,elapsed_ms,&next) ||
      !WheelSpeedController_Commit(&transaction)) return false;
  *command=next;
  return true;
}
bool WheelSpeedController_UpdateWithFeedforwardBias(MotorDriverWheel wheel,float target,
  float measured,float bias,uint32_t elapsed_ms,int16_t *command)
{ return WheelSpeedController_UpdateWithMode(wheel,target,measured,bias,
           WHEEL_SPEED_CONTROL_MODE_NORMAL,elapsed_ms,command); }
bool WheelSpeedController_Update(MotorDriverWheel wheel,float target,float measured,
  uint32_t elapsed_ms,int16_t *command)
{ return WheelSpeedController_UpdateWithFeedforwardBias(wheel,target,measured,0,elapsed_ms,command); }
bool WheelSpeedController_GetState(MotorDriverWheel wheel,WheelSpeedControllerState *state)
{
  if (!WheelValid(wheel) || !state) return false;
  *state=s_wheels.state[wheel];
  return true;
}
