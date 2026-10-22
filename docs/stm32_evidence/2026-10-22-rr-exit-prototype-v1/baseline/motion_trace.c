#include "motion_trace.h"
#include <string.h>
#include "FreeRTOS.h"
#include "task.h"
#include "chassis_runtime.h"
#include "wheel_speed_estimator.h"
#include "debug_log.h"

#define MOTION_TRACE_DEPTH 128U
#define MOTION_TRACE_PERIOD_MS 20U

typedef struct
{
  float target, measured, raw, ff, p, i;
  int32_t delta;
  int16_t pwm;
} TraceWheel;
typedef struct
{
  uint32_t time_ms, speed_dt_ms;
  ChassisVelocityCommand requested, applied;
  uint8_t state, stop, fault, standby;
  TraceWheel wheel[MOTOR_DRIVER_WHEEL_COUNT];
} TraceSample;

static TraceSample s_ring[MOTION_TRACE_DEPTH];
static uint16_t s_head, s_count, s_dump_index, s_dump_remaining;
static uint32_t s_last_record_ms;
static bool s_was_active, s_dumping;

void MotionTrace_Record(uint32_t now_ms)
{
  ChassisRuntimeStatus runtime;
  if (!ChassisRuntime_GetStatus(&runtime)) { return; }
  const bool active = runtime.state == CHASSIS_RUNTIME_STATE_ACTIVE;
  taskENTER_CRITICAL();
  if (s_dumping || (!active && !s_was_active))
  { taskEXIT_CRITICAL(); return; }
  if (active && !s_was_active) { s_head = 0U; s_count = 0U; }
  else if (active && (now_ms - s_last_record_ms < MOTION_TRACE_PERIOD_MS))
  { taskEXIT_CRITICAL(); return; }
  taskEXIT_CRITICAL();

  TraceSample sample = {0};
  ChassisControllerStatus controller;
  MotorDriverStatus motor;
  WheelSpeedState speeds[ENCODER_WHEEL_COUNT];
  if (!ChassisController_GetStatus(&controller) ||
      !MotorDriver_GetStatus(&motor) || !WheelSpeedEstimator_GetSnapshot(speeds))
  { return; }
  sample.time_ms = now_ms;
  sample.speed_dt_ms = speeds[0].sample_period_ms;
  sample.requested = runtime.requested_command;
  sample.applied = runtime.applied_command;
  sample.state = (uint8_t)runtime.state;
  sample.stop = (uint8_t)runtime.last_stop_reason;
  sample.fault = (uint8_t)runtime.fault;
  sample.standby = motor.standby_enabled ? 1U : 0U;
  for (uint32_t w = 0U; w < MOTOR_DRIVER_WHEEL_COUNT; ++w)
  {
    WheelSpeedControllerState control;
    taskENTER_CRITICAL();
    (void)WheelSpeedController_GetState((MotorDriverWheel)w, &control);
    taskEXIT_CRITICAL();
    sample.wheel[w] = (TraceWheel){
      controller.targets.counts_per_second[w],
      speeds[w].filtered_counts_per_second, speeds[w].raw_counts_per_second,
      control.feedforward_permille, control.proportional_permille,
      control.integrator_permille, speeds[w].delta_counts,
      motor.command_permille[w]
    };
  }
  taskENTER_CRITICAL();
  if (!s_dumping)
  {
    s_ring[s_head] = sample;
    s_head = (uint16_t)((s_head + 1U) % MOTION_TRACE_DEPTH);
    if (s_count < MOTION_TRACE_DEPTH) { ++s_count; }
    s_last_record_ms = now_ms;
    s_was_active = active;
  }
  taskEXIT_CRITICAL();
}

bool MotionTrace_RequestDump(void)
{
  /* This is also called from the UART motor task: never printf/dump here. */
  if (ChassisRuntime_IsActive() || ChassisController_IsActive()) { return false; }
  MotorDriverStatus motor;
  if (!MotorDriver_GetStatus(&motor) || motor.standby_enabled) { return false; }
  bool accepted = false;
  taskENTER_CRITICAL();
  if (!s_dumping && s_count != 0U)
  {
    s_dump_index = (uint16_t)((s_head + MOTION_TRACE_DEPTH - s_count) %
                              MOTION_TRACE_DEPTH);
    s_dump_remaining = s_count;
    s_dumping = true;
    accepted = true;
  }
  taskEXIT_CRITICAL();
  return accepted;
}

bool MotionTrace_ProcessDump(void)
{
  TraceSample sample;
  taskENTER_CRITICAL();
  if (!s_dumping) { taskEXIT_CRITICAL(); return false; }
  sample = s_ring[s_dump_index];
  s_dump_index = (uint16_t)((s_dump_index + 1U) % MOTION_TRACE_DEPTH);
  --s_dump_remaining;
  const bool last = s_dump_remaining == 0U;
  taskEXIT_CRITICAL();
  DEBUG_LOG_INFO(
    "TRACE t=%lu dt=%lu req=%ld/%ld applied=%ld/%ld state=%u stop=%u fault=%u stby=%u",
    (unsigned long)sample.time_ms, (unsigned long)sample.speed_dt_ms,
    (long)(sample.requested.linear_x_m_s * 1000.0f),
    (long)(sample.requested.angular_z_rad_s * 1000.0f),
    (long)(sample.applied.linear_x_m_s * 1000.0f),
    (long)(sample.applied.angular_z_rad_s * 1000.0f),
    sample.state, sample.stop, sample.fault, sample.standby);
  static const char *const names[] = {"LF", "LR", "RF", "RR"};
  for (uint32_t w = 0U; w < MOTOR_DRIVER_WHEEL_COUNT; ++w)
  {
    const TraceWheel *v = &sample.wheel[w];
    DEBUG_LOG_INFO(
      "TRACE %s dc=%ld raw=%ld target=%ld cps=%ld ff_x10=%ld p_x10=%ld i_x10=%ld pwm=%d",
      names[w], (long)v->delta, (long)v->raw, (long)v->target,
      (long)v->measured, (long)(v->ff * 10.0f), (long)(v->p * 10.0f),
      (long)(v->i * 10.0f), (int)v->pwm);
  }
  if (last)
  {
    taskENTER_CRITICAL(); s_dumping = false; s_was_active = false;
    taskEXIT_CRITICAL();
    DEBUG_LOG_INFO("TRACE END; units req/applied=mm/s,mrad/s cps=counts/s pwm=permille");
  }
  return true;
}
