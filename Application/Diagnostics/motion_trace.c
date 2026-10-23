#include "motion_trace.h"
#include <math.h>
#include <string.h>
#include "FreeRTOS.h"
#include "task.h"
#include "chassis_runtime.h"
#include "wheel_speed_estimator.h"
#include "wheel_speed_control_config.h"
#include "emergency_stop.h"
#include "encoder_diagnostics.h"
#include "debug_log.h"

#define MOTION_TRACE_DEPTH 128U
#define MOTION_TRACE_PERIOD_MS 20U
#define MOTION_TRACE_POST_SAMPLES 25U
#define MOTION_TRACE_GRACE_MS 1000U
#define MOTION_TRACE_CONFIRM_MS 100U
/* Diagnostic thresholds only: never feed these into control or safety. */
enum { TRACE_NONE, TRACE_UNDERSPEED, TRACE_STOP, TRACE_FAULT, TRACE_ESTOP };
enum { TRACE_ESTOP_PRESENT=1U, TRACE_ESTOP_BOUNDARY=2U,
       TRACE_STALE_SPEED=4U, TRACE_CONTROL_MISMATCH=8U };

typedef struct
{
  int64_t total;
  EncoderDiagnosticWheel encoder;
  float target, measured, raw, ff, p, i;
  int32_t delta;
  uint32_t ccr;
  int16_t pwm, request, control_pwm;
  uint8_t limited, output_flags;
} TraceWheel;
typedef struct
{
  uint32_t time_ms, speed_dt_ms, speed_time_ms, control_time_ms, sequence;
  ChassisVelocityCommand requested, applied;
  uint32_t arr, bdtr, ccer;
  uint16_t direction_bits;
  uint8_t state, stop, fault, standby, standby_pin, flags;
  TraceWheel wheel[MOTOR_DRIVER_WHEEL_COUNT];
} TraceSample;

static TraceSample s_ring[MOTION_TRACE_DEPTH];
static uint16_t s_head, s_count, s_dump_index, s_dump_remaining;
static uint32_t s_last_record_ms, s_start_ms, s_sequence;
static uint32_t s_trigger_ms, s_trigger_sequence, s_last_speed_ms;
static uint16_t s_low_ms[MOTOR_DRIVER_WHEEL_COUNT];
static uint8_t s_reason, s_trigger_mask, s_post_remaining;
static bool s_was_active, s_dumping, s_frozen, s_triggered, s_dump_header;

/* motorTask calls after runtime/output processing. The outer lock prevents
 * task STOP/commands from splitting these getters. It does not mask priority-4
 * E-stop; the latch bracket explicitly marks that boundary. No IO/printf here. */
static bool MotionTrace_Snapshot(uint32_t now_ms, TraceSample *sample)
{
  /* Single motorTask producer. Keep this scratch off the 2 KiB motor stack. */
  static ChassisRuntimeStatus runtime;
  static ChassisControllerStatus controller;
  static MotorDriverStatus motor;
  static MotorDriverDiagnostics hardware;
  static WheelSpeedState speeds[ENCODER_WHEEL_COUNT];
  static WheelSpeedControllerState controls[MOTOR_DRIVER_WHEEL_COUNT];
  static EncoderDiagnosticWheel encoders[ENCODER_WHEEL_COUNT];
  taskENTER_CRITICAL();
  const bool estop_before = EmergencyStop_IsLatched();
  bool ok = ChassisRuntime_GetStatus(&runtime) &&
    ChassisController_GetStatus(&controller) && MotorDriver_GetStatus(&motor) &&
    MotorDriver_GetDiagnostics(&hardware) && WheelSpeedEstimator_GetSnapshot(speeds) &&
    EncoderDiagnostics_Read(encoders);
  for (uint32_t w=0U; ok && w<MOTOR_DRIVER_WHEEL_COUNT; ++w)
    ok = WheelSpeedController_GetState((MotorDriverWheel)w, &controls[w]);
  const bool estop_after = EmergencyStop_IsLatched();
  taskEXIT_CRITICAL();
  if (!ok) { return false; }
  *sample = (TraceSample){0};
  sample->time_ms=now_ms;
  sample->control_time_ms=controller.last_control_ms;
  sample->speed_time_ms=speeds[0].sample_time_ms;
  sample->speed_dt_ms=speeds[0].sample_period_ms;
  sample->requested=runtime.requested_command;
  sample->applied=runtime.applied_command;
  sample->state=(uint8_t)runtime.state;
  sample->stop=(uint8_t)runtime.last_stop_reason;
  sample->fault=(uint8_t)runtime.fault;
  sample->standby=motor.standby_enabled;
  sample->standby_pin=hardware.standby_pin;
  sample->arr=hardware.auto_reload;
  sample->bdtr=hardware.bdtr;
  sample->ccer=hardware.ccer;
  sample->direction_bits=hardware.direction_bits;
  if (estop_before || estop_after) sample->flags |= TRACE_ESTOP_PRESENT;
  if (estop_before != estop_after) sample->flags |= TRACE_ESTOP_BOUNDARY;
  for (uint32_t w=0U; w<MOTOR_DRIVER_WHEEL_COUNT; ++w)
  {
    const WheelSpeedControllerState *c=&controls[w];
    const WheelSpeedState *v=&speeds[w];
    sample->wheel[w]=(TraceWheel){
      .total=v->accumulated_count, .target=c->target_counts_per_second,
      .encoder=encoders[w],
      .measured=v->filtered_counts_per_second, .raw=v->raw_counts_per_second,
      .ff=c->feedforward_permille, .p=c->proportional_permille,
      .i=c->integrator_permille, .delta=v->delta_counts,
      .ccr=hardware.compare[w], .pwm=motor.command_permille[w],
      .request=hardware.requested_permille[w], .control_pwm=c->command_permille,
      .limited=c->limited, .output_flags=(hardware.guard_mask >> w)&1U
    };
    /* Observations, not invented controller decisions: 1=driver zero guard,
     * 2=I at cap, 4=FF+P+I beyond PWM cap, 8=request differs from applied. */
    if (fabsf(c->integrator_permille)>=WHEEL_SPEED_CONTROL_INTEGRATOR_LIMIT_PERMILLE-0.001f)
      sample->wheel[w].output_flags |= 2U;
    if (fabsf(c->feedforward_permille+c->proportional_permille+c->integrator_permille)>
        WHEEL_SPEED_CONTROL_OUTPUT_LIMIT_PERMILLE)
      sample->wheel[w].output_flags |= 4U;
    if (hardware.requested_permille[w]!=motor.command_permille[w])
      sample->wheel[w].output_flags |= 8U;
    if (!v->valid || v->sample_time_ms!=sample->speed_time_ms ||
        v->sample_period_ms!=sample->speed_dt_ms ||
        (uint32_t)(now_ms-v->sample_time_ms)>40U)
      sample->flags |= TRACE_STALE_SPEED;
    if (runtime.state==CHASSIS_RUNTIME_STATE_ACTIVE &&
        (!controller.active ||
         c->target_counts_per_second!=controller.targets.counts_per_second[w] ||
         c->measured_counts_per_second!=v->filtered_counts_per_second))
      sample->flags |= TRACE_CONTROL_MISMATCH;
  }
  return true;
}

void MotionTrace_Record(uint32_t now_ms)
{
  taskENTER_CRITICAL();
  const bool blocked=s_dumping || s_frozen;
  taskEXIT_CRITICAL();
  if (blocked) return;
  TraceSample sample;
  if (!MotionTrace_Snapshot(now_ms, &sample)) return;
  const bool active=sample.state==CHASSIS_RUNTIME_STATE_ACTIVE;
  if (!active && !s_was_active) return;
  if (active && !s_was_active)
  {
    s_head=0U; s_count=0U; s_start_ms=now_ms;
    s_triggered=false; s_reason=TRACE_NONE; s_trigger_mask=0U;
    s_post_remaining=0U; s_trigger_ms=0U; s_trigger_sequence=0U;
    memset(s_low_ms,0,sizeof s_low_ms);
    s_last_speed_ms=sample.speed_time_ms;
  }
  else if (active && (uint32_t)(now_ms-s_last_record_ms)<MOTION_TRACE_PERIOD_MS)
    return;
  sample.sequence=++s_sequence; /* record sequence, not USB command sequence */
  uint8_t low_mask=0U;
  const bool fresh=sample.speed_time_ms!=s_last_speed_ms;
  if (!s_triggered && active && fresh && sample.flags==0U &&
      (uint32_t)(now_ms-s_start_ms)>=MOTION_TRACE_GRACE_MS)
  {
    for (uint32_t w=0U; w<MOTOR_DRIVER_WHEEL_COUNT; ++w)
    {
      const TraceWheel *v=&sample.wheel[w];
      const float sign=v->target<0.0f ? -1.0f : 1.0f;
      const float target=fabsf(v->target);
      if (target>=80.0f && (sign*v->raw < 0.25f*target ||
                           sign*v->measured < 0.5f*target))
      {
        /* Missing time is not proof of continuous low speed. */
        if (sample.speed_dt_ms==0U || sample.speed_dt_ms>40U)
        { s_low_ms[w]=0U; continue; }
        const uint32_t dt=sample.speed_dt_ms;
        s_low_ms[w]=(uint16_t)(s_low_ms[w]+dt);
        if (s_low_ms[w]>=MOTION_TRACE_CONFIRM_MS) low_mask |= (uint8_t)(1U<<w);
      }
      else s_low_ms[w]=0U;
    }
  }
  else if (!s_triggered) memset(s_low_ms,0,sizeof s_low_ms);
  s_last_speed_ms=sample.speed_time_ms;
  const bool new_trigger=!s_triggered && (low_mask!=0U || !active ||
                                         (sample.flags & TRACE_ESTOP_PRESENT)!=0U);
  if (new_trigger)
  {
    s_triggered=true; s_trigger_ms=now_ms; s_trigger_sequence=sample.sequence;
    s_trigger_mask=low_mask;
    s_reason=(sample.flags & TRACE_ESTOP_PRESENT)!=0U ? TRACE_ESTOP :
      (!active ? (sample.fault!=0U ? TRACE_FAULT : TRACE_STOP) : TRACE_UNDERSPEED);
    s_post_remaining=MOTION_TRACE_POST_SAMPLES;
  }
  taskENTER_CRITICAL();
  if (!s_dumping)
  {
    s_ring[s_head]=sample;
    s_head=(uint16_t)((s_head+1U)%MOTION_TRACE_DEPTH);
    if (s_count<MOTION_TRACE_DEPTH) ++s_count;
    s_last_record_ms=now_ms; s_was_active=active;
    if (s_triggered && !new_trigger && s_post_remaining!=0U) --s_post_remaining;
    if (!active || (sample.flags & TRACE_ESTOP_PRESENT)!=0U ||
        (s_triggered && s_post_remaining==0U)) s_frozen=true;
  }
  taskEXIT_CRITICAL();
}

static bool MotionTrace_IsStopped(void)
{
  MotorDriverStatus motor;
  MotorDriverDiagnostics hardware;
  return !ChassisRuntime_IsActive() && !ChassisController_IsActive() &&
    MotorDriver_GetStatus(&motor) && !motor.standby_enabled &&
    MotorDriver_GetDiagnostics(&hardware) && !hardware.standby_pin;
}

bool MotionTrace_RequestDump(void)
{
  bool accepted=false;
  taskENTER_CRITICAL();
  if (MotionTrace_IsStopped() && !s_dumping && s_count!=0U)
  {
    s_dump_index=(uint16_t)((s_head+MOTION_TRACE_DEPTH-s_count)%MOTION_TRACE_DEPTH);
    s_dump_remaining=s_count; s_dumping=true; s_dump_header=true;
    accepted=true;
  }
  taskEXIT_CRITICAL();
  return accepted;
}

bool MotionTrace_ProcessDump(void)
{
#define TRACE_PRINT(...) do { \
  if (!DebugLog_Printf(DEBUG_LOG_LEVEL_INFO, __VA_ARGS__)) goto dump_failed; \
} while (0)
  TraceSample sample;
  taskENTER_CRITICAL();
  if (!s_dumping) { taskEXIT_CRITICAL(); return false; }
  /* Pause if motion resumes. At most the one already dequeued frame can print
   * if a command arrives after this check. Diagnostics add no motion interlock. */
  if (!MotionTrace_IsStopped()) { taskEXIT_CRITICAL(); return true; }
  const bool header=s_dump_header; s_dump_header=false;
  sample=s_ring[s_dump_index];
  s_dump_index=(uint16_t)((s_dump_index+1U)%MOTION_TRACE_DEPTH);
  --s_dump_remaining;
  const bool last=s_dump_remaining==0U;
  taskEXIT_CRITICAL();
  if (header) {
    static ChassisRuntimeEvent events[CHASSIS_RUNTIME_EVENT_CAPACITY];
    uint32_t overwritten;
    const uint32_t count=ChassisRuntime_GetEvents(events, &overwritten);
    TRACE_PRINT("RUNTIME_EVENTS count=%lu overwritten=%lu", (unsigned long)count,
                (unsigned long)overwritten);
    for (uint32_t i=0U; i<count; ++i) {
      const ChassisRuntimeEvent *e=&events[i];
      TRACE_PRINT("RUNTIME_EVENT seq=%lu t=%lu kind=%u state=%u generation=%lu command=%lu",
        (unsigned long)e->sequence, (unsigned long)e->time_ms, (unsigned)e->kind,
        (unsigned)e->state, (unsigned long)e->generation, (unsigned long)e->command_sequence);
    }
  }
  if (header) TRACE_PRINT(
    "TRACE2 BEGIN schema=3 count=%u reason=%u mask=%u trigger=%lu trigger_seq=%lu post_left=%u",
    s_count,s_reason,s_trigger_mask,(unsigned long)s_trigger_ms,
    (unsigned long)s_trigger_sequence,s_post_remaining);
  TRACE_PRINT(
    "TRACE t=%lu dt=%lu req=%ld/%ld applied=%ld/%ld state=%u stop=%u fault=%u stby=%u",
    (unsigned long)sample.time_ms,(unsigned long)sample.speed_dt_ms,
    (long)(sample.requested.linear_x_m_s*1000.0f),(long)(sample.requested.angular_z_rad_s*1000.0f),
    (long)(sample.applied.linear_x_m_s*1000.0f),(long)(sample.applied.angular_z_rad_s*1000.0f),
    sample.state,sample.stop,sample.fault,sample.standby);
  TRACE_PRINT(
    "TRACE2 SNAP seq=%lu speed_t=%lu control_t=%lu flags=%u arr=%lu bdtr=%lu ccer=%lu dirs=%u pin=%u",
    (unsigned long)sample.sequence,(unsigned long)sample.speed_time_ms,
    (unsigned long)sample.control_time_ms,sample.flags,(unsigned long)sample.arr,
    (unsigned long)sample.bdtr,(unsigned long)sample.ccer,sample.direction_bits,sample.standby_pin);
  static const char *const names[]={"LF","LR","RF","RR"};
  for (uint32_t w=0U; w<MOTOR_DRIVER_WHEEL_COUNT; ++w)
  {
    const TraceWheel *v=&sample.wheel[w];
    TRACE_PRINT(
      "TRACE %s dc=%ld raw=%ld target=%ld cps=%ld ff_x10=%ld p_x10=%ld i_x10=%ld pwm=%d",
      names[w],(long)v->delta,(long)v->raw,(long)v->target,(long)v->measured,
      (long)(v->ff*10.0f),(long)(v->p*10.0f),(long)(v->i*10.0f),(int)v->pwm);
    const uint64_t total=(uint64_t)v->total;
    /* Avoid newlib-nano's optional 64-bit printf support. */
    TRACE_PRINT(
      "TRACE2 %s total_hi=%lu total_lo=%lu request=%d ctrl=%d ccr=%lu limited=%u obs=%u",
      names[w],(unsigned long)(total>>32),(unsigned long)(total&0xffffffffU),
      (int)v->request,(int)v->control_pwm,(unsigned long)v->ccr,v->limited,v->output_flags);
    TRACE_PRINT("TRACE2 ENC %s sampled=%lu live=%lu bad=%u ab=%u",
      names[w],(unsigned long)v->encoder.sampled_cnt,
      (unsigned long)v->encoder.live_cnt,v->encoder.bad,v->encoder.ab);
  }
  if (last)
  {
    TRACE_PRINT("TRACE END; schema=3 req/applied=mm/s,mrad/s cps=counts/s pwm=permille");
    taskENTER_CRITICAL();
    s_dumping=false; s_frozen=false; s_was_active=false; s_count=0U;
    taskEXIT_CRITICAL();
  }
  return true;
dump_failed:
  /* Bound UART failure cost to one failed log call. Retain the entire capture
   * for a fresh request, rather than reporting END for an incomplete export. */
  taskENTER_CRITICAL(); s_dumping=false; s_frozen=true; taskEXIT_CRITICAL();
  return true;
#undef TRACE_PRINT
}
