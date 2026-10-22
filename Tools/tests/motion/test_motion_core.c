/* Production PI/profile/health/timing functions; no serial or hardware. */
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include "chassis_motion_profile.h"
#include "wheel_speed_controller.h"
#include "wheel_speed_control_config.h"
#include "fault_manager.h"
#include "debug_log.h"
#include "control_timing.h"

static unsigned depth;
void Test_EnterCritical(void) { ++depth; }
void Test_ExitCritical(void) { assert(depth != 0U); --depth; }
bool DebugLog_Printf(DebugLog_Level level, const char *format, ...)
{ (void)level; (void)format; assert(depth == 0U); return true; }

static void Heartbeats(uint32_t time)
{
  for (unsigned i = 0U; i < FAULT_MANAGER_TASK_COUNT; ++i)
    FaultManager_Heartbeat((FaultManagerTask)i, time);
}

static WheelSpeedControllerState ModeResponse(MotorDriverWheel wheel,
  WheelSpeedControlMode mode, float target, float measured)
{
  int16_t pwm;
  WheelSpeedControllerState state;
  assert(WheelSpeedController_Reset(wheel));
  float bias=target>0.0f ? 60.0f : -60.0f;
  /* Equalize feedforward when isolating P-mode behavior below. */
  if(mode==WHEEL_SPEED_CONTROL_MODE_NORMAL)
    bias+=WheelSpeedController_GetPivotLoadFeedforward(wheel,target);
  for(unsigned i=0;i<20U;i++)
    assert(WheelSpeedController_UpdateWithMode(wheel,target,target,bias,mode,20U,&pwm));
  assert(WheelSpeedController_UpdateWithMode(wheel,target,measured,bias,mode,20U,&pwm));
  assert(WheelSpeedController_GetState(wheel,&state));
  return state;
}

static void TestPivotLoadFeedforward(void)
{
  const float loads[4][2]={{60.0f,65.0f},{0.0f,0.0f},{60.0f,45.0f},{0.0f,0.0f}};
  const float statics[4][2]={{51.8f,46.3f},{45.0f,50.4f},{54.7f,56.8f},{58.9f,56.9f}};
  const float gains[4][2]={{0.14600f,0.14677f},{0.14934f,0.14667f},{0.14703f,0.14249f},{0.15779f,0.15108f}};
  const float magnitudes[]={0.0f,0.5f,1.0f,20.0f,80.0f,159.999f,160.0f,280.0f,1600.0f,2000.0f};
  for(unsigned w=0;w<4;w++)
    for(unsigned reverse=0;reverse<2;reverse++)
      for(unsigned k=0;k<sizeof(magnitudes)/sizeof(magnitudes[0]);k++)
      {
        const float sign=reverse ? -1.0f : 1.0f;
        const float mag=magnitudes[k],target=sign*mag;
        const float clamped=fminf(mag,1600.0f);
        const float expected_load=mag<1.0f ? 0.0f : sign*loads[w][reverse]*fminf(mag/160.0f,1.0f);
        const float expected_base=mag<1.0f ? 0.0f : sign*(statics[w][reverse]+gains[w][reverse]*clamped);
        const MotorDriverWheel wheel=(MotorDriverWheel)w;
        assert(fabsf(WheelSpeedController_GetPivotLoadFeedforward(wheel,target)-expected_load)<0.001f);
        int16_t pwm;
        WheelSpeedControllerState normal,pivot,legacy;
        assert(WheelSpeedController_Reset(wheel));
        assert(WheelSpeedController_UpdateWithMode(wheel,target,sign*clamped,0,
          WHEEL_SPEED_CONTROL_MODE_NORMAL,20U,&pwm));
        assert(WheelSpeedController_GetState(wheel,&normal));
        assert(fabsf(normal.feedforward_permille-expected_base)<0.001f);
        assert(WheelSpeedController_Reset(wheel));
        assert(WheelSpeedController_Update(wheel,target,sign*clamped,20U,&pwm));
        assert(WheelSpeedController_GetState(wheel,&legacy));
        assert(legacy.feedforward_permille==normal.feedforward_permille &&
          legacy.integrator_permille==normal.integrator_permille &&
          legacy.command_permille==normal.command_permille);
        assert(WheelSpeedController_Reset(wheel));
        assert(WheelSpeedController_UpdateWithMode(wheel,target,sign*clamped,0,
          WHEEL_SPEED_CONTROL_MODE_PIVOT,20U,&pwm));
        assert(WheelSpeedController_GetState(wheel,&pivot));
        const float applied_load=(w==MOTOR_DRIVER_WHEEL_LF || w==MOTOR_DRIVER_WHEEL_RF) &&
          clamped<20.0f ? 0.0f : expected_load;
        assert(fabsf(pivot.feedforward_permille-expected_base-applied_load)<0.001f);
        assert(pivot.integrator_permille==0.0f && abs(pwm)<=300);
      }
  assert(isnan(WheelSpeedController_GetPivotLoadFeedforward(MOTOR_DRIVER_WHEEL_COUNT,160.0f)));
  assert(isnan(WheelSpeedController_GetPivotLoadFeedforward(MOTOR_DRIVER_WHEEL_LF,NAN)));
  assert(isnan(WheelSpeedController_GetPivotLoadFeedforward(MOTOR_DRIVER_WHEEL_LF,INFINITY)));
  assert(isnan(WheelSpeedController_GetPivotLoadFeedforward(MOTOR_DRIVER_WHEEL_LF,-INFINITY)));
  puts("PASS: explicit front/direction pivot load table, zero and 1/20/80/160/280/1600 cps ramp/clamp, rear zero; NORMAL/legacy feedforward remains original, invalid lookup rejected");
}

int main(void)
{
  TestPivotLoadFeedforward();
  FaultManagerStatus health;
  FaultManager_Init(0U); FaultManager_MarkSystemInitComplete(5000U);
  Heartbeats(5000U); FaultManager_Process(5000U);
  FaultManager_Heartbeat(FAULT_MANAGER_TASK_MOTOR, 5001U);
  FaultManager_Process(5000U);
  assert(FaultManager_GetStatus(&health) && health.stale_task_mask == 0U &&
         health.latched_faults == 0U);
  FaultManager_Process(5101U); /* safety stale, not motor/USB critical. */
  assert(FaultManager_GetStatus(&health) && health.latched_faults == 0U);
  Heartbeats(5200U);
  FaultManager_Heartbeat(FAULT_MANAGER_TASK_MOTOR, 5000U);
  FaultManager_Process(5250U);
  assert(!FaultManager_HasCriticalFault()); /* strict >250 preserved */
  FaultManager_Process(5251U);
  assert(FaultManager_HasCriticalFault());
  Heartbeats(5252U); FaultManager_Process(5252U);
  assert(FaultManager_HasCriticalFault());
  assert(FaultManager_Clear(5252U));
  FaultManager_Init(0xFFFFF000U);
  FaultManager_MarkSystemInitComplete(0xFFFFFFF0U);
  Heartbeats(0xFFFFFFF0U); FaultManager_Process(0x00000010U);
  assert(!FaultManager_HasCriticalFault());
  FaultManager_Process(0x00000100U);
  assert(FaultManager_HasCriticalFault());
  puts("PASS: stale caller/new heartbeat, true 250/251ms timeout, latch/clear and tick wrap");

  ChassisVelocityCommand v = {0}, goal = {0.04f, 0.30f}, next;
  for (unsigned i = 0U; i < 100U; ++i)
  {
    assert(ChassisMotionProfile_Step(&v, &goal, 10U, &next));
    assert(next.linear_x_m_s >= v.linear_x_m_s &&
           next.linear_x_m_s - v.linear_x_m_s < 0.00201f);
    assert(next.angular_z_rad_s - v.angular_z_rad_s < 0.01501f);
    v = next;
  }
  assert(fabsf(v.linear_x_m_s - goal.linear_x_m_s) < 1e-6f);
  goal.linear_x_m_s = -0.04f; goal.angular_z_rad_s = -0.30f;
  bool passed_zero = false;
  for (unsigned i = 0U; i < 100U; ++i)
  {
    assert(ChassisMotionProfile_Step(&v, &goal, 10U, &next));
    if (next.linear_x_m_s == 0.0f) { passed_zero = true; }
    if (next.linear_x_m_s < 0.0f) { assert(passed_zero); }
    assert(fabsf(next.linear_x_m_s - v.linear_x_m_s) < 0.00301f);
    v = next;
  }
  assert(fabsf(v.linear_x_m_s - goal.linear_x_m_s) < 1e-6f);
  assert(!ChassisMotionProfile_Step(&v, &goal, 101U, &next));
  goal.linear_x_m_s = NAN;
  assert(!ChassisMotionProfile_Step(&v, &goal, 10U, &next));
  puts("PASS: bounded linear/angular startup, reversal crosses zero, invalid dt/NaN rejected");

  WheelSpeedController_Init();
  for (unsigned w = 0U; w < MOTOR_DRIVER_WHEEL_COUNT; ++w)
  {
    int16_t old = 0, pwm;
    const float targets[] = {306.0f, -306.0f, 0.0f, 1000.0f, -1000.0f};
    for (unsigned phase = 0U; phase < 5U; ++phase)
      for (unsigned step = 0U; step < 100U; ++step)
      {
        assert(WheelSpeedController_Update((MotorDriverWheel)w,
          targets[phase], 306.0f, 10U, &pwm));
        assert(abs(pwm - old) <= 16); /* floating -> integer rounding */
        assert(abs(pwm) <= 300);
        old = pwm;
      }
    assert(WheelSpeedController_Reset((MotorDriverWheel)w));
    assert(WheelSpeedController_Update((MotorDriverWheel)w, 0, 0, 10U, &pwm));
    assert(pwm == 0);
    assert(!WheelSpeedController_Update((MotorDriverWheel)w, NAN, 0, 10U, &pwm));
    assert(!WheelSpeedController_Update((MotorDriverWheel)w, 0, INFINITY, 10U, &pwm));
  }
  puts("PASS: all four production PI outputs obey slew across reversal/zero, immediate Reset and finite inputs");

  for (unsigned w = 0U; w < MOTOR_DRIVER_WHEEL_COUNT; ++w)
  {
    for (int sign = -1; sign <= 1; sign += 2)
    {
      const MotorDriverWheel wheel = (MotorDriverWheel)w;
      const float target = (float)sign * 160.0f;
      float bias = (float)sign * WHEEL_SPEED_CONTROL_PIVOT_BREAKAWAY_PERMILLE;
      int16_t pwm = 0;
      WheelSpeedControllerState before, after;
      assert(WheelSpeedController_Reset(wheel));
      for (unsigned step = 0U; step < 20U; ++step)
        assert(WheelSpeedController_UpdateWithFeedforwardBias(
          wheel, target, target, bias, 20U, &pwm));
      assert(WheelSpeedController_GetState(wheel, &before));
      bias = WheelSpeedController_ReleaseFeedforwardBias(wheel, bias);
      assert(isfinite(bias) && fabsf(bias - (float)sign * 10.0f) < 0.001f);
      assert(WheelSpeedController_UpdateWithFeedforwardBias(
        wheel, target, target, bias, 20U, &pwm));
      assert(WheelSpeedController_GetState(wheel, &after));
      assert(fabsf((before.feedforward_permille + before.integrator_permille) -
                   (after.feedforward_permille + after.integrator_permille)) < 0.001f);
      assert(before.command_permille == after.command_permille);
      assert(fabsf(after.integrator_permille) <= 50.0f);

      /* At equilibrium retain the residual; do not remove it on the next tick. */
      for (unsigned step = 0U; step < 20U; ++step)
      {
        bias = WheelSpeedController_ReleaseFeedforwardBias(wheel, bias);
        assert(WheelSpeedController_UpdateWithFeedforwardBias(
          wheel, target, target, bias, 20U, &pwm));
        assert(pwm == before.command_permille);
      }
      /* A real overspeed error must unwind support instead of holding a floor. */
      for (unsigned step = 0U; step < 20U; ++step)
      {
        bias = WheelSpeedController_ReleaseFeedforwardBias(wheel, bias);
        assert(WheelSpeedController_UpdateWithFeedforwardBias(
          wheel, target, target * 3.0f, bias, 20U, &pwm));
        assert(WheelSpeedController_GetState(wheel, &after));
        assert(fabsf(after.integrator_permille) <= 50.0f && abs(pwm) <= 300);
      }
      assert(fabsf(bias) < 0.001f);
      assert(WheelSpeedController_Reset(wheel));
      assert(WheelSpeedController_Update(wheel, 0, 0, 20U, &pwm) && pwm == 0);
    }
  }
  assert(isnan(WheelSpeedController_ReleaseFeedforwardBias(
    MOTOR_DRIVER_WHEEL_COUNT, 60.0f)));
  assert(isnan(WheelSpeedController_ReleaseFeedforwardBias(
    MOTOR_DRIVER_WHEEL_RR, NAN)));
  assert(isnan(WheelSpeedController_ReleaseFeedforwardBias(
    MOTOR_DRIVER_WHEEL_RR, INFINITY)));
  assert(isnan(WheelSpeedController_ReleaseFeedforwardBias(
    MOTOR_DRIVER_WHEEL_RR, 301.0f)));
  puts("PASS: all wheels/both directions conserve FF+I and PWM at assist release; bounded residual holds at equilibrium, unwinds with error, invalid inputs rejected");

  for(unsigned w=0;w<4;w++)
    for(int sign=-1;sign<=1;sign+=2)
    {
      const MotorDriverWheel wheel=(MotorDriverWheel)w;
      const float target=160.0f*(float)sign;
      const float speeds[]={0.0f,80.0f,160.0f,235.0f,320.0f,-160.0f};
      for(unsigned i=0;i<sizeof(speeds)/sizeof(speeds[0]);i++)
      {
        const float measured=speeds[i]*(float)sign;
        WheelSpeedControllerState normal=ModeResponse(wheel,WHEEL_SPEED_CONTROL_MODE_NORMAL,target,measured);
        WheelSpeedControllerState pivot=ModeResponse(wheel,WHEEL_SPEED_CONTROL_MODE_PIVOT,target,measured);
        assert(fabsf(normal.proportional_permille - 0.05f*(target-measured))<0.001f);
        if(speeds[i]<=160.0f)
        {
          assert(normal.command_permille==pivot.command_permille);
          assert(normal.proportional_permille==pivot.proportional_permille);
          assert(normal.integrator_permille==pivot.integrator_permille);
        }
        else
        {
          assert(fabsf(pivot.proportional_permille - 0.15f*(target-measured))<0.001f);
          assert(abs(pivot.command_permille)<abs(normal.command_permille));
          if(speeds[i]==235.0f)
            assert(fabsf(pivot.proportional_permille-normal.proportional_permille)==7.5f);
        }
      }
      /* Strong overspeed may coast, never initiate reverse torque. */
      (void)ModeResponse(wheel,WHEEL_SPEED_CONTROL_MODE_PIVOT,target,target);
      WheelSpeedControllerState state;
      assert(WheelSpeedController_GetState(wheel,&state));
      int16_t previous=state.command_permille,pwm=previous;
      for(unsigned i=0;i<32;i++)
      {
        assert(WheelSpeedController_UpdateWithMode(wheel,target,2000.0f*(float)sign,
          60.0f*(float)sign,WHEEL_SPEED_CONTROL_MODE_PIVOT,20U,&pwm));
        assert((int)pwm*sign>=0 && abs(pwm-previous)<=20 && abs(pwm)<=300);
        assert(WheelSpeedController_GetState(wheel,&state));
        assert(fabsf(state.integrator_permille)<=50.0f);
        previous=pwm;
      }
      assert(pwm==0);
      /* No mode latch: same-state NORMAL immediately uses baseline P. */
      assert(WheelSpeedController_UpdateWithMode(wheel,target,235.0f*(float)sign,
        60.0f*(float)sign,WHEEL_SPEED_CONTROL_MODE_NORMAL,20U,&pwm));
      assert(WheelSpeedController_GetState(wheel,&state));
      assert(fabsf(state.proportional_permille+3.75f*(float)sign)<0.001f);
      assert(WheelSpeedController_Reset(wheel));
      assert(WheelSpeedController_UpdateWithMode(wheel,0,0,0,
        WHEEL_SPEED_CONTROL_MODE_PIVOT,20U,&pwm) && pwm==0);
    }
  int16_t unchanged=1234;
  assert(!WheelSpeedController_UpdateWithMode(MOTOR_DRIVER_WHEEL_RR,160,160,0,
    (WheelSpeedControlMode)9,20U,&unchanged) && unchanged==1234);
  assert(!WheelSpeedController_UpdateWithMode(MOTOR_DRIVER_WHEEL_RR,NAN,160,0,
    WHEEL_SPEED_CONTROL_MODE_PIVOT,20U,&unchanged));
  assert(!WheelSpeedController_UpdateWithMode(MOTOR_DRIVER_WHEEL_RR,160,INFINITY,0,
    WHEEL_SPEED_CONTROL_MODE_PIVOT,20U,&unchanged));
  puts("PASS: four-wheel bidirectional pivot overspeed damping; unchanged startup/underspeed/normal gains, no reverse braking, slew/integral/output limits and invalid-mode guard");

  ControlTiming_RecordStart(CONTROL_TIMING_MOTOR, 0xFFFFFFF0U);
  ControlTiming_RecordStart(CONTROL_TIMING_MOTOR, 0xFFFFFFFAU);
  ControlTiming_RecordStart(CONTROL_TIMING_MOTOR, 0x00000004U);
  assert(ControlTiming_NextWake(CONTROL_TIMING_MOTOR, 100U, 101U, 10U) == 110U);
  assert(ControlTiming_NextWake(CONTROL_TIMING_MOTOR, 100U, 110U, 10U) == 120U);
  assert(ControlTiming_NextWake(CONTROL_TIMING_MOTOR, 100U, 145U, 10U) == 155U);
  assert(ControlTiming_NextWake(CONTROL_TIMING_MOTOR, 0xFFFFFFFAU, 1U, 10U) == 4U);
  ControlTimingStatus timing;
  assert(ControlTiming_GetStatus(CONTROL_TIMING_MOTOR, &timing));
  assert(timing.period_min_ms == 10U && timing.period_max_ms == 10U);
  assert(timing.overrun_count == 2U && timing.missed_deadlines == 5U);
  assert(depth == 0U);
  puts("PASS: no overdue catch-up loop, exact deadline/late slots, timing statistics and tick wrap");
  return 0;
}
