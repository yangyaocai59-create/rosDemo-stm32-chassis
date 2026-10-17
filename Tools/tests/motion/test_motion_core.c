/* Production PI/profile/health/timing functions; no serial or hardware. */
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include "chassis_motion_profile.h"
#include "wheel_speed_controller.h"
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

int main(void)
{
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
