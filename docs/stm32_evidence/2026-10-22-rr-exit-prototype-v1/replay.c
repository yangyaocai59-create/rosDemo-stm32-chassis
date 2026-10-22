/* OFFLINE diagnosis, not a plant model or firmware candidate.
 * Frozen real C controller; prescribed dc input; conditional hidden seeds.
 * Reconstructs post-state ring only; never opens hardware or serial. */
#include "fixture.h"

static float midpoint(int printed, float unit, float shift)
{
  return ((float)printed + (printed > 0 ? .5f : printed < 0 ? -.5f : 0) + shift)*unit;
}

int main(int argc, char **argv)
{
  const float shift = argc > 1 ? strtof(argv[1], NULL) : 0;
  const unsigned rr_episode = argc > 2 ? (unsigned)strtoul(argv[2], NULL, 10) : 200;
  const unsigned rr_stop_age = argc > 3 ? (unsigned)strtoul(argv[3], NULL, 10) : 140;
  const bool reverse_order = argc > 4 && strtoul(argv[4], NULL, 10) != 0;
  assert(fabsf(shift) <= .49f && rr_episode <= 600 && rr_stop_age <= 200);
  ChassisVelocityCommand command = {0, .3f};
  ChassisWheelTargets targets;
  assert(ChassisKinematics_ToWheelTargets(&command, 1600, &targets));
  WheelSpeedController_Init();
  for (unsigned w=0; w<4; w++) ChassisController_ClearPivotWheel((MotorDriverWheel)w);
  float measured[4] = {0};
  uint32_t last_motion[4] = {0};
  unsigned t, dt, count=0;
  int dc[4], cps[4], ff[4], ix[4], pwm[4];
  puts("t,wheel,target,raw,measured,ff,p,i,pwm,temporary,active,episode,moving,load,recovery");
  while (scanf("%u %u", &t, &dt) == 2)
  {
    assert(dt == 20);
    for (unsigned w=0; w<4; w++)
      assert(scanf("%d %d %d %d %d", &dc[w], &cps[w], &ff[w], &ix[w], &pwm[w]) == 5);
    for (unsigned index=0; index<4; index++)
    {
      const unsigned w=reverse_order ? 3-index : index;
      const MotorDriverWheel wheel = (MotorDriverWheel)w;
      const float target = targets.counts_per_second[w];
      const float base_ff = WheelSpeedController_GetFeedforward(wheel, target);
      if (count == 0)
      {
        measured[w] = midpoint(cps[w], 1, shift);
        s_states[w].integrator_permille = midpoint(ix[w], .1f, shift);
        s_states[w].feedforward_permille = midpoint(ff[w], .1f, shift);
        s_states[w].command_permille = (int16_t)pwm[w];
        s_previous_output[w] = (float)pwm[w]+shift;
        s_previous_target[w] = target;
        s_states[w].target_counts_per_second = target;
        s_pivot_breakaway_sign[w] = target > 0 ? 1 : -1;
        s_pivot_has_moved[w] = true;
        last_motion[w] = t - (w == 3 ? rr_stop_age : 0);
        if (w == 0 || w == 2)
        {
          s_front_pivot[w] = (WheelSpeedFrontPivotState){
            .active=true, .sign=target > 0 ? 1 : -1, .moving_ms=40,
            .load_permille=fabsf(s_states[w].feedforward_permille-base_ff)};
          if (w == 0)
            s_lf_first_start = (WheelSpeedFirstStartState){
              .elapsed_ms=600, .initialized=true, .has_moved=true, .consumed=true};
        }
        if (w == 1)
        {
          s_lr_running.enabled = true;
          s_lr_running.moving_ms = 40;
        }
        if (w == 3)
          s_rr_running = (WheelSpeedRrRunningState){
            .temporary_permille=s_states[w].feedforward_permille-base_ff,
            .enabled=true, .active=true, .spent=true,
            .under_ms=60, .episode_ms=rr_episode};
      }
      else
      {
        const float alpha=(float)dt/(CHASSIS_SPEED_FILTER_TIME_CONSTANT_MS+dt);
        measured[w] += alpha*(dc[w]*1000.0f/dt-measured[w]);
        if (dc[w]) last_motion[w]=t;
        else if (t-last_motion[w] >= CHASSIS_SPEED_STOP_TIMEOUT_MS) measured[w]=0;
        const float bias=ChassisController_GetFeedforwardBias(
          &command,wheel,target,measured[w],t,dt);
#ifndef BASELINE
        if (w==3) WheelSpeedController_ObserveRrRecoveryCounts(dc[w],dt,t,t);
#endif
        int16_t output;
        assert(WheelSpeedController_UpdateWithMode(wheel,target,measured[w],bias,
          WHEEL_SPEED_CONTROL_MODE_PIVOT,dt,&output));
      }
      const WheelSpeedControllerState *s=&s_states[w];
      const bool rear=w==1 || w==3;
      const float a=w==1 ? s_lr_running.temporary_permille :
                    w==3 ? s_rr_running.temporary_permille : 0;
      const bool active=w==1 ? s_lr_running.active : w==3 ? s_rr_running.active : false;
      const uint32_t episode=w==1 ? s_lr_running.episode_ms : w==3 ? s_rr_running.episode_ms : 0;
      const uint32_t moving=w==1 ? s_lr_running.moving_ms :
                            w==3 ? s_rr_running.moving_ms : s_front_pivot[w].moving_ms;
      printf("%u,%u,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,%d,%.9f,%d,%u,%u,%.9f,%.9f\n",
        t,w,target,dc[w]*1000.0f/dt,measured[w],s->feedforward_permille,
        s->proportional_permille,s->integrator_permille,s->command_permille,
        a,active,episode,moving,rear ? 0 : s_front_pivot[w].load_permille,
        rear ? 0 : s_front_pivot[w].recovery_permille);
    }
    count++;
  }
  assert(count==127);
  return 0;
}
