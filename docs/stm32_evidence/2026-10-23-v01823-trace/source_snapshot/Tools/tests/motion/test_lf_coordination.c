/* Artificial private seeds and prescribed feedback: invariants, NOT a plant. */
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include "../../../Application/Chassis/wheel_speed_controller.c"

static MotorDriverWheel W;
static float seed(float sign,float magnitude,float load,float i,float recovery,
                  bool recovering)
{
  WheelSpeedController_Init();
  const float target=sign*magnitude;
  s_states[W].integrator_permille=sign*i;
  s_states[W].feedforward_permille=WheelSpeedController_GetFeedforward(W,target)+
    sign*(load+recovery);
  s_previous_target[W]=target;
  s_front_pivot[W]=(WheelSpeedFrontPivotState){.active=true,.sign=(int8_t)sign,
    .load_permille=load,.recovery_permille=recovery,.moving_ms=40,
    .load_overspeed_ms=60,.recovery_ms=100,.recovering=recovering,
    .exhausted=recovering};
  assert(WheelSpeedController_PrimeOutput(W,(int16_t)(sign*100)));
  return target;
}
static void step(float target,float measured,unsigned dt)
{
  const int previous=s_states[W].command_permille;
  int16_t pwm;
  assert(WheelSpeedController_UpdateWithMode(W,target,measured,0,
    WHEEL_SPEED_CONTROL_MODE_PIVOT,dt,&pwm));
  const WheelSpeedFrontPivotState *f=&s_front_pivot[W];
  assert(abs(pwm)<=300 && abs(pwm-previous)<=(int)dt+1);
  assert(fabsf(s_states[W].integrator_permille)<=50.001f);
  assert(s_states[W].integrator_permille*f->sign+f->recovery_permille<=50.001f);
  assert(f->load_permille>=0 && f->load_permille<=
    fabsf(WheelSpeedController_GetPivotLoadFeedforward(W,target))+0.001f);
  assert(f->recovery_permille>=0 && f->recovery_permille<=
    50*fminf(fabsf(target)/160,1)+0.001f);
}
static void aggregate(float sign,unsigned dt,float magnitude)
{
  const float cap=fabsf(WheelSpeedController_GetPivotLoadFeedforward(W,sign*magnitude));
  const float authority=50*fminf(magnitude/160,1);
  for(unsigned l=0;l<3;l++) for(unsigned r=0;r<3;r++)
  {
    float target=seed(sign,magnitude,cap*l/3,10,
      fminf(30,authority)*r/3,true);
    s_front_pivot[W].still_ms=60;
    s_front_pivot[W].moving_ms=0;
    float before=s_front_pivot[W].load_permille+s_front_pivot[W].recovery_permille;
    step(target,0,dt);
    float after=s_front_pivot[W].load_permille+s_front_pivot[W].recovery_permille;
    assert(after-before<=250*dt/1000.0f+0.002f && after>=before-0.002f);

    target=seed(sign,magnitude,cap*l/3,10,fminf(30,authority)*r/3,true);
    before=s_front_pivot[W].load_permille+s_front_pivot[W].recovery_permille;
    const float band=fmaxf(20,magnitude*0.1f);
    step(target,sign*(magnitude+band+100),dt);
    after=s_front_pivot[W].load_permille+s_front_pivot[W].recovery_permille;
    assert(before-after<=400*dt/1000.0f+0.002f && after<=before+0.002f);
    assert(!s_front_pivot[W].recovering);
  }
}
static void moving_restore(float sign,unsigned dt)
{
  float t=seed(sign,160,20,10,0,false);
  step(t,sign*100,dt);
  const float increment=40*dt/1000.0f;
  assert(fabsf(s_front_pivot[W].load_permille-20-increment)<0.002f);
  assert(fabsf(s_states[W].integrator_permille*sign-10)<0.002f);
  assert(s_front_pivot[W].still_ms==0 && !s_front_pivot[W].recovering);
  t=seed(sign,160,20,10,0,false);
  step(t,sign*150,dt); /* inside band: ordinary I, no load restoration */
  assert(s_front_pivot[W].load_permille==20);
  t=seed(sign,160,20,10,0,false);
  step(t,-sign*100,dt); /* opposite motion never qualifies restoration */
  assert(s_front_pivot[W].load_permille==20 && s_front_pivot[W].moving_ms==0);
  t=seed(sign,160,20,-10,0,false);
  step(t,sign*100,dt); /* unwind opposing I first, no added load + I */
  assert(s_front_pivot[W].load_permille==20);
  t=seed(sign,240,0,0,0,false); /* RF recorded gap: no >10-permille I prerequisite */
  step(t,sign*180,dt);
  assert(s_front_pivot[W].load_permille>0);
  assert(s_states[W].integrator_permille==0);
}
static void finite_outcomes(float sign,unsigned dt,float magnitude)
{
  const float cap=fabsf(WheelSpeedController_GetPivotLoadFeedforward(W,sign*magnitude));
  float t=seed(sign,magnitude,cap,20,0,false);
  const float band=fmaxf(20,magnitude*0.1f);
  for(unsigned elapsed=0;elapsed<7000;elapsed+=dt)
    step(t,sign*(magnitude+band+20),dt);
  assert(s_front_pivot[W].load_permille==0 &&
    s_front_pivot[W].recovery_permille==0);
  assert(s_states[W].integrator_permille*sign<=0);

  t=seed(sign,magnitude,0,0,0,false);
  s_front_pivot[W].moving_ms=0;
  for(unsigned elapsed=0;elapsed<2000;elapsed+=dt) step(t,0,dt);
  assert(s_front_pivot[W].recovery_failed && !s_front_pivot[W].recovering &&
    s_front_pivot[W].recovery_permille==0);
}
int main(void)
{
  const unsigned periods[]={1,10,20,25,59,60,100};
  const float targets[]={20,40,160,280,1600};
  const MotorDriverWheel fronts[]={MOTOR_DRIVER_WHEEL_LF,MOTOR_DRIVER_WHEEL_RF};
  for(unsigned wheel=0;wheel<2;wheel++)
  for(int sign=-1;sign<=1;sign+=2) for(unsigned d=0;d<7;d++)
  {
    W=fronts[wheel];
    moving_restore((float)sign,periods[d]);
    for(unsigned m=0;m<5;m++)
    {
      aggregate((float)sign,periods[d],targets[m]);
      finite_outcomes((float)sign,periods[d],targets[m]);
    }
  }
  puts("PASS: both fronts load+transient rise <=250/s, fall <=400/s; existing individual caps, 50-permille shared I/recovery, PWM and slew bounds remain");
  puts("PASS: confirmed moving underspeed restores bounded load BEFORE near-rest without simultaneous positive I; band/opposite motion/opposing-I gates retained");
  puts("PASS: both fronts/directions, 20/40/160/280/1600 cps, dt1/10/20/25/59/60/100ms: sustained overspeed unloads fully; continuous rest still has finite failure");
  return 0;
}
