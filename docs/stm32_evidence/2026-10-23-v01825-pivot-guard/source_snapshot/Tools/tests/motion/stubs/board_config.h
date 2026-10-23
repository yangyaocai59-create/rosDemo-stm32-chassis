#ifndef TEST_MOTION_BOARD_H_
#define TEST_MOTION_BOARD_H_
#include "main.h"
#define BOARD_PWM_LF_TIMER_HANDLE (&htim8)
#define BOARD_PWM_LR_TIMER_HANDLE (&htim8)
#define BOARD_PWM_RF_TIMER_HANDLE (&htim8)
#define BOARD_PWM_RR_TIMER_HANDLE (&htim8)
#define BOARD_PWM_LF_CHANNEL 0U
#define BOARD_PWM_LR_CHANNEL 1U
#define BOARD_PWM_RF_CHANNEL 2U
#define BOARD_PWM_RR_CHANNEL 3U
#define BOARD_MOTOR_LF_FORWARD_SIGN 1
#define BOARD_MOTOR_LR_FORWARD_SIGN -1
#define BOARD_MOTOR_RF_FORWARD_SIGN -1
#define BOARD_MOTOR_RR_FORWARD_SIGN 1
void Board_EmergencyMotorShutdown(void);
void Board_ForceMotorSafeState(void);
#endif
