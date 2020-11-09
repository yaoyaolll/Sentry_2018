#ifndef __MAIN_H__
#define __MAIN_H__

#include "stm32f10x.h"
#include <stdio.h>
#include <stdlib.h>
#include "math.h"
#include "string.h"
#include "stdint.h"
#include "arm_math.h"

#include "can.h"
#include "usart1.h"
#include "usart2.h"
#include "usart3.h"
#include "encoder.h"
//#include "frictionWheel.h"
#include "pid.h"
#include "CRC_algorithm.h"
#include "tim2.h"
#include "limit_switch.h"
#include "led.h"

#include "motor_task.h"
#include "data_rec_task.h" 
#include "data_send_task.h"
#include "control_task.h"
#include "kalman_filter.h"

#define ABS(x) ((x)>0? (x):(-(x))) 
#define LIMIT_MAX_MIN(x, max, min)	(((x) <= (min)) ? (min):(((x) >= (max)) ? (max) : (x)))

typedef struct{
	short chassis_speed;
	short last_chassis_dir;
	float yaw_speed;
	float last_yaw_speed_dir;
	float yaw_pos;
	unsigned char yaw_flag;
	float pitch_pos;
	short pitch_dir;
	unsigned char bodan_flag;
	unsigned char bodan_mode;
}motor_setpoint_t;

#define YAW_SINGLE 1
#define YAW_DOUBLE 2

extern float PITCH;
extern float GY;
extern int   YAW;
extern float GZ;
extern motor_setpoint_t motor_setpoint;
extern short encoder_cnt;
extern int encoder_all;

#define PATROL_PITCH_POS_INIT  10.0f
#define PATROL_CHASSIS_SPEED 3000
#define DODGE_CHASSIS_SPEED  6000
#define PATROL_YAW_SPEED 1.0f
#define REMAIN_LIMIT_HP  750

int get_sys_cnt(void);
void delay_ms(unsigned int t);
void System_Config(void);
void System_Init(void);

#endif
