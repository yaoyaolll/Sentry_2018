#ifndef __MAIN_H__
#define __MAIN_H__

#include "stm32f10x.h"
#include <stdio.h>
#include <stdlib.h>
#include "math.h"
#include "string.h"

#include "can.h"
#include "usart1.h"
#include "usart2.h"
#include "usart3.h"
#include "frictionWheel.h"
#include "pid.h"
#include "algorithmOfCRC.h"
#include "laser.h"
#include "solenoidSW.h"
#include "led.h"
#include "pid.h"

#include "RemoteTask.h"
#include "GYROTask.h"
#include "PC_Task.h"
 

#include <spi.h>
#include <ADIS16448.h>
#include <AHRS.h>
#include <delay.h>

typedef struct 
{
	float circle;           //转过圈数
	float count_cycle;       //转过一圈的总计数周期
	float last_value;        //检测过零量上一次的值	
	float actual_value;      //检测过零量当前值
	float pre_error;         //检测量判断差值
}
zero_check_t;

#define SLAVE_ISCNT_ID 0x555

void system_Config(void);
void system_Init(void);
void delay_ms(unsigned int t);

#define ABS(x) ((x)>0? (x):(-(x))) 
#define LIMIT_MAX_MIN(x, max, min)	(((x) <= (min)) ? (min):(((x) >= (max)) ? (max) : (x)))

extern _820r_motor_t motor_bodan;

void zero_check_cal(void);
float zero_check(zero_check_t *zero,float value,short zero_check_mode);
void zero_check_init(void);
short bodan_task(short new_state, u8 shoot_mode);
void _820r_send_task(short chassis_data1, short chassisdata2, short bodan_data);
void gimbal_send_task(short yaw, short pitch) ;
short pitch_task(float pos) ;

#endif
