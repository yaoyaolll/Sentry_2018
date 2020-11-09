#ifndef _MOTOR_TASK_H_
#define _MOTOR_TASK_H_
#include "stm32f10x.h"

typedef struct
{
	unsigned short angle;//0~8191(0x1fff)
	short real_speed;//     RPM
}_820r_motor_t;

typedef struct
{
	unsigned short angle;
	short real_flow;
	short set_flow;
}_6623_motor_t;

typedef struct 
{
	float circle;           //ת��Ȧ��
	float count_cycle;       //ת��һȦ���ܼ�������
	float last_value;        //����������һ�ε�ֵ	
	float actual_value;      //����������ǰֵ
	float pre_error;         //������жϲ�ֵ
}
zero_check_t;

void PID_chassis_init(void);
_820r_motor_t* get_motor_chassis(void);
short* chassis_task(short speed);
_820r_motor_t* get_motor_bodan(void);
short bodan_task(short shoot_flag, u8 shoot_mode);
_6623_motor_t* get_motor_pitch(void);
short pitch_task(float pos);
void zero_check_init(void);
float zero_check(zero_check_t *zero,float value,short zero_check_mode);
void zero_check_cal(void);
_6623_motor_t* get_motor_yaw(void);
short yaw_task(float setpoint, unsigned char flag);

#endif
