#include "main.h"

#define bodan_init_angle 5800

_820r_motor_t motor_chassis[2];           //���̵���ṹ��
_820r_motor_t motor_bodan;                //��������ṹ��
_6623_motor_t motor_pitch;                //PITCH����ṹ��
_6623_motor_t motor_yaw;                  //YAW����ṹ��
Pid_Typedef PID_chassis[2];
short current_chassis[2];
short current_bodan;
short current_pitch;
short current_yaw;
zero_check_t zero_check_pitch, zero_check_pitch_speed;
zero_check_t zero_check_yaw;
zero_check_t zero_check_bodan;
int gimbal_pitch, gimbal_pitch_speed;
int gimbal_yaw;
int bodan_angle;
int YAW;


/*---------------PID������ʼ��-----------*/
//���̵���ٶȻ�PID��ʼ��
void PID_chassis_init(void)
{
	int i;
	for(i = 0; i < 2; i ++)
	{
		PID_chassis[i].P = 3.5f;
		PID_chassis[i].I = 0.25f;
		PID_chassis[i].D = 5.0f;
		PID_chassis[i].IMax = 500.0f;
	}
}

//��������ٶȻ�PID�ṹ��
Pid_Typedef speed_PID_bodan = {
	.P = 4.0f,
	.I = 0.2f,
	.D = 1.5f,
	.IMax = 500.0f
};

//�������λ�û��ṹ��
Pid_Typedef pos_PID_bodan = {
	.P = 0.22f,
	.I = 0.025f,
	.D = 0,
	.IMax = 500.0f
};    

//PITCH����ٶȻ�PID�ṹ��
Pid_Typedef speed_PID_pitch = {
//	.P = 3000.0f,
//	.I = 50.0f,
//	.D = 500.0f,
//	.IMax = 50.0f
	.P = 1750.0f,
	.I = 50.0f,
	.D = 250.0f,
	.IMax = 50.0f
};

//PITCH���λ�û��ṹ��
Pid_Typedef pos_PID_pitch = {
	.P = 0.375f,
	.I = 0.0125f,
	.D = 0,
	.IMax = 20.0f
};

//YAW����ٶȻ��ṹ��
Pid_Typedef speed_PID_yaw = {
	.P = 1500.0f,
	.I = 5.0f,
	.D = 250.0f,
	.IMax = 5.0f
};

//YAW���λ�û��ṹ��
Pid_Typedef pos_PID_yaw = {
	.P = 0.00375f,
	.I = 0.0005f,
	.D = 0.0f,
	.IMax = 100.0f
};


/*---------------------CHASSIS---------------*/
/**
  * @brief  ����motor_chassis�����ַ
  * @param  None
  * @retval None
  */
_820r_motor_t* get_motor_chassis(void)
{
	return motor_chassis;
}


/**
  * @brief  ���̿���
  * @param  None
  * @retval None
  */
short* chassis_task(short speed)  
{
	int i;
	
	PID_chassis[0].SetPoint = LIMIT_MAX_MIN(-speed, 10000, -10000);
	PID_chassis[1].SetPoint = LIMIT_MAX_MIN(speed, 10000, -10000);
	
	for(i = 0; i < 2; i ++)
	{
		current_chassis[i] = LIMIT_MAX_MIN(PID_Calc(&PID_chassis[i], motor_chassis[i].real_speed), 4800, -4800);
	}
	return current_chassis;
}

/*---------------------BODAN---------------*/
/**
  * @brief  ����motor_bodan�����ַ
  * @param  None
  * @retval None
  */
_820r_motor_t* get_motor_bodan(void)
{
	return &motor_bodan;
}

/**
  * @brief  ���̿���,�ٶȻ�
  * @param  None
  * @retval None
  */
//short bodan_flag = 1;
//short bodan_dir = 1;
//short seizingBulletTimeCnt;   //��������
//int pre_pos;

//short bodan_task(short shoot_flag)    
//{
//	if(shoot_flag == 1)
//	{
//		/*---------------------�����ٶȻ�-------------------*/
//		speed_PID_bodan.SetPoint = bodan_dir * 2000; 
//		/*--------------------������ת---------------------*/
//		if(ABS(motor_bodan.real_speed) < 10)
//		{
//			seizingBulletTimeCnt ++;
//			if(seizingBulletTimeCnt >= 20)               //100ms
//			{
//				seizingBulletTimeCnt = 0;
//				bodan_flag = 1;
//				bodan_dir *= -1;
//			}
//		}
//	}
//	else 
//		speed_PID_bodan.SetPoint = 0;
//	
//	current_bodan = PID_Calc(&speed_PID_bodan, motor_bodan.real_speed);   //�ٶȻ�����
//	current_bodan = LIMIT_MAX_MIN(current_bodan, 9000, -9000);		
//	return current_bodan;
//}



/**
  * @brief  ���̿���
  * @param  new_state:  1����ʼ��� 0������� 
	* @param  shoot_mode: 1������     3��3����
  * @retval None
  */
u8 bodan_act_on;
int bullet_num;
short bodan_task(short new_state, u8 shoot_mode)
{
	static int bullet_time_cnt;
	static short time_long;
	int bodan_pos_error = pos_PID_bodan.SetPoint - bodan_angle;
	static int bullet_reverse_cnt;
	
	if(!heat_limit_task())      //��������
		new_state = 0;
	
	if(new_state && !bodan_act_on)
	{
		bodan_act_on = 1;
		if(shoot_mode == 1)
		{
			bullet_num ++;
			time_long = 30;
		}
		else if(shoot_mode == 3)
		{
			bullet_num += 3;
			time_long = 90;
		}
	}
	if(bodan_act_on)
	{
		bullet_time_cnt ++;
		pos_PID_bodan.SetPoint = bullet_num*8192*36/7 + bodan_init_angle;
		if(bullet_time_cnt == time_long)  
		{
			if(ABS(bodan_pos_error) < 4000)
			{
				bullet_time_cnt = 0;
				bodan_act_on = 0;
			}
			else                          //��ת
			{
				time_long += 30;
				bullet_num -= 1;
				bullet_reverse_cnt ++;
				if(bullet_reverse_cnt >= 3)
				{
					bullet_time_cnt = 0;
					bodan_act_on = 0;
					bullet_reverse_cnt = 0;
					bullet_num = (int)(bodan_angle/(8192*36/7));
				}
			}
		}
	}
	if(bodan_act_on)
	{
		speed_PID_bodan.SetPoint = LIMIT_MAX_MIN(PID_Calc(&pos_PID_bodan, bodan_angle), 5000, -5000);
		current_bodan = LIMIT_MAX_MIN(PID_Calc(&speed_PID_bodan, motor_bodan.real_speed), 9000, -9000);
	}
	else	if(!bodan_act_on)
	{
		speed_PID_bodan.SetPoint = 0;
		current_bodan = LIMIT_MAX_MIN(PID_Calc(&speed_PID_bodan, motor_bodan.real_speed), 100, -100);
	}
	return current_bodan;
}


/*----------------PITCH-------------------*/
//6623����Ƕȣ�2580-2140
/**
  * @brief  ����pitch_motor�����ַ
  * @param  None
  * @retval None
  */
_6623_motor_t* get_motor_pitch(void)
{
	return &motor_pitch;
}

/**
  * @brief  Pitch��������
  * @param  None
  * @retval None
  */
float _pos_pitch;
float _speed_pitch;
//25~33
//{
//	3000
//	30
//	250
//}
//4~25
//{
//	1250
//	50
//	500
//}
float last_PITCH;
short pitch_task(float pos) 
{
	remote_t* remote = get_remote();
	
	_pos_pitch = LIMIT_MAX_MIN(pos, 26.0f, 1.0f);
	pos_PID_pitch.SetPoint = _pos_pitch;
	speed_PID_pitch.SetPoint = LIMIT_MAX_MIN(PID_Calc(&pos_PID_pitch, PITCH), 5.0f, -5.0f);
	current_pitch = PID_Calc(&speed_PID_pitch, GY);
	current_pitch = LIMIT_MAX_MIN(current_pitch, 4800, -4800);
	
//	_speed_pitch = LIMIT_MAX_MIN((1024 - remote->rc.ch3)/120.0f, 5.0f, -5.0f);
//	speed_PID_pitch.SetPoint = _speed_pitch;
//	current_pitch = PID_Calc(&speed_PID_pitch, GY);
//	current_pitch = LIMIT_MAX_MIN(current_pitch, 4800, -4800);
	
	last_PITCH = PITCH;
	return current_pitch;
}



/*------------------������-------------*/
/**
  * @brief  ������ṹ�������ʼ��
  * @param  None
  * @retval None
  */
void zero_check_init(void)
{
	zero_check_pitch.count_cycle = 8192;
	zero_check_pitch.last_value = motor_pitch.angle;
	
	zero_check_yaw.count_cycle = 32767;
	zero_check_yaw.last_value = motor_yaw.angle;
	
	zero_check_bodan.count_cycle = 8192;
	zero_check_bodan.last_value = motor_bodan.angle;
}

/**
  * @brief  λ��ʽ���ٶ�ʽ������
	           Zero->ActualValue ��ʾ�������ǰֵ
						 Zero->LastValue ��ʾ�������һ��ֵ
						 Zero->CountCycle ��ʾ���������ʱԽ��ֵ������������
						 Zero->PreError ��ʾ�������ֵ
						 ʹ�ô˺���ǰҪ������Ӧ������ṹ��� Zero->CountCycle��Zero->LastValue
  * @param  ZeroCheck_Typedef *Zero  ������ṹ��
  *         float value  �������
            short zero_check_mode��ȡֵPosition��Speed
  * @retval ȡ����Zerocheck_mode���ֱ�����������λ��ֵ���ٶ�ֵ
  */
float zero_check(zero_check_t *zero,float value,short zero_check_mode)
{
	zero->actual_value = value;
	
	zero->pre_error = zero->actual_value - zero->last_value;
	zero->last_value = zero->actual_value;
	
	if(zero->pre_error > 0.7f*zero->count_cycle)
	{
		zero->pre_error = zero->pre_error - zero->count_cycle;
		zero->circle++;
	}
	if(zero->pre_error < -0.7f*zero->count_cycle)
	{
		zero->pre_error = zero->pre_error + zero->count_cycle;
		zero->circle--;
	}
	
	if(zero_check_mode == 1)
		return zero->actual_value - zero->circle*zero->count_cycle;
	else if(zero_check_mode == 2)
	  return zero->pre_error;
	else 
		return 0;
}

/**
  * @brief  ������ִ�к���
  * @param  None
  * @retval None
  */
void zero_check_cal(void)
{
	gimbal_pitch = zero_check(&zero_check_pitch, motor_pitch.angle, 1);
	gimbal_pitch = zero_check(&zero_check_pitch_speed, motor_pitch.angle, 2);
	gimbal_yaw = zero_check(&zero_check_yaw, motor_yaw.angle, 1);
	YAW = gimbal_yaw;
	bodan_angle = zero_check(&zero_check_bodan, motor_bodan.angle, 1);
}



/*----------------YAW--------------------*/
//����4800
//�ĵ�21200
/**
  * @brief  ����YawMotorData�����ַ
  * @param  None
  * @retval None
  */
extern int chassis_camera_flag;	
_6623_motor_t* get_motor_yaw(void)
{
	return &motor_yaw;
}

/**
  * @brief  YAW��������
  * @param  None
  * @retval None
  */
short yaw_task(float setpoint, unsigned char flag)
{	
	if(flag == 1)       //�ٶȻ�
	{    
		speed_PID_yaw.SetPoint = LIMIT_MAX_MIN(setpoint, 7.0f, -7.0f);
		current_yaw = PID_Calc(&speed_PID_yaw, GZ);
		current_yaw = LIMIT_MAX_MIN(current_yaw, 5000, -5000);
		
		return current_yaw;
	}
	else if(flag == 2)  //λ�û�
	{
		pos_PID_yaw.SetPoint = setpoint;
		speed_PID_yaw.SetPoint = PID_Calc(&pos_PID_yaw, YAW);  //�˴�����Ҫ�޷�
		if(chassis_camera_flag == 3 || chassis_camera_flag == 4)
			speed_PID_yaw.SetPoint = LIMIT_MAX_MIN(speed_PID_yaw.SetPoint, 2.0f, -2.0f);
		else 
			speed_PID_yaw.SetPoint = LIMIT_MAX_MIN(speed_PID_yaw.SetPoint, 5.0f, -5.0f);
		current_yaw = LIMIT_MAX_MIN(PID_Calc(&speed_PID_yaw, GZ), 5000, -5000);
	
		return current_yaw;
	}
	else
	{
		return 0;
	} 
}
