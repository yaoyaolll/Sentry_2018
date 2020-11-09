#include "main.h"

#define bodan_init_angle 5800

_820r_motor_t motor_chassis[2];           //底盘电机结构体
_820r_motor_t motor_bodan;                //拨弹电机结构体
_6623_motor_t motor_pitch;                //PITCH电机结构体
_6623_motor_t motor_yaw;                  //YAW电机结构体
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


/*---------------PID参数初始化-----------*/
//底盘电机速度环PID初始化
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

//拨弹电机速度环PID结构体
Pid_Typedef speed_PID_bodan = {
	.P = 4.0f,
	.I = 0.2f,
	.D = 1.5f,
	.IMax = 500.0f
};

//拨弹电机位置环结构体
Pid_Typedef pos_PID_bodan = {
	.P = 0.22f,
	.I = 0.025f,
	.D = 0,
	.IMax = 500.0f
};    

//PITCH电机速度环PID结构体
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

//PITCH电机位置环结构体
Pid_Typedef pos_PID_pitch = {
	.P = 0.375f,
	.I = 0.0125f,
	.D = 0,
	.IMax = 20.0f
};

//YAW电机速度环结构体
Pid_Typedef speed_PID_yaw = {
	.P = 1500.0f,
	.I = 5.0f,
	.D = 250.0f,
	.IMax = 5.0f
};

//YAW电机位置环结构体
Pid_Typedef pos_PID_yaw = {
	.P = 0.00375f,
	.I = 0.0005f,
	.D = 0.0f,
	.IMax = 100.0f
};


/*---------------------CHASSIS---------------*/
/**
  * @brief  返回motor_chassis对象地址
  * @param  None
  * @retval None
  */
_820r_motor_t* get_motor_chassis(void)
{
	return motor_chassis;
}


/**
  * @brief  底盘控制
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
  * @brief  返回motor_bodan对象地址
  * @param  None
  * @retval None
  */
_820r_motor_t* get_motor_bodan(void)
{
	return &motor_bodan;
}

/**
  * @brief  拨盘控制,速度环
  * @param  None
  * @retval None
  */
//short bodan_flag = 1;
//short bodan_dir = 1;
//short seizingBulletTimeCnt;   //卡弹计数
//int pre_pos;

//short bodan_task(short shoot_flag)    
//{
//	if(shoot_flag == 1)
//	{
//		/*---------------------单独速度环-------------------*/
//		speed_PID_bodan.SetPoint = bodan_dir * 2000; 
//		/*--------------------卡弹反转---------------------*/
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
//	current_bodan = PID_Calc(&speed_PID_bodan, motor_bodan.real_speed);   //速度环计算
//	current_bodan = LIMIT_MAX_MIN(current_bodan, 9000, -9000);		
//	return current_bodan;
//}



/**
  * @brief  拨盘控制
  * @param  new_state:  1，开始射击 0，不射击 
	* @param  shoot_mode: 1，单发     3，3连发
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
	
	if(!heat_limit_task())      //热量限制
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
			else                          //堵转
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
//6623电机角度：2580-2140
/**
  * @brief  返回pitch_motor对象地址
  * @param  None
  * @retval None
  */
_6623_motor_t* get_motor_pitch(void)
{
	return &motor_pitch;
}

/**
  * @brief  Pitch轴电机控制
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



/*------------------过零检测-------------*/
/**
  * @brief  过零检测结构体参数初始化
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
  * @brief  位置式和速度式过零检测
	           Zero->ActualValue 表示检测量当前值
						 Zero->LastValue 表示检测量上一次值
						 Zero->CountCycle 表示检测量过零时越变值，即计数周期
						 Zero->PreError 表示检测量差值
						 使用此函数前要申明对应检测量结构体的 Zero->CountCycle与Zero->LastValue
  * @param  ZeroCheck_Typedef *Zero  过零检测结构体
  *         float value  待检测量
            short zero_check_mode：取值Position或Speed
  * @retval 取决于Zerocheck_mode，分别输出过零检测后位置值或速度值
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
  * @brief  过零检测执行函数
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
//基地4800
//荒地21200
/**
  * @brief  返回YawMotorData对象地址
  * @param  None
  * @retval None
  */
extern int chassis_camera_flag;	
_6623_motor_t* get_motor_yaw(void)
{
	return &motor_yaw;
}

/**
  * @brief  YAW轴电机控制
  * @param  None
  * @retval None
  */
short yaw_task(float setpoint, unsigned char flag)
{	
	if(flag == 1)       //速度环
	{    
		speed_PID_yaw.SetPoint = LIMIT_MAX_MIN(setpoint, 7.0f, -7.0f);
		current_yaw = PID_Calc(&speed_PID_yaw, GZ);
		current_yaw = LIMIT_MAX_MIN(current_yaw, 5000, -5000);
		
		return current_yaw;
	}
	else if(flag == 2)  //位置环
	{
		pos_PID_yaw.SetPoint = setpoint;
		speed_PID_yaw.SetPoint = PID_Calc(&pos_PID_yaw, YAW);  //此处不需要限幅
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
