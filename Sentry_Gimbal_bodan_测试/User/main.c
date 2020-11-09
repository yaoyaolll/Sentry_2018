#include "main.h" 

/*******************
1.���߼��
2.ģ����߶Բ�
3.����ģʽ

�ڱ�Ѫ����3000
��ʼ������500
ǹ���������ޣ�360
ÿ��������ȴֵ��72
*******************/

/*�����Ǳ���*/
extern float m_yaw,m_yaw_last;
extern int m_circle,zero_cnt;
extern int  sys_cnt;

short FrictionSpeed;
zero_check_t zero_check_bodan;
int bodan_angle;
int current_bodan;
extern _6623_motor_t motor_pitch;
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

extern RemoteRec_TypeDef RemoteData;
int main(void)
{
 	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	delay_ms(2500);
	system_Config();
	system_Init();	
//	quadraticSmooth5(in,out,5); 
	while(1)
	{
	}
}

void system_Config(void)   
{ 
	FrictionWheel_Configuration();//��
	LASER_Configuration();     //��
	SolenoidSW_Configuration();//��
	LED_Configuration();       //��
	CAN_Configuration();       //��
	USART1_Configuration();    //��
	USART2_Configuration();	   //��
	USART3_Configuration();    //��
	
	TIM2Init();
	//TIM3Init();       //TIM3�����������Ħ����
	SPI1_Init();	
	ADIS_Init();
	EXTIX_Init();
	delay_ms(1000);
}

void system_Init(void)
{
	zero_check_init();
	SysTick_Config(72000);		//1ms��ʱ�ж�
	FrictionSpeed = 84;   //��ʼ�ر�Ħ����
}



void delay_ms(unsigned int t) 
{
	int i;
	for( i=0;i<t;i++)
	{
		int a=10300;
 		while(a--);
	}
}


//short bullet_dir = 1;
u8 bodan_act_on;
int bullet_num;
int bodan_init_angle = 5800;
short bodan_task(short new_state, u8 shoot_mode)
{
	static int bullet_time_cnt;
	static short time_long;
//	short shoot_time_threshold[2];
	int bodan_pos_error = pos_PID_bodan.SetPoint - bodan_angle;

//	if(new_state == 1 && !bodan_act_on)
//	{
//		if(shoot_mode == 3)
//		{
//			shoot_time_threshold[0] = 60;  //300ms
//			shoot_time_threshold[1] = 80;  //400ms
//			pos_PID_bodan.SetPoint += 3*41862*bullet_dir; 
//		}
//		else if(shoot_mode == 1)
//		{
//			shoot_time_threshold[0] = 100;  //100ms
//			shoot_time_threshold[1] = 200;  //200ms
//			pos_PID_bodan.SetPoint += 41862*bullet_dir;
//		}
//		bodan_act_on = 1;       //����������ʼ
//	}
//	if(bodan_act_on == 1)
//	{
//		bullet_time_cnt ++;         //����ʱ�����
//		if(bullet_time_cnt == shoot_time_threshold[0])   
//		{
//			if(ABS(bodan_pos_error) > 4000)  //δ�ﵽԤ��λ�ã���ǰλ�÷�תһ��
//			{
//				pos_PID_bodan.SetPoint = bodan_angle - 41862;
//			}
//			else                             //����Ԥ��λ�� 
//			{
//				bullet_time_cnt = 0;
//				bodan_act_on = 0;
//			}
//		}
//		else if(bullet_time_cnt >= shoot_time_threshold[1]) 
//		{
//			bullet_time_cnt = 0;
//			bodan_act_on = 0;     //�����������
//		}
//	}
	
	
	if(new_state && !bodan_act_on)
	{
		bodan_act_on = 1;
		if(shoot_mode == 1)
		{
			bullet_num ++;
			time_long = 50;
		}
		else if(shoot_mode == 3)
		{
			bullet_num += 3;
			time_long = 150;
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

//		PidBodanMotorPos.SetPoint=-(BulletNum*8192*36/7/10+Bodan_Init_Angle);
//		PidBodanMotorSpeed.SetPoint = LIMIT_MIN_MAX( PID_Calc(&PidBodanMotorPos, ZeroCheck_BodanPosOutPut()), -6000,6000);//-6000, 6000);//?????��
////		PidBodanMotorSpeed.SetPoint=RemoteControl_Left_Right()*5;
//		BodanSpeed = (float)PID_Calc(&PidBodanMotorSpeed,(float)GetBodanMotorSpeed());
//		CurrentSendBodan=LIMIT_MIN_MAX((short)BodanSpeed,-10000,10000);// -9000,9000);//Max:-10000~10000
//		BodanCan1Send(CurrentSendBodan);	
//	}
//	else if(Shoot_Flag==0||Heat_flag==0)
//	{
//		PidBodanMotorSpeed.SetPoint=0;
//	  BodanSpeed = (float)PID_Calc(&PidBodanMotorSpeed,(float)GetBodanMotorSpeed());
//	  CurrentSendBodan=LIMIT_MIN_MAX((short)BodanSpeed, -100,100);//Max:-10000~10000
//	  BodanCan1Send(CurrentSendBodan);	
//	}



/*------------------������-------------*/
/**
  * @brief  ������ṹ�������ʼ��
  * @param  None
  * @retval None
  */
void zero_check_init(void)
{
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
	bodan_angle = zero_check(&zero_check_bodan, motor_bodan.angle, 1);
}

void _820r_send_task(short chassis_data1, short chassisdata2, short bodan_data)
{
	CanTxMsg tx_message;
	
	tx_message.IDE = CAN_ID_STD;    
	tx_message.RTR = CAN_RTR_DATA; 
	tx_message.DLC = 0x08;    
	tx_message.StdId = 0x200;

	tx_message.Data[0] = (unsigned char)((chassis_data1>>8)&0xff);
	tx_message.Data[1] = (unsigned char)(chassis_data1&0xff);  
	tx_message.Data[2] = (unsigned char)((chassisdata2>>8)&0xff);
	tx_message.Data[3] = (unsigned char)(chassisdata2&0xff); 
	tx_message.Data[4] = (unsigned char)((bodan_data>>8)&0xff);
	tx_message.Data[5] = (unsigned char)(bodan_data&0xff);  	

	CAN_Transmit(CAN1, &tx_message);
}

void gimbal_send_task(short yaw, short pitch)  
{
	CanTxMsg tx_message;
	
	tx_message.IDE = CAN_ID_STD;    
	tx_message.RTR = CAN_RTR_DATA; 
	tx_message.DLC = 0x08;    
	tx_message.StdId = 0x1FF;

	tx_message.Data[0] = (unsigned char)((pitch>>8)&0xff);
	tx_message.Data[1] = (unsigned char)(pitch&0xff);  	
	tx_message.Data[2] = (unsigned char)((yaw>>8)&0xff);
	tx_message.Data[3] = (unsigned char)(yaw&0xff);  	

	CAN_Transmit(CAN1, &tx_message);
}

int _speed_pitch;
extern RemoteRec_TypeDef RemoteData;
short current_pitch;
//PITCH���λ�û��ṹ��
Pid_Typedef pos_PID_pitch = {
	.P = 0.375f,
	.I = 0.0125f,
	.D = 0,
	.IMax = 20.0f
};

//PITCH����ٶȻ�PID�ṹ��
Pid_Typedef speed_PID_pitch = {
//	.P = 3000.0f,
//	.I = 50.0f,
//	.D = 500.0f,
//	.IMax = 50.0f
	.P = 0.0f,
	.I = 0.0f,
	.D = 0.0f,
	.IMax = 50.0f
};
Pid_Typedef single_pos_PID = {
	.P = 0,
	.I = 0,
	.D = 0,
	.IMax = 50.0f,
};
extern float MPU_gyro[3];
extern float comp_pitch;
short current_pitch;
short pitch_task(float pos) 
{
//	single_pos_PID.SetPoint = pos;//2600 + (1024 - RemoteData.rc.ch3);
//	single_pos_PID.SetPoint = LIMIT_MAX_MIN(single_pos_PID.SetPoint, 3000, 2450);
//	current_pitch = PID_Calc(&single_pos_PID, motor_pitch.angle);
//		current_pitch = LIMIT_MAX_MIN(current_pitch, 4800, -4800);
	pos_PID_pitch.SetPoint = LIMIT_MAX_MIN(pos, 26.0f, 1.0f);
	speed_PID_pitch.SetPoint = LIMIT_MAX_MIN(PID_Calc(&pos_PID_pitch, comp_pitch), 5.0f, -5.0f);
	current_pitch = -PID_Calc(&speed_PID_pitch, MPU_gyro[1]);
	current_pitch = LIMIT_MAX_MIN(current_pitch, 4800, -4800);
	
//	_speed_pitch = LIMIT_MAX_MIN((1024 - RemoteData.rc.ch3)/120.0f, 5.0f, -5.0f);
//	speed_PID_pitch.SetPoint = _speed_pitch;
//	current_pitch = PID_Calc(&speed_PID_pitch, MPU_gyro[1]	);
//	current_pitch = LIMIT_MAX_MIN(current_pitch, 4800, -4800);
	
	return current_pitch;
}


int friction_wheel_sw;
float pitch_pos ;
short _current_pitch;
void SysTick_Handler(void)
{	

//	current_pitch = pitch_task(motor_setpoint.pitch_pos);
	static int ms5;
//	RemoteRec_TypeDef* RemoteData = Get_RemoteData();
	ms5 ++;
//	
//	zero_check_cal();
//	
//	friction_wheel_sw = 1024 - RemoteData->rc.ch1;
//	if(friction_wheel_sw > 600)
//		FrictionWheel_Set(84);
//	else if(friction_wheel_sw < -600)
//		FrictionWheel_Set(112);
//	
	if(ms5 == 5)
	{
		ms5 = 0;
		
//		pitch_pos += (ABS(1024 - RemoteData.rc.ch3)<10?0:(1024 - RemoteData.rc.ch3))*0.0005f;
		pitch_pos =  LIMIT_MAX_MIN(pitch_pos, 26.0f, 1.0f);
		_current_pitch = pitch_task(10);
		gimbal_send_task(_current_pitch, _current_pitch);

//	single_pos_PID.SetPoint = 2800.0f;//2600 + (1024 - RemoteData.rc.ch3);
//	single_pos_PID.SetPoint = LIMIT_MAX_MIN(single_pos_PID.SetPoint, 3000, 2450);
//	_current_pitch = PID_Calc(&single_pos_PID, motor_pitch.angle);
//		_current_pitch = LIMIT_MAX_MIN(_current_pitch, 4800, -4800);
//		gimbal_send_task(_current_pitch, _current_pitch);
	}
}
