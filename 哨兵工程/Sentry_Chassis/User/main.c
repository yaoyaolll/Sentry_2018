#include "main.h"

/*--------------main.c内部变量-------------*/
int sys_cnt;
short encoder_cnt;
int encoder_all;
int ms5;
int ms10;
int wifi_time_cnt;

extern unsigned char wifi_rec_flag;
extern unsigned char wifi_choose_flag;
  
motor_setpoint_t motor_setpoint = {
	.chassis_speed = PATROL_CHASSIS_SPEED,
	.yaw_speed = PATROL_YAW_SPEED,
	.yaw_flag = YAW_SINGLE,
	.last_yaw_speed_dir = 1,
	.pitch_pos = PATROL_PITCH_POS_INIT,
	.pitch_dir = 1,
	.bodan_flag = 0,
	.bodan_mode = 1,
};

//主函数
int main(void)
{
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);	
	delay_ms(2500);
	System_Config();
	delay_ms(2500);
	System_Init();
	while(1)
	{
	}
}

//定时中断
void SysTick_Handler(void)
{
	remote_t* remote = get_remote();
	
	/*----------------计时器-------------*/
	ms5 ++;
	ms10 ++;
	sys_cnt ++;
	wifi_time_cnt ++;
	
	/*----------------传感器-------------*/
	encoder_cnt = Encoder_read();
	encoder_all += encoder_cnt;
	limit_sw_read();
	
	/*----------------过零检测-----------*/
	zero_check_cal();
	
	/*----------------断线处理-----------*/
	if(ms10 >= 10)
	{
		ms10 = 0;
		disconnect_handler(); 
	}
	if(wifi_rec_flag == 1)
	{
		wifi_rec_flag = 0;
		wifi_time_cnt = 0;
	}
	if(wifi_time_cnt >= 1000)     //1s
	{
		wifi_time_cnt = 1000;
		wifi_choose_flag = 4;
	}
	
	/*----------------模式选择-----------*/
	if(remote->rc.s1 == 1)
	{
		user_mode(ms5);
	}
	else if(remote->rc.s1 == 3)
	{
		sleep_mode(ms5);
	}
	else if(remote->rc.s1 == 2)
	{
		auto_mode(ms5); 
	}
	
	if(ms5 >= 5)
		ms5 = 0;
	
	remote->last_s1 = remote->rc.s1;
}


int get_sys_cnt(void)
{
	return sys_cnt;
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

void System_Config(void)
{
	CAN_Configuration();              
//	TIM2Init();
//	ENCODER_Configuration();
//	USART1_Configuration();         
//	USART2_Configuration();
//	USART3_Configuration();
//	LED_Configuration();	 
//	LimitSW_Configuration(); 
}

void System_Init(void)
{
	zero_check_init(); 
	gimbal_kalman_para_init(); 
	PID_chassis_init();
	SysTick_Config(72000);		  //初始化完成后开启1ms定时中断
}
