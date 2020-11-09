#include "main.h" 

/*******************
1.断线检测
2.模块掉线对策
3.各种模式

哨兵血量：3000
初始弹量：500
枪口热量上限：360
每秒热量冷却值：72
*******************/

/*陀螺仪变量*/
extern float m_yaw,m_yaw_last;
extern int m_circle,zero_cnt;
extern int  is_tx2_connect_cnt;
extern int  is_can_disconnect_cnt;
extern int  is_gyro_disconnect_cnt;
short FrictionSpeed;

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
	FrictionWheel_Configuration();//√
	LASER_Configuration();     //√
	//SolenoidSW_Configuration();//√
	LED_Configuration();       //√
	CAN_Configuration();       //√
	USART1_Configuration();    //√
	USART2_Configuration();	   //√
	USART3_Configuration();    //√
	
	TIM2Init();
	SPI1_Init();	
	ADIS_Init();
	EXTIX_Init();
	delay_ms(1000);
}

void system_Init(void)
{
	SysTick_Config(72000);		//1ms定时中断
	FrictionSpeed = 84;   //初始关闭摩擦轮
}

void SysTick_Handler(void)
{	
	static int ms50;
	static int ms100;
	RemoteRec_TypeDef* RemoteData = Get_RemoteData();
	ms50 ++;

	if(ms50 >= 50)
	{
		ms50 = 0;
		GPIO_SetBits(GPIOA, GPIO_Pin_5);    //关闭PC接收数据显示灯，PA5，红灯
	}
	
//	//TX2串口断线检测
//	is_tx2_connect_cnt ++;
//	if(is_tx2_connect_cnt >= 100)   //100ms
//	{
//		is_tx2_connect_cnt = 0;
//		DMA_Cmd(DMA1_Channel5, DISABLE);
//		USART1_Configuration();
//	}
//	
//	//CAN断线检测
//	is_can_disconnect_cnt ++;
//	if(is_can_disconnect_cnt >= 10) //10ms
//	{
//		is_can_disconnect_cnt  = 0;
//		CAN_Configuration();
//	}
//	
//	//陀螺仪断线检测
//	is_gyro_disconnect_cnt ++;
//	if(is_gyro_disconnect_cnt >= 10) //10ms
//	{
//		is_gyro_disconnect_cnt = 0;
//		TIM2Init();
//		SPI1_Init();	
//		ADIS_Init();
//		EXTIX_Init();
//	}
	
	FrictionWheel_Set(FrictionSpeed);
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

