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
	FrictionWheel_Configuration();//��
	LASER_Configuration();     //��
	//SolenoidSW_Configuration();//��
	LED_Configuration();       //��
	CAN_Configuration();       //��
	USART1_Configuration();    //��
	USART2_Configuration();	   //��
	USART3_Configuration();    //��
	
	TIM2Init();
	SPI1_Init();	
	ADIS_Init();
	EXTIX_Init();
	delay_ms(1000);
}

void system_Init(void)
{
	SysTick_Config(72000);		//1ms��ʱ�ж�
	FrictionSpeed = 84;   //��ʼ�ر�Ħ����
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
		GPIO_SetBits(GPIOA, GPIO_Pin_5);    //�ر�PC����������ʾ�ƣ�PA5�����
	}
	
//	//TX2���ڶ��߼��
//	is_tx2_connect_cnt ++;
//	if(is_tx2_connect_cnt >= 100)   //100ms
//	{
//		is_tx2_connect_cnt = 0;
//		DMA_Cmd(DMA1_Channel5, DISABLE);
//		USART1_Configuration();
//	}
//	
//	//CAN���߼��
//	is_can_disconnect_cnt ++;
//	if(is_can_disconnect_cnt >= 10) //10ms
//	{
//		is_can_disconnect_cnt  = 0;
//		CAN_Configuration();
//	}
//	
//	//�����Ƕ��߼��
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

