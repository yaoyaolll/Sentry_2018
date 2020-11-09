#include "main.h"
#include "string.h"
//CAN_TX---PA12
//CAN_RX---PA11
extern unsigned char PC_TXBuffer[20];
extern short FrictionSpeed;
_820r_motor_t motor_bodan;
_820r_motor_t chassis_motor1;
_820r_motor_t chassis_motor2;
_6623_motor_t motor_pitch;
_6623_motor_t motor_yaw;

void CAN_Configuration(void)
{
	CAN_InitTypeDef        can;
	CAN_FilterInitTypeDef  can_filter;
	GPIO_InitTypeDef 	   	 gpio;
	NVIC_InitTypeDef   	   nvic;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1,  ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA |
												 RCC_APB2Periph_AFIO , ENABLE);
	
	gpio.GPIO_Pin = GPIO_Pin_11;
	gpio.GPIO_Mode = GPIO_Mode_IPU;
	gpio.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &gpio);	//CAN_RX

	gpio.GPIO_Pin = GPIO_Pin_12;	   
	gpio.GPIO_Mode = GPIO_Mode_AF_PP;
	gpio.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &gpio);    //CAN_TX

	nvic.NVIC_IRQChannel = USB_HP_CAN1_TX_IRQn;
	nvic.NVIC_IRQChannelPreemptionPriority = 1;
	nvic.NVIC_IRQChannelSubPriority = 0;
	nvic.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&nvic);
	
	nvic.NVIC_IRQChannel = USB_LP_CAN1_RX0_IRQn;  
	nvic.NVIC_IRQChannelPreemptionPriority = 1;
	nvic.NVIC_IRQChannelSubPriority = 1;
	nvic.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&nvic);

	CAN_DeInit(CAN1);

	can.CAN_TTCM = DISABLE;
	can.CAN_ABOM = DISABLE;
	can.CAN_AWUM = DISABLE;
	can.CAN_NART = DISABLE;	  
	can.CAN_RFLM = DISABLE;																
	can.CAN_TXFP = ENABLE;		
	can.CAN_Mode = CAN_Mode_Normal;
	//can.CAN_Mode = CAN_Mode_LoopBack;
	can.CAN_SJW = CAN_SJW_1tq;
	can.CAN_BS1 = CAN_BS1_5tq;
	can.CAN_BS2 = CAN_BS2_3tq;
	can.CAN_Prescaler = 4;     //CAN BaudRate 36/(1+5+3)/4=1Mbps
	CAN_Init(CAN1, &can);

	can_filter.CAN_FilterNumber = 0; 
	can_filter.CAN_FilterMode = CAN_FilterMode_IdMask;
	can_filter.CAN_FilterScale = CAN_FilterScale_32bit;
	can_filter.CAN_FilterIdHigh = 0;
	can_filter.CAN_FilterIdLow = 0;
	can_filter.CAN_FilterMaskIdHigh = 0;
	can_filter.CAN_FilterMaskIdLow = 0;
	can_filter.CAN_FilterFIFOAssignment = 0;
	can_filter.CAN_FilterActivation = ENABLE;
	CAN_FilterInit(&can_filter);
	
	CAN_ITConfig(CAN1,CAN_IT_FMP0,ENABLE);
	CAN_ITConfig(CAN1,CAN_IT_TME,ENABLE);
}

void USB_LP_CAN1_RX0_IRQHandler(void)
{
	CanRxMsg rx_message;
	if (CAN_GetITStatus(CAN1,CAN_IT_FMP0)!= RESET) 
	{
		CAN_ClearITPendingBit(CAN1,CAN_IT_FMP0);
		CAN_Receive(CAN1, CAN_FIFO0, &rx_message);
		
		if(rx_message.StdId == 0x701)    //摩擦轮开启或者关闭
		{
			FrictionSpeed = (short)rx_message.Data[0]<<8|rx_message.Data[1];
		}
		if(rx_message.StdId == 0x702 | rx_message.StdId == 0x706)
		{
			pc_chassis2pc_gimbal_send_task(rx_message.Data); 
		}
		if(rx_message.StdId == 0x203)
		{
			motor_bodan.angle = rx_message.Data[0]<<8 | rx_message.Data[1];
			motor_bodan.real_speed = rx_message.Data[2]<<8 | rx_message.Data[3];
		}
		if(rx_message.StdId == 0x205)
		{
			motor_yaw.angle=rx_message.Data[0]<<8 | rx_message.Data[1];
		  motor_yaw.real_flow=rx_message.Data[2]<<8 | rx_message.Data[3];
		}
		if(rx_message.StdId == 0x206)
		{
			motor_pitch.angle=rx_message.Data[0]<<8 | rx_message.Data[1];
		  motor_pitch.real_flow=rx_message.Data[2]<<8 | rx_message.Data[3];                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   	  motor_pitch.set_flow=rx_message.Data[4]<<8 | rx_message.Data[5];
		}
		if(rx_message.StdId == 0x201)
		{
			chassis_motor1.angle=rx_message.Data[0]<<8 | rx_message.Data[1];
		  chassis_motor1.real_speed=rx_message.Data[2]<<8 | rx_message.Data[3]; 
		}
		if(rx_message.StdId == 0x202)
		{
			chassis_motor2.angle=rx_message.Data[0]<<8 | rx_message.Data[1];
		  chassis_motor2.real_speed=rx_message.Data[2]<<8 | rx_message.Data[3];
		}
	}
}

void USB_HP_CAN1_TX_IRQHandler(void)
{
	if (CAN_GetITStatus(CAN1,CAN_IT_TME)!= RESET) 
	{
			CAN_ClearITPendingBit(CAN1,CAN_IT_TME);
	}
}
