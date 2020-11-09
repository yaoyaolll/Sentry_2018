#include "main.h"
#include "string.h"

//CAN_TX---PB9
//CAN_RX---PB8

float PITCH;
float GY;
float GZ;
unsigned char is_gyro_connect;

void CAN_Configuration(void)
{
	CAN_InitTypeDef        can;
	CAN_FilterInitTypeDef  can_filter;
	GPIO_InitTypeDef 	   	 gpio;
	NVIC_InitTypeDef   	   nvic;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1,  ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB |
												 RCC_APB2Periph_AFIO , ENABLE);
	
	GPIO_PinRemapConfig(GPIO_Remap1_CAN1,ENABLE);

	gpio.GPIO_Pin = GPIO_Pin_8;
	gpio.GPIO_Mode = GPIO_Mode_IPU;
	gpio.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &gpio);	//CAN_RX

	gpio.GPIO_Pin = GPIO_Pin_9;	   
	gpio.GPIO_Mode = GPIO_Mode_AF_PP;
	gpio.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &gpio);    //CAN_TX

	nvic.NVIC_IRQChannel = USB_HP_CAN1_TX_IRQn;
	nvic.NVIC_IRQChannelPreemptionPriority = 0;
	nvic.NVIC_IRQChannelSubPriority = 0;
	nvic.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&nvic);
	
	nvic.NVIC_IRQChannel = USB_LP_CAN1_RX0_IRQn;  
	nvic.NVIC_IRQChannelPreemptionPriority = 0;
	nvic.NVIC_IRQChannelSubPriority = 1;
	nvic.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&nvic);

	CAN_DeInit(CAN1);

	can.CAN_TTCM = DISABLE;
	can.CAN_ABOM = ENABLE;
	can.CAN_AWUM = DISABLE;
	can.CAN_NART = ENABLE;	  
	can.CAN_RFLM = DISABLE;																
	can.CAN_TXFP = ENABLE;		
	can.CAN_Mode = CAN_Mode_Normal;
//		can.CAN_Mode = CAN_Mode_LoopBack;
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
	_820r_motor_t *motor_bodan = get_motor_bodan();
	_820r_motor_t *motor_chassis = get_motor_chassis();
	_6623_motor_t *motor_pitch = get_motor_pitch(); 
	_6623_motor_t *motor_yaw = get_motor_yaw();
	
	if (CAN_GetITStatus(CAN1,CAN_IT_FMP0)!= RESET) 
	{
		CAN_Receive(CAN1, CAN_FIFO0, &rx_message);
		
		if(rx_message.StdId == motor_chassis_id1)  //0x201
		{
			(*motor_chassis).angle = rx_message.Data[0]<<8 | rx_message.Data[1];
			(*motor_chassis).real_speed = rx_message.Data[2]<<8 | rx_message.Data[3];
		}
		if(rx_message.StdId == motor_chassis_id2)  //0x202
		{
 			(*(motor_chassis+1)).angle = rx_message.Data[0]<<8 | rx_message.Data[1];
			(*(motor_chassis+1)).real_speed = rx_message.Data[2]<<8 | rx_message.Data[3];
		}
		if(rx_message.StdId == motor_bodan_id)     //0x203
		{
			(*motor_bodan).angle = rx_message.Data[0]<<8 | rx_message.Data[1];
			(*motor_bodan).real_speed = rx_message.Data[2]<<8 | rx_message.Data[3];
		}
		if(rx_message.StdId == motor_pitch_id) //0x206
		{
		  (*motor_pitch).angle=rx_message.Data[0]<<8 | rx_message.Data[1];
		  (*motor_pitch).real_flow=rx_message.Data[2]<<8 | rx_message.Data[3];
		  (*motor_pitch).set_flow=rx_message.Data[4]<<8 | rx_message.Data[5];
		}
		if(rx_message.StdId == motor_yaw_id)  //0x205
		{
			(*motor_yaw).angle=rx_message.Data[0]<<8 | rx_message.Data[1];
		  (*motor_yaw).real_flow=rx_message.Data[2]<<8 | rx_message.Data[3];
		  (*motor_yaw).set_flow=rx_message.Data[4]<<8 | rx_message.Data[5];
		}
		if(rx_message.StdId == pc_gimbal2mcu_id)    //0x703 
		{
			if(rx_message.Data[0] == 0x20)
				pc_gimbal2mcu_rec_task(rx_message.Data); 
		}
		if(rx_message.StdId == gyro_id)        //0x704
		{
			short b1,b2,b3;
			
			LED_GYRO_ON;             //开启陀螺仪连接指示灯   蓝灯亮起说明陀螺仪连接，此时方可由断电模式切换到用户或自动模式
			is_gyro_connect = 0;     //陀螺仪断线计数
			
			b1 = (short)(rx_message.Data[0]<<8 | rx_message.Data[1]);
			b2 = (short)(rx_message.Data[2]<<8 | rx_message.Data[3]);
			b3 = (short)(rx_message.Data[4]<<8 | rx_message.Data[5]);
			PITCH = b1/100.0f;
			//过滤掉上限值
			if(ABS(b2) <= 700)
				GY = b2/100.0f;		
			if(ABS(b3) <= 700)   
				GZ = b3/100.0f;
		}
		if(rx_message.StdId == remote_id)      //0x705
		{
			remote_t* remote = get_remote();

			remote->rc.ch0 = (rx_message.Data[0]| (rx_message.Data[1] << 8)) & 0x07ff; //!< Channel 0
			remote->rc.ch1 = ((rx_message.Data[1] >> 3) | (rx_message.Data[2] << 5)) & 0x07ff; //!< Channel 1
			remote->rc.ch2 = ((rx_message.Data[2] >> 6) | (rx_message.Data[3] << 2) | (rx_message.Data[4] << 10)) & 0x07ff;//!< Channel 2
			remote->rc.ch3 = ((rx_message.Data[4] >> 1) | (rx_message.Data[5] << 7)) & 0x07ff; //!< Channel 3
			remote->rc.s1 = ((rx_message.Data[5] >> 4)& 0x0003); //!< Switch left
			remote->rc.s2 = ((rx_message.Data[5] >> 6)& 0x0003);
		}
		CAN_ClearITPendingBit(CAN1, CAN_IT_FMP0);
	}
}

void USB_HP_CAN1_TX_IRQHandler(void)
{
	if (CAN_GetITStatus(CAN1,CAN_IT_TME)!= RESET) 
	{
			CAN_ClearITPendingBit(CAN1,CAN_IT_TME);
	}
}


//#include "main.h"
//#include "string.h"

////CAN_TX---PB12
////CAN_RX---PB11

//float PITCH;
//float GY;
//float GZ;
//unsigned char is_gyro_connect;

//void CAN_Configuration(void)
//{
//	CAN_InitTypeDef        can;
//	CAN_FilterInitTypeDef  can_filter;
//	GPIO_InitTypeDef 	   	 gpio;
//	NVIC_InitTypeDef   	   nvic;

//	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1,  ENABLE);
//	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA |
//												 RCC_APB2Periph_AFIO , ENABLE);
//	
//	gpio.GPIO_Pin = GPIO_Pin_11;
//	gpio.GPIO_Mode = GPIO_Mode_IPU;
//	gpio.GPIO_Speed = GPIO_Speed_50MHz;
//	GPIO_Init(GPIOA, &gpio);	//CAN_RX

//	gpio.GPIO_Pin = GPIO_Pin_12;	   
//	gpio.GPIO_Mode = GPIO_Mode_AF_PP;
//	gpio.GPIO_Speed = GPIO_Speed_50MHz;
//	GPIO_Init(GPIOA, &gpio);    //CAN_TX

//	nvic.NVIC_IRQChannel = USB_HP_CAN1_TX_IRQn;
//	nvic.NVIC_IRQChannelPreemptionPriority = 0;
//	nvic.NVIC_IRQChannelSubPriority = 0;
//	nvic.NVIC_IRQChannelCmd = ENABLE;
//	NVIC_Init(&nvic);
//	
//	nvic.NVIC_IRQChannel = USB_LP_CAN1_RX0_IRQn;  
//	nvic.NVIC_IRQChannelPreemptionPriority = 0;
//	nvic.NVIC_IRQChannelSubPriority = 1;
//	nvic.NVIC_IRQChannelCmd = ENABLE;
//	NVIC_Init(&nvic);

//	CAN_DeInit(CAN1);

//	can.CAN_TTCM = DISABLE;
//	can.CAN_ABOM = ENABLE;
//	can.CAN_AWUM = DISABLE;
//	can.CAN_NART = ENABLE;	  
//	can.CAN_RFLM = DISABLE;																
//	can.CAN_TXFP = ENABLE;		
//	can.CAN_Mode = CAN_Mode_Normal;
////		can.CAN_Mode = CAN_Mode_LoopBack;
//	can.CAN_SJW = CAN_SJW_1tq;
//	can.CAN_BS1 = CAN_BS1_5tq;
//	can.CAN_BS2 = CAN_BS2_3tq;
//	can.CAN_Prescaler = 4;     //CAN BaudRate 36/(1+5+3)/4=1Mbps
//	CAN_Init(CAN1, &can);

//	can_filter.CAN_FilterNumber = 0; 
//	can_filter.CAN_FilterMode = CAN_FilterMode_IdMask;
//	can_filter.CAN_FilterScale = CAN_FilterScale_32bit;
//	can_filter.CAN_FilterIdHigh = 0;
//	can_filter.CAN_FilterIdLow = 0;
//	can_filter.CAN_FilterMaskIdHigh = 0;
//	can_filter.CAN_FilterMaskIdLow = 0;
//	can_filter.CAN_FilterFIFOAssignment = 0;
//	can_filter.CAN_FilterActivation = ENABLE;
//	CAN_FilterInit(&can_filter);
//	
//	CAN_ITConfig(CAN1,CAN_IT_FMP0,ENABLE);
//	CAN_ITConfig(CAN1,CAN_IT_TME,ENABLE);
//}

//int last_gyro_time;
//int cur_gyro_time;
//int gyro_time_interval;
//int last_pitch_can_time;
//int cur_pitch_can_time;
//int pitch_can_interval;
//void USB_LP_CAN1_RX0_IRQHandler(void)
//{
//	CanRxMsg rx_message;
//	_820r_motor_t *motor_bodan = get_motor_bodan();
//	_820r_motor_t *motor_chassis = get_motor_chassis();
//	_6623_motor_t *motor_pitch = get_motor_pitch(); 
//	_6623_motor_t *motor_yaw = get_motor_yaw();
//	
//	if (CAN_GetITStatus(CAN1,CAN_IT_FMP0)!= RESET) 
//	{
//		CAN_Receive(CAN1, CAN_FIFO0, &rx_message);
//		
//		if(rx_message.StdId == motor_chassis_id1)  //0x201
//		{
//			(*motor_chassis).angle = rx_message.Data[0]<<8 | rx_message.Data[1];
//			(*motor_chassis).real_speed = rx_message.Data[2]<<8 | rx_message.Data[3];
//		}
//		if(rx_message.StdId == motor_chassis_id2)  //0x202
//		{
// 			(*(motor_chassis+1)).angle = rx_message.Data[0]<<8 | rx_message.Data[1];
//			(*(motor_chassis+1)).real_speed = rx_message.Data[2]<<8 | rx_message.Data[3];
//		}
//		if(rx_message.StdId == motor_bodan_id)     //0x203
//		{
//			(*motor_bodan).angle = rx_message.Data[0]<<8 | rx_message.Data[1];
//			(*motor_bodan).real_speed = rx_message.Data[2]<<8 | rx_message.Data[3];
//		}
//		if(rx_message.StdId == 0x205) //0x205
//		{
//		  (*motor_pitch).angle=rx_message.Data[0]<<8 | rx_message.Data[1];
//		  (*motor_pitch).real_flow=rx_message.Data[2]<<8 | rx_message.Data[3];
//		  (*motor_pitch).set_flow=rx_message.Data[4]<<8 | rx_message.Data[5];
////			cur_pitch_can_time = get_sys_cnt();
////			pitch_can_interval = cur_pitch_can_time - last_pitch_can_time;	
////			last_pitch_can_time = cur_pitch_can_time;
//		}
//		if(rx_message.StdId == 0x206)  //0x206
//		{
//			(*motor_yaw).angle=rx_message.Data[0]<<8 | rx_message.Data[1];
//		  (*motor_yaw).real_flow=rx_message.Data[2]<<8 | rx_message.Data[3];
//		  (*motor_yaw).set_flow=rx_message.Data[4]<<8 | rx_message.Data[5];
//		}
//		if(rx_message.StdId == pc_gimbal2mcu_id)    //0x703 
//		{
//			pc_gimbal2mcu_rec_task(rx_message.Data); 
//		}
//		if(rx_message.StdId == gyro_id)        //0x704
//		{
//			short b1,b2,b3;
//			
//			LED_GYRO_ON;             //开启陀螺仪连接指示灯   蓝灯亮起说明陀螺仪连接，此时方可由断电模式切换到用户或自动模式
//			is_gyro_connect = 0;     //陀螺仪断线计数
//			
//			b1 = (short)(rx_message.Data[0]<<8 | rx_message.Data[1]);
//			b2 = (short)(rx_message.Data[2]<<8 | rx_message.Data[3]);
//			b3 = (short)(rx_message.Data[4]<<8 | rx_message.Data[5]);
//			PITCH = b1/100.0f;
//			//过滤掉上限值
//			if(ABS(b2) <= 700)
//				GY = b2/100.0f;		
//			if(ABS(b3) <= 700)   
//				GZ = b3/100.0f;
//			
//			cur_gyro_time = get_sys_cnt();
//			gyro_time_interval = cur_gyro_time - last_gyro_time;	
//			last_gyro_time = cur_gyro_time;
//		}
//		if(rx_message.StdId == remote_id)      //0x705
//		{
//			remote_t* remote = get_remote();

//			remote->rc.ch0 = (rx_message.Data[0]| (rx_message.Data[1] << 8)) & 0x07ff; //!< Channel 0
//			remote->rc.ch1 = ((rx_message.Data[1] >> 3) | (rx_message.Data[2] << 5)) & 0x07ff; //!< Channel 1
//			remote->rc.ch2 = ((rx_message.Data[2] >> 6) | (rx_message.Data[3] << 2) | (rx_message.Data[4] << 10)) & 0x07ff;//!< Channel 2
//			remote->rc.ch3 = ((rx_message.Data[4] >> 1) | (rx_message.Data[5] << 7)) & 0x07ff; //!< Channel 3
//			remote->rc.s1 = ((rx_message.Data[5] >> 4)& 0x0003); //!< Switch left
//			remote->rc.s2 = ((rx_message.Data[5] >> 6)& 0x0003);
//		}
//		CAN_ClearITPendingBit(CAN1, CAN_IT_FMP0);
//	}
//}

//void USB_HP_CAN1_TX_IRQHandler(void)
//{
//	if (CAN_GetITStatus(CAN1,CAN_IT_TME)!= RESET) 
//	{
//			CAN_ClearITPendingBit(CAN1,CAN_IT_TME);
//	}
//}






