#include "main.h"

RemoteRec_TypeDef RemoteData;

/**
  * @brief  返回遥控器对象地址
  * @param  rx_buffer18位数据
  * @retval None 
  */ 
RemoteRec_TypeDef* Get_RemoteData(void)
{
	return &RemoteData;
} 

/**
  * @brief  USART1接收遥控器数据
  * @param  rx_buffer18位数据
  * @retval None 
  */ 
void RemoteReceiveTask(volatile unsigned char rx_buffer[])
{
	CanTxMsg tx_message;
	int i;
	
	RemoteData.rc.ch0 = (rx_buffer[0]| (rx_buffer[1] << 8)) & 0x07ff; //!< Channel 0
	RemoteData.rc.ch1 = ((rx_buffer[1] >> 3) | (rx_buffer[2] << 5)) & 0x07ff; //!< Channel 1
	RemoteData.rc.ch2 = ((rx_buffer[2] >> 6) | (rx_buffer[3] << 2) | (rx_buffer[4] << 10)) & 0x07ff;//!< Channel 2
	RemoteData.rc.ch3 = ((rx_buffer[4] >> 1) | (rx_buffer[5] << 7)) & 0x07ff; //!< Channel 3
	RemoteData.rc.s1 = ((rx_buffer[5] >> 4)& 0x0003); //!< Switch left
	RemoteData.rc.s2 = ((rx_buffer[5] >> 6)& 0x0003);

	RemoteData.mouse.x = rx_buffer[6] | (rx_buffer[7] << 8); //!< Mouse X axis
	RemoteData.mouse.y = rx_buffer[8] | (rx_buffer[9] << 8); //!< Mouse Y axis
	RemoteData.mouse.z = rx_buffer[10] | (rx_buffer[11] << 8); //!< Mouse Z axis
	RemoteData.mouse.press_l = rx_buffer[12]; //!< Mouse Left Is Press ?
	RemoteData.mouse.press_r = rx_buffer[13]; //!< Mouse Right Is Press ?

	RemoteData.key.w = rx_buffer[14]&0x01; // KeyBoard value
	RemoteData.key.s = (rx_buffer[14]>>1)&0x01;
	RemoteData.key.a = (rx_buffer[14]>>2)&0x01;
	RemoteData.key.d = (rx_buffer[14]>>3)&0x01;
	RemoteData.key.shift =(rx_buffer[14]>>4)&0x01;
	RemoteData.key.ctrl = (rx_buffer[14]>>5)&0x01;
	RemoteData.key.q = (rx_buffer[14]>>6)&0x01;
	RemoteData.key.e = (rx_buffer[14]>>7)&0x01;	
	RemoteData.key.r = (rx_buffer[15])&0x01;
	RemoteData.key.f = (rx_buffer[15]>>1)&0x01;
	RemoteData.key.g = (rx_buffer[15]>>2)&0x01; 
	RemoteData.key.z = (rx_buffer[15]>>3)&0x01;
	RemoteData.key.x = (rx_buffer[15]>>4)&0x01;
	RemoteData.key.c = (rx_buffer[15]>>5)&0x01;
	RemoteData.key.v = (rx_buffer[15]>>6)&0x01;
	RemoteData.key.b = (rx_buffer[15]>>7)&0x01;
	
	tx_message.IDE = CAN_ID_STD;    
	tx_message.RTR = CAN_RTR_DATA; 
	tx_message.DLC = 0x08;    
	tx_message.StdId = 0x600;
	for(i=0;i<6;i++)
		tx_message.Data[i] = rx_buffer[i];    
	
	CAN_Transmit(CAN1, &tx_message);
}

/**
  * @brief  遥控器数据复位
  * @param  None
  * @retval None
  */
void RC_Init(void)
{
	RemoteData.rc.ch0 = 1024;
	RemoteData.rc.ch1 = 1024;
	RemoteData.rc.ch2 = 1024;
	RemoteData.rc.ch3 = 1024;
	RemoteData.mouse.x = 0;
	RemoteData.mouse.y = 0;
	RemoteData.mouse.z = 0;
	RemoteData.mouse.press_l = 0;                                                
	RemoteData.mouse.press_r = 0;

	RemoteData.key.w = 0;
	RemoteData.key.s = 0;                            
	RemoteData.key.a = 0;
	RemoteData.key.d = 0;
	RemoteData.key.q = 0;
	RemoteData.key.e = 0;
	RemoteData.key.r = 0;
	RemoteData.key.f = 0;
	RemoteData.key.shift = 0;
	RemoteData.key.ctrl = 0;

	RemoteData.rc.s1=3;
	RemoteData.rc.s2=3;
} 



