#include "main.h"

/*-----------------底盘电机以及拨弹电机-------------*/
/**
  * @brief  发送各个电机电流值
  * @param  None
  * @retval None
  */
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


/*-----------------云台电机-------------*/
/**
  * @brief  发送云台电机电流值
  * @param  None
  * @retval None
  */
	CanTxMsg see_tx_message;
void gimbal_send_task(int yaw, int pitch)  
{
	
	see_tx_message.IDE = CAN_ID_STD;    
	see_tx_message.RTR = CAN_RTR_DATA; 
	see_tx_message.DLC = 0x08;    
	see_tx_message.StdId = 0x210;

	see_tx_message.Data[0] = (unsigned char)((pitch>>8)&0xff);
	see_tx_message.Data[1] = (unsigned char)(pitch&0xff);  	
	see_tx_message.Data[2] = (unsigned char)((yaw>>8)&0xff);
	see_tx_message.Data[3] = (unsigned char)(yaw&0xff);  	

	CAN_Transmit(CAN1, &see_tx_message);
}

/*-----------------摩擦轮-------------*/
/**
  * @brief  摩擦轮开启或者关闭
  * @param  None
  * @retval None
  */
void friction_send_task(short speed)  
{
	CanTxMsg tx_message;
	
	tx_message.IDE = CAN_ID_STD;    
	tx_message.RTR = CAN_RTR_DATA; 
	tx_message.DLC = 0x08;    
	tx_message.StdId = friction_wheel_id;  
	tx_message.Data[0] = speed>>8 & 0xff;
	tx_message.Data[1] = speed & 0xff;
	
	CAN_Transmit(CAN1, &tx_message);
} 

/*----------------PC_CHASSIS_TO_PC_GIMBAL-----------*/
extern unsigned char pc_chassis_rec_data[8];
extern unsigned char pc_chassis_rec_flag;
extern unsigned char pc_disconnect_cnt;     //底盘PC断线计数
/**
  * @brief  底盘PC发送数据至云台PC
  * @param  None
  * @retval None
  */
void pc_chassis2pc_gimbal_send_task(void)
{
	int index;
	CanTxMsg tx_message;
	
	tx_message.IDE = CAN_ID_STD;    
	tx_message.RTR = CAN_RTR_DATA; 
	tx_message.DLC = 0x08;    
	tx_message.StdId = pc_chassis2pc_gimbal_id; 
	
	if(pc_chassis_rec_flag == 1 && pc_disconnect_cnt == 0)
	{ 
		for(index = 0;index < 8;index ++)
			tx_message.Data[index] = pc_chassis_rec_data[index];
		
		CAN_Transmit(CAN1, &tx_message);
		pc_chassis_rec_flag = 0;
	}
	else if(pc_disconnect_cnt == 6)     //底盘断线
	{
		for(index = 0;index < 8;index ++)
			tx_message.Data[index] = 0;
		
		CAN_Transmit(CAN1, &tx_message);
		pc_chassis_rec_flag = 0;
	}
}


/*----------------MCU_TO_PC_GIMBAL------------*/
void mcu2pc_gimbal_send_task(void) 
{
	CanTxMsg tx_message;
	
	tx_message.IDE = CAN_ID_STD;    
	tx_message.RTR = CAN_RTR_DATA; 
	tx_message.DLC = 0x08;    
	tx_message.StdId = mcu2pc_gimbal_id;
		
	tx_message.Data[0] = 0x02;                //源地址为下位机，目的地址为云台PC
	tx_message.Data[1] = 0x01;
	tx_message.Data[2] = 0x12;
	tx_message.Data[3] = ((short)(PITCH*100))<<8;
	tx_message.Data[4] = ((short)(PITCH*100))&0xff;
	tx_message.Data[5] = 0;
	tx_message.Data[6] = 0;
	tx_message.Data[7] = 0x21;
	
	CAN_Transmit(CAN1, &tx_message);
}

