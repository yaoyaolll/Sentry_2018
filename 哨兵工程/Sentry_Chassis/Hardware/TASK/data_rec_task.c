#include "main.h"

pc_t pc_gimbal; 
judge_t judge;
unsigned char save_buffer[68]; 
unsigned char pc_chassis_rec_data[8];
unsigned char pc_chassis_rec_flag;
unsigned char is_pc_chassis_connect;
unsigned char wifi_choose_flag = 4;
unsigned char wifi_rec_flag;

//遥控器参数初始化
remote_t remote = {
	.rc.ch0 = 1024,
	.rc.ch1 = 1024,
	.rc.ch2 = 1024,
	.rc.ch3 = 1024,
	.mouse.x = 0,
	.mouse.y = 0,
	.mouse.z = 0,
	.mouse.press_l = 0,                                                
	.mouse.press_r = 0,

	.key.w = 0,
	.key.s = 0,                            
	.key.a = 0,
	.key.d = 0,
	.key.q = 0,
	.key.e = 0,
	.key.r = 0,
	.key.f = 0,
	.key.shift = 0,
	.key.ctrl = 0,

	.rc.s1=3,
	.rc.s2=3
}; 


/*------------------遥控器---------------*/
/**
  * @brief  返回遥控器对象地址
  * @param  rx_buffer18位数据
  * @retval None 
  */ 
remote_t* get_remote(void)
{
	return &remote; 
} 

/**
  * @brief  遥控器数据从云台板发来
  * @param  rx_buffer18位数据
  * @retval None 
  */ 
void remote_rec_task(volatile unsigned char rx_buffer[])
{
	remote.rc.ch0 = (rx_buffer[0]| (rx_buffer[1] << 8)) & 0x07ff; //!< Channel 0
	remote.rc.ch1 = ((rx_buffer[1] >> 3) | (rx_buffer[2] << 5)) & 0x07ff; //!< Channel 1
	remote.rc.ch2 = ((rx_buffer[2] >> 6) | (rx_buffer[3] << 2) | (rx_buffer[4] << 10)) & 0x07ff;//!< Channel 2
	remote.rc.ch3 = ((rx_buffer[4] >> 1) | (rx_buffer[5] << 7)) & 0x07ff; //!< Channel 3
	remote.rc.s1 = ((rx_buffer[5] >> 4)& 0x0003); //!< Switch left
	remote.rc.s2 = ((rx_buffer[5] >> 6)& 0x0003);

	remote.mouse.x = rx_buffer[6] | (rx_buffer[7] << 8); //!< Mouse X axis
	remote.mouse.y = rx_buffer[8] | (rx_buffer[9] << 8); //!< Mouse Y axis
	remote.mouse.z = rx_buffer[10] | (rx_buffer[11] << 8); //!< Mouse Z axis
	remote.mouse.press_l = rx_buffer[12]; //!< Mouse Left Is Press ?
	remote.mouse.press_r = rx_buffer[13]; //!< Mouse Right Is Press ?

	remote.key.w = rx_buffer[14]&0x01; // KeyBoard value
	remote.key.s = (rx_buffer[14]>>1)&0x01;
	remote.key.a = (rx_buffer[14]>>2)&0x01;
	remote.key.d = (rx_buffer[14]>>3)&0x01;
	remote.key.shift =(rx_buffer[14]>>4)&0x01;
	remote.key.ctrl = (rx_buffer[14]>>5)&0x01;
	remote.key.q = (rx_buffer[14]>>6)&0x01;
	remote.key.e = (rx_buffer[14]>>7)&0x01;	
	remote.key.r = (rx_buffer[15])&0x01;
	remote.key.f = (rx_buffer[15]>>1)&0x01;
	remote.key.g = (rx_buffer[15]>>2)&0x01; 
	remote.key.z = (rx_buffer[15]>>3)&0x01;
	remote.key.c = (rx_buffer[15]>>5)&0x01;
	remote.key.v = (rx_buffer[15]>>6)&0x01;
	remote.key.b = (rx_buffer[15]>>7)&0x01;
}

/*------------------PC---------------*/
/**
  * @brief  返回pc_gimbal结构体地址值
  * @param  
  * @retval 
  */
pc_t* get_pc_gimbal(void)
{
	return &pc_gimbal;
}

/**
  * @brief  MCU接收云台PC数据
  * @param  *str PC发送数据数组
  * @retval 
  */
void pc_gimbal2mcu_rec_task(unsigned char *str)
{
	LED_PC_ON;                   //开启PC连接指示灯 
	
	if(str[0] == 0x20)
	{
		pc_gimbal.source_address = str[0]&0x0f;
		pc_gimbal.destin_address = (str[0]&0xf0)>>4;
		pc_gimbal.chasiss_act_mode = str[1]&0x0f;
		pc_gimbal.gimbal_act_mode = (str[1]&0xf0)>>4;
		pc_gimbal.chassis_move_pos = str[2]&0x0f;
		pc_gimbal.gimbal_shoot_mode = (str[2]&0xf0)>>4;
		pc_gimbal.relative_pitch_degree = ((short)(str[3]<<8|str[4]))/100.0f;
		pc_gimbal.relative_yaw_degree = ((short)(str[5]<<8|str[6]))/100.0f;
			
		pc_gimbal.pc_receive_flag = 1;
	}
//	//未识别到会一直发送0，卡尔曼滤波处理
}

/**
  * @brief  底盘PC接收到数据直接转发给云台PC
  * @param  *str PC发送数据数组
  * @retval 
  */
extern unsigned char pc_disconnect_cnt;
void pc_chassis2pc_gimbal_rec_task(unsigned char *str)
{
	int i;
	
	if(str[0] == '!' && Verify_CRC16_Check_Sum(str, 11) == 1)    //第0位以及第9、10位为校验位
	{
		is_pc_chassis_connect = 0;
		pc_disconnect_cnt = 0;
		pc_chassis_rec_flag = 1;                                   //收到底盘PC发送数据并通过校验
		for(i = 0; i < 8; i ++)
			pc_chassis_rec_data[i] = str[i + 1];
	}
}

/*------------------WIFI---------------*/
void wifi_rec_task(unsigned char *str)
{
	wifi_rec_flag = 1;
	wifi_choose_flag = str[2];
}


/*------------------裁判系统---------------*/
/**
  * @brief  返回裁判系统结构体地址
  * @param  None
  * @retval None
  */
judge_t* get_judge(void)
{
	return &judge;
}

/**
  * @brief  USART1接收裁判系统数据，数据帧5+2+n+2
	* cmdid 1:表示状态，剩余血量，当前等级,数据位长度为25
	*				3：弹丸射速，数据位长度为10Bytes;
	*       4:底盘电流电压，功率，功率缓冲 数据位长度为20Bytes
	*				10：自定义数据，数据位13Bytes大小。
  * @param  ReceiveBuffer[] DMA数据接收
  * @retval None 
  */ 
uint16_t cmd_id;
uint16_t seeDataLen;
void judge_rec_task(unsigned char ReceiveBuffer[])
{
	uint16_t DataLen;
	short PackPoint = 0;
	memcpy(&save_buffer[JudgeBufBiggestSize],&ReceiveBuffer[0],JudgeBufBiggestSize); 
	for(PackPoint = 0; PackPoint < JudgeBufBiggestSize; PackPoint ++)
	{
		if(save_buffer[PackPoint]==0xA5)
		{	
			if((Verify_CRC8_Check_Sum(&save_buffer[PackPoint],5)==1))
			{
				cmd_id=(save_buffer[PackPoint+6])&0xff;
				cmd_id=(cmd_id<<8)|save_buffer[PackPoint+5];
				
				DataLen=save_buffer[PackPoint+2]&0xff;
				DataLen=(DataLen<<8)|save_buffer[PackPoint+1];
			  seeDataLen = DataLen;
				if(cmd_id == 0x0001&&(Verify_CRC16_Check_Sum(&save_buffer[PackPoint],DataLen+9))) 
				{
					judge.stage_remain_time = save_buffer[PackPoint+8]<<8|save_buffer[PackPoint+7];
					judge.remain_HP=(save_buffer[PackPoint+12]<<8)|save_buffer[PackPoint+11];
				}
				if(cmd_id == 0x0002&&(Verify_CRC16_Check_Sum(&save_buffer[PackPoint],DataLen+9)))
				{
					judge.hurt_type = (save_buffer[PackPoint+7] & 0xf0) >> 4;  //4~7bits
					judge.armor_type = save_buffer[PackPoint+7] & 0x0f;        //0~3bits
				}
				if(cmd_id == 0x0003&&(Verify_CRC16_Check_Sum(&save_buffer[PackPoint],DataLen+9)))
				{
					judge.bullet_freq=save_buffer[PackPoint+8];
					memcpy(&judge.bullet_speed, &save_buffer[PackPoint+9], 4);
				}
				if(cmd_id==0x0004&&(Verify_CRC16_Check_Sum(&save_buffer[PackPoint],DataLen+9)))
				{
					judge.shooter_heat17mm = save_buffer[PackPoint+24]<<8|save_buffer[PackPoint+23];
				}
		  }
		}
	}
	memcpy(&save_buffer[0],&save_buffer[JudgeBufBiggestSize],JudgeBufBiggestSize);
}
