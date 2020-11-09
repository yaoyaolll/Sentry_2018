#include "main.h"

ChassisMotor_TypeDef ChassisMotor[2];
Pid_Typedef speedPIDOfChassisMotor[2];

/**
  * @brief  发送底盘电机电流值
  * @param  None
  * @retval None
  */
void Chassis_SendData(short *Data)  //0x201,0x202
{
	CanTxMsg tx_message;
	
	tx_message.IDE = CAN_ID_STD;    
	tx_message.RTR = CAN_RTR_DATA; 
	tx_message.DLC = 0x08;    
	tx_message.StdId = 0x200;

	tx_message.Data[0] = (unsigned char)(((*Data)>>8)&0xff);
	tx_message.Data[1] = (unsigned char)((*Data)&0xff);  
	tx_message.Data[2] = (unsigned char)(((*(Data+1))>>8)&0xff);
	tx_message.Data[3] = (unsigned char)((*(Data+1))&0xff);  

	CAN_Transmit(CAN1,&tx_message);
}

/**
  * @brief  返回ChassisMotor对象地址
  * @param  None
  * @retval None
  */
ChassisMotor_TypeDef* Get_ChassisMotor(void)
{
	return ChassisMotor;
}

/**
  * @brief  底盘电机速度PID初始化
  * @param  None
  * @retval None
  */
void speedPIDOfChassisMotor_Init(void)
{
	int i;
	for(i = 0; i < 2; i ++)
	{
		speedPIDOfChassisMotor[i].P = 0.05f;
		speedPIDOfChassisMotor[i].I = 0;
		speedPIDOfChassisMotor[i].D = 2.5;
		speedPIDOfChassisMotor[i].IMax = 0;
	}
}

/**
  * @brief  底盘控制
  * @param  None
  * @retval None
  */
short speedOfChassisMotor;
short currentOfChassisMotor[2];
short* ChassisTask(short speed) 
{
	int i;
	RemoteRec_TypeDef *RemoteData = Get_RemoteData();
	
	speedOfChassisMotor = (1024 - (*RemoteData).rc.ch1)*10;
	speedPIDOfChassisMotor[0].SetPoint = speedOfChassisMotor;
	speedPIDOfChassisMotor[1].SetPoint = -speedOfChassisMotor;
	
//	speedPIDOfChassisMotor[0].SetPoint = speed;//speedOfChassisMotor*30;
//	speedPIDOfChassisMotor[1].SetPoint = -speed;//-speedOfChassisMotor*30;
	for(i = 0; i < 2; i ++)
	{
		currentOfChassisMotor[i] += PID_Calc(&speedPIDOfChassisMotor[i], ChassisMotor[i].RealSpeed);
		currentOfChassisMotor[i] = LIMIT_MAX_MIN(currentOfChassisMotor[i], 4800, -4800);
	}
	return currentOfChassisMotor;
}
