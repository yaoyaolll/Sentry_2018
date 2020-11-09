#include "main.h"

BodanMotor_TypeDef BodanMotorData;        //拨弹电机结构体
Pid_Typedef speedPIDOfBodanMotor;         //拨弹电机速度环PID结构体
Pid_Typedef positionPIDOfBodanMotor;      //拨弹电机位置环结构体
int relativePositionOfBodanMotor;         //拨弹电机相对位置
short lastPositionOfBodanMotor;           //拨弹电机上一次位置
short currentPositionOfBodanMotor;        //拨弹电机当前位置

/**
  * @brief  发送拨弹电机电调的电流值
  * @param  None
  * @retval None
  */
void DriverPlate_SendData(short Data)     //0x203
{
	CanTxMsg tx_message;
	
	tx_message.IDE = CAN_ID_STD;    
	tx_message.RTR = CAN_RTR_DATA; 
	tx_message.DLC = 0x08;    
	tx_message.StdId = 0x200;

	tx_message.Data[4] = (unsigned char)((Data>>8)&0xff);
	tx_message.Data[5] = (unsigned char)(Data&0xff);  

	CAN_Transmit(CAN1,&tx_message);
}

/**
  * @brief  返回BodanMotor对象地址
  * @param  None
  * @retval None
  */
BodanMotor_TypeDef* Get_BodanMotor(void)
{
	return &BodanMotorData;
}

/**
  * @brief  初始化拨弹电机速度环PID值
  * @param  None
  * @retval None
  */
void speedPIDOfBodanMotor_Init(void)
{
	speedPIDOfBodanMotor.P = 0.025f;
	speedPIDOfBodanMotor.I = 0;
	speedPIDOfBodanMotor.D = 1.55f;
	speedPIDOfBodanMotor.IMax = 100.0f;
}

/**
  * @brief  初始化拨弹电机位置环PID值
  * @param  None
  * @retval None
  */
void positionPIDOfBodanMotor_Init(void)
{
	positionPIDOfBodanMotor.P = 17.5f;
	positionPIDOfBodanMotor.I = 0.05;
	positionPIDOfBodanMotor.D = 0;
	positionPIDOfBodanMotor.IMax = 100.0f;
}

/**
  * @brief  拨盘控制
  * @param  None
  * @retval None
  */
short speedOfBodanMotor = 3000;
short positionOfBodanMotor;
short currentOfBodanMotor;
short pre_speed;
short cur_speed;
short DriverPlateTask(void)   
{
	RemoteRec_TypeDef *RemoteData = Get_RemoteData();
	
	if(RemoteData->rc.s2 == 1)
		speedOfBodanMotor = 0;
	else
		speedOfBodanMotor = 3000;
	
//	cur_speed = BodanMotorData.RealSpeed;
////	speedOfBodanMotor = 850 + (850*sin(0.0314*t/5));                 //(*RemoteData).rc.ch0 - 1024;
////	speedPIDOfBodanMotor.SetPoint = speedOfBodanMotor;
//	positionOfBodanMotor = ((*RemoteData).rc.ch0 - 1024);
//	positionPIDOfBodanMotor.SetPoint = positionOfBodanMotor*5;
//	
	//检测卡弹
	if(ABS(cur_speed) < 20 && ABS(pre_speed) < 20)
		speedOfBodanMotor = -speedOfBodanMotor;
	
	speedPIDOfBodanMotor.SetPoint = speedOfBodanMotor; //PID_Calc(&positionPIDOfBodanMotor,ZeroCheck_BodanMotor()/36);        //位置环计算,2310减速比为36:1
	currentOfBodanMotor += PID_Calc(&speedPIDOfBodanMotor, BodanMotorData.RealSpeed);   //速度环计算
	currentOfBodanMotor = LIMIT_MAX_MIN(currentOfBodanMotor, 4800, -4800);
	
	pre_speed = cur_speed;
	
	return currentOfBodanMotor;
}

/**
  * @brief  拨弹电机过零检测
  * @param  None
  * @retval None
  */
int ZeroCheck_BodanMotor(void)
{
	static int flag;
	if(!flag)
	{
		flag = 1;
		currentPositionOfBodanMotor = lastPositionOfBodanMotor = BodanMotorData.Angle;
	}
	
	currentPositionOfBodanMotor = BodanMotorData.Angle;
	if(currentPositionOfBodanMotor - lastPositionOfBodanMotor < -4000)      //正向过零
	{
		relativePositionOfBodanMotor += (8191 - lastPositionOfBodanMotor + currentPositionOfBodanMotor); 
	}
	else if(currentPositionOfBodanMotor - lastPositionOfBodanMotor > 4000)   //反向过零
	{
		relativePositionOfBodanMotor += (-8191 - lastPositionOfBodanMotor + currentPositionOfBodanMotor); 
	}
	else
	{
		relativePositionOfBodanMotor += (currentPositionOfBodanMotor - lastPositionOfBodanMotor);
	}
	
	lastPositionOfBodanMotor = currentPositionOfBodanMotor;
	return relativePositionOfBodanMotor;
}
