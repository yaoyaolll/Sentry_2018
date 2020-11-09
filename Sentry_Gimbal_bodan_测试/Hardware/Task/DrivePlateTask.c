#include "main.h"

BodanMotor_TypeDef BodanMotorData;        //��������ṹ��
Pid_Typedef speedPIDOfBodanMotor;         //��������ٶȻ�PID�ṹ��
Pid_Typedef positionPIDOfBodanMotor;      //�������λ�û��ṹ��
int relativePositionOfBodanMotor;         //����������λ��
short lastPositionOfBodanMotor;           //���������һ��λ��
short currentPositionOfBodanMotor;        //���������ǰλ��

/**
  * @brief  ���Ͳ����������ĵ���ֵ
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
  * @brief  ����BodanMotor�����ַ
  * @param  None
  * @retval None
  */
BodanMotor_TypeDef* Get_BodanMotor(void)
{
	return &BodanMotorData;
}

/**
  * @brief  ��ʼ����������ٶȻ�PIDֵ
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
  * @brief  ��ʼ���������λ�û�PIDֵ
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
  * @brief  ���̿���
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
	//��⿨��
	if(ABS(cur_speed) < 20 && ABS(pre_speed) < 20)
		speedOfBodanMotor = -speedOfBodanMotor;
	
	speedPIDOfBodanMotor.SetPoint = speedOfBodanMotor; //PID_Calc(&positionPIDOfBodanMotor,ZeroCheck_BodanMotor()/36);        //λ�û�����,2310���ٱ�Ϊ36:1
	currentOfBodanMotor += PID_Calc(&speedPIDOfBodanMotor, BodanMotorData.RealSpeed);   //�ٶȻ�����
	currentOfBodanMotor = LIMIT_MAX_MIN(currentOfBodanMotor, 4800, -4800);
	
	pre_speed = cur_speed;
	
	return currentOfBodanMotor;
}

/**
  * @brief  �������������
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
	if(currentPositionOfBodanMotor - lastPositionOfBodanMotor < -4000)      //�������
	{
		relativePositionOfBodanMotor += (8191 - lastPositionOfBodanMotor + currentPositionOfBodanMotor); 
	}
	else if(currentPositionOfBodanMotor - lastPositionOfBodanMotor > 4000)   //�������
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
