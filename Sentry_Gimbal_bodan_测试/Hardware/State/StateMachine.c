#include "main.h"

unsigned char cur_Mode;
extern int encoderAll;
extern short chassis_Speed;
extern short yaw_Speed;
extern float pitch_Position;
extern float yaw_Position;
extern float Yaw_Degree;   //Yaw����ԽǶ�

/**
  * @brief  ״̬��ʼ��
  * @param  None
  * @retval None
  */
void State_Init(void)
{
	cur_Mode = PATROL_MODE;
}

/**
  * @brief  ���ص�ǰģʽ
  * @param  None
  * @retval None
  */
unsigned char Get_Cur_Mode(void)
{
	return cur_Mode;
}

/**
  * @brief  ģʽ�л�
  * @param  None
  * @retval None
  */
void State_Switch(void)
{
	PC_TypeDef *PC_Receive = Get_PC_Receive(); 
	
//	switch (PC_Receive->Mode){
	switch (cur_Mode){
		
		case PATROL_MODE:
			//Ѳ�ߣ��յ��Ӿ��źţ�ת��Ϊ�̶����
			Patrol_Mode(); 
			break;
		
		case FIXED_SHOOT_MODE:
			//�̶�������յ��Ӿ��źţ�ת��ΪѲ��ģʽ
			Patrol_Mode();
			break;
		
		case DODGE_MODE: 
			//���ģʽ������ʶ��һ��ʱ���ת��ΪѲ��ģʽ
			break;
	}
}

/**
  * @brief  �Զ�Ѳ��ģʽ
  * @param  None
  * @retval None
  */
void Patrol_Mode(void)
{
	short Road_State = Get_Road_State();    //��ȡ��ǰλ��
	gyro_Typedef *GyroReceive = Get_GyroReceive();
	
	if(Road_State == Start_Point)                  //���
	{
		encoderAll = 0;
		chassis_Speed = -5000;                       //����ǰ��
	}
	else if(Road_State == End_Point)               //�յ�
	{
		chassis_Speed = 5000;                        //����ǰ��
	}
	
	if(Road_State == Strai_Road2)                   //Yaw����ת����Strai_Road2ʱ�������ǻ�仯50�ȣ��ٷ�60�ȣ�
	{
		Yaw_Degree -= 50; 
	}
	if(Yaw_Degree < 15)                         
	{
		yaw_Speed = -1200;
	}
	if(Yaw_Degree > 280)
	{
		yaw_Speed = 1200;
	}
	
	pitch_Position = -17;                         //Pitch��ʼ�ձ�����-17��
}

/**
  * @brief  �̶����ģʽ
  * @param  None
  * @retval None
  */
void Fixed_Shoot_Mode(void)         
{
	PC_TypeDef *PC_Receive = Get_PC_Receive();
	gyro_Typedef *GyroReceive = Get_GyroReceive();
	
	//���̣�Pitch��Yaw���
	chassis_Speed = 0;
	pitch_Position = PC_Receive->PitchDegree;
	yaw_Position = PC_Receive->YawDegree;
	
	//�������,��PC���͵���Ƶ��ȷ��ת��
	
}

/**
  * @brief  ���ģʽ
  * @param  None
  * @retval None
  */
void Dodge_Mode(void)
{
	
}
