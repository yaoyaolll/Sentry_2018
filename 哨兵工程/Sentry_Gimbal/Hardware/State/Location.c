#include "main.h"

unsigned char Road_State;
phSensor_TypeDef phSensor; 

extern short encoderCount;
extern int encoderAll;

/**
  * @brief  ��ʼ��λ��ֵ
  * @param  None
  * @retval None
  */
void Road_Init(void)
{
	Road_State = Strai_Road1; 
}

/**
  * @brief  ���ݹ�翪�ر任λ��ֵ
  * @param  None
  * @retval None
  */
void Location_Switch(void)
{
	phSensor.cur_phS1 = PE_SENSOR1;     //ǰ��������翪��
	phSensor.cur_phS2 = PE_SENSOR2;
	
	phSensor.cur_phS3 = PE_SENSOR3;     //���������翪��       
	phSensor.cur_phS4 = PE_SENSOR4;
	
	phSensor.cur_phS5 = PE_SENSOR5;     //�Ҳ�������翪��
	phSensor.cur_phS6 = PE_SENSOR6;
	//״̬�ж�����ת
	switch (Road_State){
		
		case (Start_Point):
			if(phSensor.cur_phS1 == phS_OFF && phSensor.pre_phS1 == phS_OFF)
				Road_State = Strai_Road1;
			break;
			
		case (Strai_Road1):
			if(phSensor.cur_phS1 == phS_ON && phSensor.pre_phS1 == phS_OFF)
				Road_State = Start_Point;
			if(phSensor.cur_phS5 == phS_OFF || phSensor.cur_phS6 == phS_OFF)
				Road_State = Curve_Road1;
			break;
			
		case (Curve_Road1):
			if(phSensor.cur_phS5 == phS_ON && phSensor.cur_phS6 == phS_ON && encoderCount < 0)  //encoderCount<0 to straight_road2
				Road_State = Strai_Road2;
			if(phSensor.cur_phS5 == phS_ON && phSensor.cur_phS6 == phS_ON && encoderCount > 0)  //encoderCount>0 to straight_road1
				Road_State = Strai_Road1;
			break;
		
		case (Strai_Road2):
			if(phSensor.cur_phS5 == phS_OFF || phSensor.cur_phS6 == phS_OFF)
				Road_State = Curve_Road1;
			if(phSensor.cur_phS3 == phS_OFF || phSensor.cur_phS4 == phS_OFF)
				Road_State = Curve_Road2;
			break;
			
		case (Curve_Road2):
			if(phSensor.cur_phS3 == phS_ON && phSensor.cur_phS4 == phS_ON && encoderCount < 0)  //encoderCount<0 to straight_road3
				Road_State = Strai_Road3;
			if(phSensor.cur_phS3 == phS_ON && phSensor.cur_phS4 == phS_ON && encoderCount > 0)  //encoderCount>0 to straight_road2
				Road_State = Strai_Road2;
			break;
			
		case (Strai_Road3):
			if(phSensor.cur_phS2 == phS_ON && phSensor.pre_phS2 == phS_OFF)
				Road_State = End_Point;
			if(phSensor.cur_phS3 == phS_OFF || phSensor.cur_phS4 == phS_OFF)
				Road_State = Curve_Road2;
			break;
		
		case (End_Point):
			if(phSensor.cur_phS2 == phS_OFF && phSensor.pre_phS2 == phS_OFF)
				Road_State = Strai_Road3;
			break;
			
		default:
			break;
	}
	
	//�˴����������������ֱ����ֵ���Խ���λ��
//	if(encoderAll)
//		Road_State = Strai_Road1;
//	else if(encoderAll)
//		Road_State = Strai_Road2;
//	else if(encoderAll)
//		Road_State = Strai_Road3;
	
	phSensor.pre_phS1 = phSensor.cur_phS1;
	phSensor.pre_phS2 = phSensor.cur_phS2;
}

/**
  * @brief  ����Road_State
  * @param  None
  * @retval None
  */
unsigned char Get_Road_State(void)
{
	return Road_State;
}
