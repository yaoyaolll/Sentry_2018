#ifndef _STATEMACHINE_H_
#define _STATEMACHINE_H_

#define  PATROL_MODE  		 1   	//Ѳ��ģʽ
#define  FIXED_SHOOT_MODE  2    //�̶����ģʽ
#define  DODGE_MODE				 3    //������ģʽ

void State_Init(void);
void State_Switch(void);
void Patrol_Mode(void);
void Fixed_Shoot_Mode(void);
unsigned char Get_Cur_Mode(void);

#endif
