#ifndef _STATEMACHINE_H_
#define _STATEMACHINE_H_

#define  PATROL_MODE  		 1   	//巡逻模式
#define  FIXED_SHOOT_MODE  2    //固定射击模式
#define  DODGE_MODE				 3    //躲避射击模式

void State_Init(void);
void State_Switch(void);
void Patrol_Mode(void);
void Fixed_Shoot_Mode(void);
unsigned char Get_Cur_Mode(void);

#endif
