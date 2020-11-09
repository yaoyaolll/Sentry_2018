#ifndef _CHASSISTASK_H
#define _CHASSISTASK_H

typedef struct
{
	unsigned short Angle;//0~8191(0x1fff)
	short RealSpeed;//     RPM
}ChassisMotor_TypeDef;

void Chassis_SendData(short *Data);
ChassisMotor_TypeDef* Get_ChassisMotor(void);
void speedPIDOfChassisMotor_Init(void);
short* ChassisTask(short speed);

#endif
