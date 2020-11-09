#ifndef _YAWTASK_H_
#define _YAWTASK_H_

typedef struct
{
	unsigned short Angle;//0~8191(0x1fff)
	short RealSpeed;//     RPM
}YawMotor_TypeDef;

float YawZeroCheck(void);
YawMotor_TypeDef* Get_YawMotorData(void);
void speedPIDOfYawMotor_Init(void);
void positionPIDOfYawMotor_Init(void);
short Single_YawTask(float yaw_Speed);
short Double_YawTask(float yaw_Pos);

#endif 
