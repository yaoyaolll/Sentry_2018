#ifndef _DRIVEPLATETASK_H_
#define _DRIVEPLATETASK_H_

typedef struct
{
	unsigned short Angle;//0~8191(0x1fff)
	short RealSpeed;//     RPM
}BodanMotor_TypeDef;

void DriverPlate_SendData(short Data);
BodanMotor_TypeDef* Get_BodanMotor(void);
short DriverPlateTask(void);
void speedPIDOfBodanMotor_Init(void);
int ZeroCheck_BodanMotor(void);
void positionPIDOfBodanMotor_Init(void);

#endif
