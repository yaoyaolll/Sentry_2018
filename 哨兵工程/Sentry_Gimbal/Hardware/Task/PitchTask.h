#ifndef _PITCHTASK_H
#define _PITCHTASK_H

typedef struct
{
	unsigned short Angle;//0~8191(0x1fff)
	short RealSpeed;//     RPM
}PitchMotor_TypeDef;

PitchMotor_TypeDef* Get_PitchMotorData(void);
void speedPIDOfPitchMotor_Init(void);
short PitchTask(float pitch_Position);
void positionPIDOfPitchMotor_Init(void);

#endif
