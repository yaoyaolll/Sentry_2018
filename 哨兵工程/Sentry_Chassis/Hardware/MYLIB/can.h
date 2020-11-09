#ifndef __CAN_H__
#define __CAN_H__

#include "stdint.h"

#define ChassisMotor_ID1 0x201
#define ChassisMotor_ID2 0x202
#define BodanMotor_ID    0x203
#define PitchMotor_ID    0x206
#define YawMotor_ID      0x205


#define CAN_PC_ID        0x333
#define CAN_GYRO_ID      0x444

void CAN_Configuration(void);

#endif 
