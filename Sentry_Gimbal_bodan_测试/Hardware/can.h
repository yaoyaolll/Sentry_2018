#ifndef __CAN_H__
#define __CAN_H__

#define ChassisMotor_ID1 0x201
#define ChassisMotor_ID2 0x202
#define PitchMotor_ID    0x203
#define YawMotor_ID      0x204

typedef struct
{
	unsigned short angle;//0~8191(0x1fff)
	short real_speed;//     RPM
}_820r_motor_t;

typedef struct
{
	unsigned short angle;
	short real_flow;
	short set_flow;
}_6623_motor_t;

void CAN_Configuration(void);
//void CANSendCrash(unsigned int add,short state); 

#endif 
