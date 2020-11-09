#include "main.h"

#define PITCH_POSITION_INIT -14
/*-34.0 degree ~ 0.0 degree, from bottom to top.
*/

PitchMotor_TypeDef PitchMotorData;
Pid_Typedef speedPIDOfPitchMotor;
Pid_Typedef positionPIDOfPitchMotor;

/**
  * @brief  返回PitchMotorData对象地址
  * @param  None
  * @retval None
  */
PitchMotor_TypeDef* Get_PitchMotorData(void)
{
	return &PitchMotorData;
}

/**
  * @brief  Pitch轴电机速度PID参数初始化
  * @param  None
  * @retval None
  */
void speedPIDOfPitchMotor_Init(void)
{
	speedPIDOfPitchMotor.P = 3.75f;//3.25f;//6.25f;
	speedPIDOfPitchMotor.I = 0.0f;
	speedPIDOfPitchMotor.D = 8.5f;//8.5f;//12.5f;
	speedPIDOfPitchMotor.IMax = 0.0f;
}

/**
  * @brief  Pitch轴电机位置PID参数初始化
  * @param  None
  * @retval None
  */
void positionPIDOfPitchMotor_Init(void)
{
	positionPIDOfPitchMotor.P = 27.5f;
	positionPIDOfPitchMotor.I = 0.25f;//.105f;
	positionPIDOfPitchMotor.D = 0;//20.5f;
	positionPIDOfPitchMotor.IMax = 500.0f;
}
/**
  * @brief  Pitch轴电机控制
  * @param  None
  * @retval None
  */
float positionOfPitchMotor;
float speedOfPitchMotor;
short currentOfPitchMotor;
short PitchTask(float pitch_Position) 
{
	RemoteRec_TypeDef *RemoteData = Get_RemoteData();
	gyro_Typedef *GyroReceive = Get_GyroReceive();
	
	positionOfPitchMotor = pitch_Position;
//	positionOfPitchMotor = ((1024 - (*RemoteData).rc.ch3)/10 - 17.0);   //initial value is -170.
	positionOfPitchMotor = LIMIT_MAX_MIN(positionOfPitchMotor, 0, -34.0f);
	positionPIDOfPitchMotor.SetPoint = positionOfPitchMotor;
	speedPIDOfPitchMotor.SetPoint = PID_Calc(&positionPIDOfPitchMotor, (*GyroReceive).PITCH);
	currentOfPitchMotor = PID_Calc(&speedPIDOfPitchMotor, PitchMotorData.RealSpeed);
	currentOfPitchMotor = LIMIT_MAX_MIN(currentOfPitchMotor, 4800, -4800);
	
//	speedOfPitchMotor = (1024 - (*RemoteData).rc.ch3);
//	speedPIDOfPitchMotor.SetPoint = speedOfPitchMotor;
//	currentOfPitchMotor = PID_Calc(&speedPIDOfPitchMotor, PitchMotorData.RealSpeed);
//	currentOfPitchMotor = LIMIT_MAX_MIN(currentOfPitchMotor, 4800, -4800);
	
	return currentOfPitchMotor;
}



