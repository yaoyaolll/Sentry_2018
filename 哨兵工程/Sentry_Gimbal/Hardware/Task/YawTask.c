#include "main.h"

//0 degree ~ -340 degree
YawMotor_TypeDef YawMotorData;
Pid_Typedef speedPIDOfYawMotor;
Pid_Typedef positionPIDOfYawMotor;
float lastYawAngle;
float curYawAngle;
int circleCount;

/**
  * @brief  返回YawMotorData对象地址
  * @param  None
  * @retval None
  */
YawMotor_TypeDef* Get_YawMotorData(void)
{
	return &YawMotorData;
}

/**
  * @brief  Yaw轴过零处理
  * @param  None
  * @retval None
  */
float YawZeroCheck(void)
{
	float errorAngle;
	gyro_Typedef *GyroReceive = Get_GyroReceive();
	
	curYawAngle = GyroReceive->YAW;
	errorAngle = curYawAngle - lastYawAngle;
	
	if(errorAngle < -180)  //180到-180,正转
	{
		circleCount ++;
	}
	else if(errorAngle > 180)  //-180到180,反转
	{
		circleCount --;
	}
	
	lastYawAngle = curYawAngle;
	return circleCount*360 + curYawAngle;
}

/**
  * @brief  Yaw轴速度PID初始化
  * @param  None
  * @retval None
  */
void speedPIDOfYawMotor_Init(void)
{
	speedPIDOfYawMotor.P = 2.75f;
	speedPIDOfYawMotor.I = 0;
	speedPIDOfYawMotor.D = 2.5f;
	speedPIDOfYawMotor.IMax = 500.0f;
}

/**
  * @brief  Yaw轴位置PID初始化
  * @param  None
  * @retval None
  */
void positionPIDOfYawMotor_Init(void)
{
	positionPIDOfYawMotor.P =  10.0f;//2.5f;  // 20.0f;
	positionPIDOfYawMotor.I = 0.25f;
	positionPIDOfYawMotor.D =  0;      //12.5f;
	positionPIDOfYawMotor.IMax = 500.0f;
}

float positionOfYawMotor;
float speedOfYawMotor;
short currentOfYawMotor;
/**
  * @brief  Yaw轴电机速度环控制
  * @param  None
  * @retval None
  */
short Single_YawTask(float yaw_Speed) 
{
	RemoteRec_TypeDef *RemoteData = Get_RemoteData();
	gyro_Typedef *GyroReceive = Get_GyroReceive();
	
	//速度环
//	speedOfYawMotor = yaw_Speed;    
	speedOfYawMotor = (1024 - (*RemoteData).rc.ch2)*10;
	speedPIDOfYawMotor.SetPoint = speedOfYawMotor;
	currentOfYawMotor = PID_Calc(&speedPIDOfYawMotor, YawMotorData.RealSpeed);
	currentOfYawMotor = LIMIT_MAX_MIN(currentOfYawMotor, 4800, -4800);
	
	return currentOfYawMotor;
}

/**
  * @brief  Yaw轴电机速度环位置环控制
  * @param  None
  * @retval None
  */
short Double_YawTask(float yaw_Pos)
{
	RemoteRec_TypeDef *RemoteData = Get_RemoteData();
	gyro_Typedef *GyroReceive = Get_GyroReceive();
	
	//速度加位置环
//	positionOfYawMotor = (1024 - (*RemoteData).rc.ch2)*2 + 300;
	positionOfYawMotor = yaw_Pos*10;
	positionOfYawMotor = LIMIT_MAX_MIN(positionOfYawMotor, 2500, 0);
	positionPIDOfYawMotor.SetPoint = positionOfYawMotor;
	
	speedPIDOfYawMotor.SetPoint = -PID_Calc(&positionPIDOfYawMotor, YawZeroCheck()*10);
	currentOfYawMotor = PID_Calc(&speedPIDOfYawMotor, YawMotorData.RealSpeed);
	currentOfYawMotor = LIMIT_MAX_MIN(currentOfYawMotor, 4800, -4800);
	
	return currentOfYawMotor;
}
