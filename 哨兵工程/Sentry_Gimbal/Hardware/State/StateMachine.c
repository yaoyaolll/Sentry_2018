#include "main.h"

unsigned char cur_Mode;
extern int encoderAll;
extern short chassis_Speed;
extern short yaw_Speed;
extern float pitch_Position;
extern float yaw_Position;
extern float Yaw_Degree;   //Yaw轴绝对角度

/**
  * @brief  状态初始化
  * @param  None
  * @retval None
  */
void State_Init(void)
{
	cur_Mode = PATROL_MODE;
}

/**
  * @brief  返回当前模式
  * @param  None
  * @retval None
  */
unsigned char Get_Cur_Mode(void)
{
	return cur_Mode;
}

/**
  * @brief  模式切换
  * @param  None
  * @retval None
  */
void State_Switch(void)
{
	PC_TypeDef *PC_Receive = Get_PC_Receive(); 
	
//	switch (PC_Receive->Mode){
	switch (cur_Mode){
		
		case PATROL_MODE:
			//巡逻，收到视觉信号，转换为固定射击
			Patrol_Mode(); 
			break;
		
		case FIXED_SHOOT_MODE:
			//固定射击，收到视觉信号，转换为巡逻模式
			Patrol_Mode();
			break;
		
		case DODGE_MODE: 
			//躲避模式，自我识别一段时间后，转换为巡逻模式
			break;
	}
}

/**
  * @brief  自动巡逻模式
  * @param  None
  * @retval None
  */
void Patrol_Mode(void)
{
	short Road_State = Get_Road_State();    //获取当前位置
	gyro_Typedef *GyroReceive = Get_GyroReceive();
	
	if(Road_State == Start_Point)                  //起点
	{
		encoderAll = 0;
		chassis_Speed = -5000;                       //正向前行
	}
	else if(Road_State == End_Point)               //终点
	{
		chassis_Speed = 5000;                        //反向前行
	}
	
	if(Road_State == Strai_Road2)                   //Yaw轴自转，在Strai_Road2时，陀螺仪会变化50度（官方60度）
	{
		Yaw_Degree -= 50; 
	}
	if(Yaw_Degree < 15)                         
	{
		yaw_Speed = -1200;
	}
	if(Yaw_Degree > 280)
	{
		yaw_Speed = 1200;
	}
	
	pitch_Position = -17;                         //Pitch轴始终保持在-17度
}

/**
  * @brief  固定射击模式
  * @param  None
  * @retval None
  */
void Fixed_Shoot_Mode(void)         
{
	PC_TypeDef *PC_Receive = Get_PC_Receive();
	gyro_Typedef *GyroReceive = Get_GyroReceive();
	
	//底盘，Pitch，Yaw电机
	chassis_Speed = 0;
	pitch_Position = PC_Receive->PitchDegree;
	yaw_Position = PC_Receive->YawDegree;
	
	//拨弹电机,由PC发送的射频来确定转速
	
}

/**
  * @brief  躲避模式
  * @param  None
  * @retval None
  */
void Dodge_Mode(void)
{
	
}
