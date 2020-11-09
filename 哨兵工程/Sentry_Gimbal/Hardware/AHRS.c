/*
* AHRS
* Copyright 2010 S.O.H. Madgwick
*
* This program is free software: you can redistribute it and/or
* modify it under the terms of the GNU Lesser Public License as
* published by the Free Software Foundation, either version 3 of the
* License, or (at your option) any later version.
*
* This program is distributed in the hope that it will be useful, but
* WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
* Lesser Public License for more details.
*
* You should have received a copy of the GNU Lesser Public License
* along with this program.  If not, see
* <http://www.gnu.org/licenses/>.
*/
// AHRS.c
// S.O.H. Madgwick
// 25th August 2010
//
// Quaternion implementation of the 'DCM filter' [Mayhony et al].  Incorporates the magnetic distortion
// compensation algorithms from my filter [Madgwick] which eliminates the need for a reference
// direction of flux (bx bz) to be predefined and limits the effect of magnetic distortions to yaw
// axis only.
//
// User must define 'halfT' as the (sample period / 2), and the filter gains 'Kp' and 'Ki'.
//
// Global variables 'q0', 'q1', 'q2', 'q3' are the quaternion elements representing the estimated
// orientation.  See my report for an overview of the use of quaternions in this application.
//
// User must call 'AHRSupdate()' every sample period and parse calibrated gyroscope ('gx', 'gy', 'gz'),
// accelerometer ('ax', 'ay', 'ay') and magnetometer ('mx', 'my', 'mz') data.  Gyroscope units are
// radians/second, accelerometer and magnetometer units are irrelevant as the vector is normalised.
//

														
#include "main.h"
#include "string.h"
//#define K           0.001f             //互补滤波系数，系数越大加速度计影响越大
#define XRS_DELAY   0                   //弥补ADXRS453数据与MPU6050之间时间差
#define PI 3.1415926f

/* Private define ------------------------------------------------------------*/

#define halfT 0.00244f                // half the sample period        : 0.005s/2=0.0025s

#define ACCEL_1G 1000 //the acceleration of gravity is: 1000 mg

/* Public variables ----------------------------------------------------------*/
u8 InitEulerAngle_Finished = 0;

float Magnetoresistor_mGauss_X = 0, Magnetoresistor_mGauss_Y = 0, Magnetoresistor_mGauss_Z = 0;//unit: milli-Gauss                                                                                                                                                                                                      
float Accelerate_mg_X, Accelerate_mg_Y, Accelerate_mg_Z;//unit: mg                                                               
float AngularRate_dps_X, AngularRate_dps_Y, AngularRate_dps_Z;//unit: dps: degree per second      

int16_t Magnetoresistor_X, Magnetoresistor_Y, Magnetoresistor_Z;                                                                                                                                                                                                      
uint16_t Accelerate_X = 0, Accelerate_Y = 0, Accelerate_Z = 0;                                                                                                                                                                                               
uint16_t AngularRate_X = 0, AngularRate_Y = 0, AngularRate_Z = 0;

u8 Quaternion_Calibration_ok = 0;

float pitch, yaw, roll;
float pitch_plus,yaw_plus,roll_plus;
int EX_CNT=2;
/* Private macro -------------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/

/*******************************************************************************
* Function Name  : AHRSupdate
* Description    : None
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
int tim_cnt, cycle,zero_cnt,m_circle;
int is_gyro_disconnect_cnt;
int angle_pitch, gyro_pitch, angle_yaw, gyro_yaw;
int fliter_flag;
float offset_XRS, offset_yaw, offset_pitch, offset_roll,offset_m_yaw;
float offset_ax,offset_ay,offset_az;
float XRS_buff[100];
float accl_pitch, accl_pitch_last, comp_pitch;
float accl_roll, accl_roll_last, comp_roll;
float comp_yaw,m_yaw,m_yaw_last;
float integ_angle;
float accl1, accl2, accl3;
float XRS_gyro;
float XRS_gyro_delay;
float XRS_gyro_last;
float MPU_accel[3];		
float MPU_gyro[3];
int sys_cnt,sys_cnt_last,sys_delay;
float K_pitch=0.005,K_yaw=0;
float m_x,m_y,m_z;
float m_yaw_avg[10];

void AHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz)
{
	CanTxMsg see_tx_message;
	short PITCH;
	short YAW;
	short GY;
	short GZ;
	sys_delay=sys_cnt-sys_cnt_last;
	sys_cnt_last=sys_cnt;

	MPU_accel[0]=ax;
	MPU_accel[1]=ay;
	MPU_accel[2]=az;
	
	MPU_gyro[0]=gx;
	MPU_gyro[1]=gy;
	MPU_gyro[2]=gz;
	

	if(tim_cnt == 0)
	{
			comp_pitch = atan((float)MPU_accel[2] / (float)MPU_accel[0]) / 3.1415926f * 180.0f;
			
			if(comp_pitch < 0)
					comp_pitch += 90.0f;
			else
					comp_pitch -= 90.0f;
	}
/***********************************数据处理*********************************************/
	if(tim_cnt < 999)                   //前2秒累计误差
	{
			
			offset_pitch += MPU_gyro[0];    //计算MPU6050零偏
			offset_roll += MPU_gyro[1];     
			offset_yaw += MPU_gyro[2];      
			get_accel_angle();
			offset_m_yaw+=get_yaw(mx,my,mz);

			tim_cnt++;
	}
	else if (tim_cnt == 999)            //2秒时计算零偏
	{
			offset_pitch += MPU_gyro[0];
			offset_roll += MPU_gyro[1];
			offset_yaw += MPU_gyro[2];
			get_accel_angle();
			offset_m_yaw+=get_yaw(mx,my,mz);
		
			offset_pitch /= 1000.0f;
			offset_roll /= 1000.0f;
			offset_yaw /= 1000.0f;	
			offset_m_yaw /= 1000.0f;	
			tim_cnt++;
		
			m_yaw_last=offset_m_yaw;
	}
	else
	{
		
			MPU_gyro[0] -= offset_pitch;    //每次计算修正零偏
			MPU_gyro[1] -= offset_roll;
			MPU_gyro[2] -= offset_yaw;
			get_accel_angle();
/************************************pitch*********************************************/            

			pitch_plus=  comp_pitch + gy*(halfT*2)/3.1415926f * 180.0f; 
			comp_pitch = K_pitch * accl_pitch + (1 - K_pitch) * (comp_pitch + gy*(halfT*2)/3.1415926f * 180.0f);  //互补滤波
			
			
		if(comp_pitch > 180.0f)
				comp_pitch -= 360.0f;
		else if(comp_pitch > 180.0f)
				comp_pitch += 360.0f;
			
/*************************************roll*********************************************/              
//			roll_plus=comp_roll + gx*(halfT*2)/3.1415926f * 180.0f;
//			comp_roll = K * accl_roll + (1 - K) * (comp_roll + gx*(halfT*2)/3.1415926f * 180.0f);  //互补滤波
//			
//			if(comp_roll > 180.0f)
//					comp_roll -= 360.0f;
//			else if(comp_roll > 180.0f)
//					comp_roll += 360.0f;拍	去111111111111q=
//			
/**************************************yaw*********************************************/ 
		yaw_plus=comp_yaw + gz*(halfT*2)/3.1415926f * 180.0f;
			
			if(yaw_plus>90.0f-offset_m_yaw)			
			{
					m_circle=1;
				 if(yaw_plus>270.0f-offset_m_yaw)
				 {
					 m_circle=2;
				 }
			}
			else if(yaw_plus<-90.0f-offset_m_yaw)
			{
				m_circle=-1;					
				if(yaw_plus<-270.0f-offset_m_yaw)
				 {
					 m_circle=-2;
				 }
			}
			else 
				m_circle=0;			

			m_yaw=get_yaw(mx,my,mz)- offset_m_yaw+m_circle*180.0f;
			
			yaw_filtering(get_yaw(mx,my,mz));
			if(fliter_flag==0)
			{
				yaw_plus=comp_yaw + gz*(halfT*2)/3.1415926f * 180.0f;
				comp_yaw = K_yaw * m_yaw + (1 - K_yaw) * (comp_yaw + gz*(halfT*2)/3.1415926f * 180.0f);  //互补滤波
			}
			else if(fliter_flag==1)
			{
				comp_yaw=comp_yaw + gz*(halfT*2)/3.1415926f * 180.0f;
			}
	}
	see_tx_message.IDE = CAN_ID_STD;    
	see_tx_message.RTR = CAN_RTR_DATA; 
	see_tx_message.DLC = 0x08;    
	see_tx_message.StdId = 0x704;    //向主控板发送GYRO接收到的数据
	
	PITCH = (short)(comp_pitch*100);
	YAW = (short)(comp_yaw*100);
	GY = (short)(MPU_gyro[1]*100);
	GZ = (short)(MPU_gyro[2]*100);
	
	see_tx_message.Data[0] = PITCH>>8&0xff;
	see_tx_message.Data[1] = PITCH&0xff;
	see_tx_message.Data[2] = GY>>8&0xff;
	see_tx_message.Data[3] = GY&0xff;
	see_tx_message.Data[4] = GZ>>8&0xff;
	see_tx_message.Data[5] = GZ&0xff;
	see_tx_message.Data[6] = YAW>>8&0xff;
	see_tx_message.Data[7] = YAW&0xff;
	CAN_Transmit(CAN1, &see_tx_message);
	
	is_gyro_disconnect_cnt = 0;
}

float clamp(float Value, float Min, float Max)
{
	if(Value > Max)
	{
		return Max;
	}
	else if(Value < Min)
	{
		return Min;
	}
	else
	{
		return Value;
	}
}


float get_yaw(float mx,float my,float mz)
{
	float m;
	float n;
	float arc;
	m_x=mx;
	m_y=my;
	m_z=mz;
	
		m=my*cos(accl_roll/180.0f*PI)+mx*sin(accl_roll/180.0f*PI)*sin(accl_pitch/180.0f*PI)-mz*cos(accl_pitch/180.0f*PI)*sin(accl_roll/180.0f*PI);
		n=mx*cos(accl_pitch/180.0f*PI)+mz*sin(accl_pitch/180.0f*PI);
		arc=-atan(m/n)/PI*180.0f;
	return 	arc;
}



float get_accel_angle()
{
		accl_pitch = atan((float)MPU_accel[2] / (float)MPU_accel[0]) / 3.1415926f * 180.0f; 
		if(accl_pitch < 0)
			 accl_pitch += 90.0f;
		else
			 accl_pitch -= 90.0f;
            
	 if((accl_pitch - accl_pitch_last) > 150)
			accl_pitch += 180;
	 if((accl_pitch - accl_pitch_last) < -150)
			accl_pitch -=180;
	 accl_pitch_last = accl_pitch;
 
	accl_roll = -atan((float)MPU_accel[2] / (float)MPU_accel[1]) / 3.1415926f * 180.0f;
						
	 if(accl_roll < 0)
			accl_roll += 90.0f;
	 else
			accl_roll -= 90.0f;
							 
	 if((accl_roll - accl_roll_last) > 150)
			accl_roll += 180;
	 if((accl_roll - accl_roll_last) < -150)
			accl_roll -=180;
		accl_roll_last = accl_roll;
 
	
}



float yaw_filtering(float yaw)
{
	static int i;
	int j;
	float avg;
	if(i<=9)
	{
		m_yaw_avg[i]=m_yaw;
		i++;
	}
	else 
	{
		for(j=0;j<9;j++)
		{
			m_yaw_avg[j]=m_yaw_avg[j+1];
			avg+=m_yaw_avg[j];
		}
		m_yaw_avg[9]=m_yaw;
		avg=(avg+m_yaw_avg[9])/10.0f;
		if(m_yaw-avg>=150||m_yaw-avg<=-150)
		{
			fliter_flag=1;		//表示有信号干扰
		}
		else 
			fliter_flag=0;
	}
		
	
	if(yaw>=85||yaw<=-85)
		fliter_flag=1;
	else 
		fliter_flag=0;
}

