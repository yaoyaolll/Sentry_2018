#include "main.h"
#include "string.h"

gyro_Typedef GyroReceive;   
extern int8_t GYRO_isConnectCnt; 

/**
  * @brief  陀螺仪数据解码函数
  * @param  None
  * @retval None
  */
void GyroReceiveFunc(unsigned char gyroBuffer[],short BufferNum)
{		
//	CanTxMsg tx_message;
	
	GyroReceive.YAW = ((signed int)((((gyroBuffer[(BufferNum+1)%GYRO_BUF_SIZE])&0xff) << 24)| (gyroBuffer[(BufferNum+2)%GYRO_BUF_SIZE]&0xff)<<16
		|(gyroBuffer[(BufferNum+3)%GYRO_BUF_SIZE]&0xff)<<8|(gyroBuffer[(BufferNum+4)%GYRO_BUF_SIZE]&0xff)))/ 100000.0f;
	
	GyroReceive.GZ = ((signed int)((((gyroBuffer[(BufferNum+5)%GYRO_BUF_SIZE])&0xff) << 24)| (gyroBuffer[(BufferNum+6)%GYRO_BUF_SIZE]&0xff)<<16
		|(gyroBuffer[(BufferNum+7)%GYRO_BUF_SIZE]&0xff)<<8|(gyroBuffer[(BufferNum+8)%GYRO_BUF_SIZE]&0xff)))/ 100000.0f;

	GyroReceive.PITCH = ((signed int)((((gyroBuffer[(BufferNum+9)%GYRO_BUF_SIZE])&0xff) << 24)| (gyroBuffer[(BufferNum+10)%GYRO_BUF_SIZE]&0xff)<<16
		|(gyroBuffer[(BufferNum+11)%GYRO_BUF_SIZE]&0xff)<<8|(gyroBuffer[(BufferNum+12)%GYRO_BUF_SIZE]&0xff)))/ 100000.0f;
	
	GyroReceive.GY= ((signed int)((((gyroBuffer[(BufferNum+13)%GYRO_BUF_SIZE])&0xff) << 24)| (gyroBuffer[(BufferNum+14)%GYRO_BUF_SIZE]&0xff)<<16
		|(gyroBuffer[(BufferNum+15)%GYRO_BUF_SIZE]&0xff)<<8|(gyroBuffer[(BufferNum+16)%GYRO_BUF_SIZE]&0xff)))/ 100000.0f;
					
	GyroReceive.MPUrecvd = 1; 
	GyroReceive.MPUDisconnectCnt = 0; 	

//	memcpy(tx_message.Data, &GyroReceive.YAW, 4);   //float2uchar
//	tx_message.IDE = CAN_ID_STD;    
//	tx_message.RTR = CAN_RTR_DATA; 
//	tx_message.DLC = 0x08;    
//	tx_message.StdId = 0x700;    //向主控板发送GYRO接收到的数据
//	
//	CAN_Transmit(CAN1, &tx_message);
////	
//	GYRO_isConnectCnt = 1;
}

/**
  * @brief  返回Pitch轴陀螺仪对象地址
  * @param  None
  * @retval None
  */
gyro_Typedef* Get_GyroReceive(void)
{
	return &GyroReceive;
}



//利用二次函数拟合平滑
void quadraticSmooth5(double in[], double out[], int N)  
{  
    int i;  
    if ( N < 5 )  
    {  
        for ( i = 0; i <= N - 1; i++ )  
        {  
            out[i] = in[i];  
        }  
    }  
    else  
    {  
        out[0] = ( 31.0 * in[0] + 9.0 * in[1] - 3.0 * in[2] - 5.0 * in[3] + 3.0 * in[4] ) / 35.0;  
        out[1] = ( 9.0 * in[0] + 13.0 * in[1] + 12 * in[2] + 6.0 * in[3] - 5.0 *in[4]) / 35.0;  
        for ( i = 2; i <= N - 3; i++ )  
        {  
            out[i] = ( - 3.0 * (in[i - 2] + in[i + 2]) +  
                      12.0 * (in[i - 1] + in[i + 1]) + 17 * in[i] ) / 35.0;  
        }  
        out[N - 2] = ( 9.0 * in[N - 1] + 13.0 * in[N - 2] + 12.0 * in[N - 3] + 6.0 * in[N - 4] - 5.0 * in[N - 5] ) / 35.0;  
        out[N - 1] = ( 31.0 * in[N - 1] + 9.0 * in[N - 2] - 3.0 * in[N - 3] - 5.0 * in[N - 4] + 3.0 * in[N - 5]) / 35.0;  
    }  
}  
