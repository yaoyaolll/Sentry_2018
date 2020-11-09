#ifndef _GYROTASK_H
#define _GYROTASK_H

typedef struct GYRO{
		float AX;
		float AY;
		float AZ;
		float GX;
		float GY;
		float GZ;
		float PITCH;
		float ROLL;
		float YAW;
		float Temperature;
	  volatile char MPUrecvd;
    volatile char MPUDisconnectCnt;
}
gyro_Typedef;

void GyroReceiveFunc(unsigned char gyroBuffer[],short BufferNum);
gyro_Typedef* Get_GyroReceive(void);
void quadraticSmooth5(double in[], double out[], int N);

#endif
