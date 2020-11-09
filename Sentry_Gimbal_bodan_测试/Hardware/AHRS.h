#ifndef __AHRS_H
#define __AHRS_H	
#include "main.h"
void AHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);
float get_yaw(float mx,float my,float mz);
float get_accel_angle(void);
float yaw_filtering(float yaw);
#endif 

