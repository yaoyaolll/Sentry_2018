#ifndef __ADIS16448_H
#define __ADIS16448_H	
#include "main.h"
u16 ADIS16448_READ(u8 address);
void ADIS16448_WRITE(u8 BaseADDR, u16 data);
float ADIS16448_GYRO(u8 address);
float ADIS16448_ACCL(u8 address);
void EXTIX_Init(void);
void ADIS_Init(void);
#endif 
