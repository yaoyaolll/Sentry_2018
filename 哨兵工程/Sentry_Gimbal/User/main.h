#ifndef __MAIN_H__
#define __MAIN_H__

#include "stm32f10x.h"
#include <stdio.h>
#include <stdlib.h>
#include "math.h"
#include "string.h"

#include "can.h"
#include "usart1.h"
#include "usart2.h"
#include "usart3.h"
#include "frictionWheel.h"
#include "pid.h"
#include "algorithmOfCRC.h"
#include "laser.h"
#include "solenoidSW.h"
#include "led.h"

#include "RemoteTask.h"
#include "GYROTask.h"
#include "PC_Task.h"
 

#include <spi.h>
#include <ADIS16448.h>
#include <AHRS.h>
#include <delay.h>


#define SLAVE_ISCNT_ID 0x555

void system_Config(void);
void system_Init(void);
void delay_ms(unsigned int t);

#define ABS(x) ((x)>0? (x):(-(x))) 
#define LIMIT_MAX_MIN(x, max, min)	(((x) <= (min)) ? (min):(((x) >= (max)) ? (max) : (x)))

#endif
