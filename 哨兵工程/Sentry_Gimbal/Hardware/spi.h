#ifndef __SPI_H
#define __SPI_H
#include "main.h"
#include "stm32f10x_spi.h"
 
 				  	    													  
void SPI1_Init(void);			 //��ʼ��SPI��
  
u8 SPI1_ReadWriteByte(u8 TxData);
		 
#endif

