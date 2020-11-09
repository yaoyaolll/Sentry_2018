#include "main.h"

void SPI1_Init(void)
{
 	GPIO_InitTypeDef GPIO_InitStructure;
	SPI_InitTypeDef  SPI_InitStructure;

	RCC_APB2PeriphClockCmd(	RCC_APB2Periph_GPIOB|RCC_APB2Periph_AFIO, ENABLE );
	RCC_APB1PeriphClockCmd(	RCC_APB1Periph_SPI2 , ENABLE );

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;  
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;  
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
 	GPIO_SetBits(GPIOB, GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15);  
	
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;  
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;		
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;		
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;		
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;	
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;		
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_64;		
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;	
	SPI_InitStructure.SPI_CRCPolynomial = 7;	
	SPI_Init(SPI2, &SPI_InitStructure);  
 
	SPI_Cmd(SPI2, ENABLE); 

}   

u8 SPI1_ReadWriteByte(u8 TxData)
{		
			 	
	while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET);		  
	SPI_I2S_SendData(SPI2, TxData);

	while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_RXNE) == RESET);  						    
	return SPI_I2S_ReceiveData(SPI2); 			    
}


