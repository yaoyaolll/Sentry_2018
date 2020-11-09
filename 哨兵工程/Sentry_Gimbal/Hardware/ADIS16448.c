#include "main.h"

u16 ADIS16448_READ(u8 address)
{
	u16 data;
	u8 spirc0, spirc1;
	GPIO_ResetBits(GPIOA, GPIO_Pin_4);	
	
	spirc0 = SPI1_ReadWriteByte(address & 0x7F);
	spirc1 = SPI1_ReadWriteByte(0x00);
	
	GPIO_SetBits(GPIOA, GPIO_Pin_4);
	
	data = spirc0 << 8 | spirc1;
	return data;
}

void ADIS16448_WRITE(u8 BaseADDR, u16 data)
{
	GPIO_ResetBits(GPIOB, GPIO_Pin_12);	
	
	SPI1_ReadWriteByte(BaseADDR | 0X80);
	SPI1_ReadWriteByte(data & 0xFF);
	
	GPIO_SetBits(GPIOB, GPIO_Pin_12);
	
	delay_ms(10);
	
	GPIO_ResetBits(GPIOB, GPIO_Pin_12);	
	
	SPI1_ReadWriteByte((BaseADDR + 1) | 0X80);
	SPI1_ReadWriteByte((data >> 8)  & 0XFF);
	
	GPIO_SetBits(GPIOB, GPIO_Pin_12);
}

float ADIS16448_GYRO(u8 address)
{
	int data;
	float GYRO;
	data = ADIS16448_READ(address);
	if (data > 0x7fff)
		data -= 0x10000;
	GYRO = data / 25.0F * 3.1415926f / 180.0f;
//	GYRO = data;
	return GYRO;
}

float ADIS16448_ACCL(u8 address)
{
	int data;
	float ACCL;
	data = ADIS16448_READ(address);
	if (data > 0x7fff)
		data -= 0x10000;
	ACCL = data / 1200.0f;
	return ACCL;
}

float ADIS16448_MAGN(u8 address)
{
	int data;
	float MAGN;
	data = ADIS16448_READ(address);
	if (data > 0x7fff)
		data -= 0x10000;
	MAGN = data / 7.0f;
	return MAGN;
}
void EXTIX_Init(void)
{
	EXTI_InitTypeDef EXTI_InitStructure;
 	NVIC_InitTypeDef NVIC_InitStructure;
    GPIO_InitTypeDef  GPIO_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);	
  	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);	
    
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;				
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD; 		 
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 
    GPIO_Init(GPIOA, &GPIO_InitStructure);					 


  	GPIO_EXTILineConfig(GPIO_PortSourceGPIOA,GPIO_PinSource8);
  	EXTI_InitStructure.EXTI_Line = EXTI_Line8;
  	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;	
  	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
  	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  	EXTI_Init(&EXTI_InitStructure);	

  	NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;			
  	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;	
  	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;				
  	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;								
  	NVIC_Init(&NVIC_InitStructure); 

}
u16 DIAG_STAT, BARO, TEMP;
float gyroX, gyroY, gyroZ;
float acclX, acclY, acclZ; 
float magnX, magnY, magnZ; 
int ex_cnt=0;
void EXTI9_5_IRQHandler(void)
{
	if(EXTI_GetITStatus(EXTI_Line8) == SET)
	{
		GPIO_ResetBits(GPIOB, GPIO_Pin_12);	

		SPI1_ReadWriteByte(0x3E);
		SPI1_ReadWriteByte(0x00);
		
		DIAG_STAT = ADIS16448_READ(0x56);
		
		gyroX = ADIS16448_GYRO(0x56);			//陀螺仪速度
		gyroY = ADIS16448_GYRO(0x56);
		gyroZ = ADIS16448_GYRO(0x56);
		
		acclX = ADIS16448_ACCL(0x56);			//加速度计
		acclY = ADIS16448_ACCL(0x56);
		acclZ = ADIS16448_ACCL(0x56);
		
		magnX = ADIS16448_MAGN(0x56);			//磁力计
		magnY = ADIS16448_MAGN(0x56);
		magnZ = ADIS16448_MAGN(0x56);
		
		BARO = ADIS16448_READ(0x56);
		TEMP = ADIS16448_READ(0x56);
//		
		GPIO_SetBits(GPIOB, GPIO_Pin_12);
		ex_cnt++;
		
		AHRSupdate(gyroX, gyroY, gyroZ, acclX, acclY, acclZ, magnX, magnY, magnZ);
		
		EXTI_ClearITPendingBit(EXTI_Line8);
	}		
	
}


void ADIS_Init(void)
{
	delay_ms_2(10);
	ADIS16448_WRITE(0X36, 0X0201);
	delay_ms_2(10);
	ADIS16448_WRITE(0X38, 0X0402);
	delay_ms_2(10);
	ADIS16448_WRITE(0X1E, 0XFFDA);
	delay_ms_2(10);
}




















