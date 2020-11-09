#include "main.h"

//PA2-USART2_TX PA3-USART2_RX 陀螺仪

unsigned char GYRO0_buffer[GYRO_BUF_SIZE]; 
//ort RX_USART6_BUFFER=70;
/**
  * @brief  陀螺仪数据传入
  * @param  None
  * @retval None
  */
void USART2_Configuration(void)
{
	USART_InitTypeDef usart;
	GPIO_InitTypeDef  gpio;
	NVIC_InitTypeDef  nvic;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,ENABLE);

	gpio.GPIO_Pin = GPIO_Pin_3;
	gpio.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOA,&gpio);

	gpio.GPIO_Pin = GPIO_Pin_2;  
	gpio.GPIO_Mode = GPIO_Mode_AF_PP;
	gpio.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA,&gpio);

	USART_DeInit(USART2);
	usart.USART_BaudRate = 115200;
	usart.USART_WordLength = USART_WordLength_8b;
	usart.USART_StopBits = USART_StopBits_1;
	usart.USART_Parity = USART_Parity_No ;
	usart.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	usart.USART_HardwareFlowControl = USART_HardwareFlowControl_None;   
	USART_Init(USART2,&usart);
	
	USART_ITConfig(USART2, USART_IT_IDLE, ENABLE);   //开启串口闲时中断  
	USART_Cmd(USART2,ENABLE);
	USART_DMACmd(USART2, USART_DMAReq_Rx, ENABLE); 
	
	nvic.NVIC_IRQChannel = USART2_IRQn;
	nvic.NVIC_IRQChannelPreemptionPriority = 0;
	nvic.NVIC_IRQChannelSubPriority = 2;
	nvic.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&nvic);
	
	{
		DMA_InitTypeDef  dma;
		
		RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
		
		DMA_DeInit(DMA1_Channel6);

		dma.DMA_PeripheralBaseAddr = (uint32_t)&(USART2->DR);
		dma.DMA_MemoryBaseAddr = (uint32_t)GYRO0_buffer;
		dma.DMA_DIR = DMA_DIR_PeripheralSRC;
		dma.DMA_BufferSize = GYRO_BUF_SIZE;
		dma.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
		dma.DMA_MemoryInc = DMA_MemoryInc_Enable;
		dma.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
		dma.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
		dma.DMA_Mode = DMA_Mode_Circular;
		dma.DMA_Priority = DMA_Priority_VeryHigh;
		dma.DMA_M2M = DMA_M2M_Disable;
		
		DMA_Init(DMA1_Channel6, &dma);
		DMA_ITConfig(DMA1_Channel6,DMA_IT_TC,ENABLE);
		DMA_Cmd(DMA1_Channel6, ENABLE);
	}
	nvic.NVIC_IRQChannel = DMA1_Channel6_IRQn;
	nvic.NVIC_IRQChannelPreemptionPriority = 0;
	nvic.NVIC_IRQChannelSubPriority = 3;
	nvic.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&nvic);
}


void DMA1_Channel6_IRQHandler(void)
{
	short i,n;
	unsigned char temp[GYRO_BUF_SIZE];
	if(DMA_GetFlagStatus(DMA1_FLAG_TC6) == SET)
	{
		DMA_ClearFlag(DMA1_FLAG_TC6);
		for(n=0;n<GYRO_BUF_SIZE;n++)
		{
			if(GYRO0_buffer[n] == 0x55)
			{
				for(i=0;i<GYRO_BUF_SIZE;i++)
				{
					temp[i]=GYRO0_buffer[(n+i)%GYRO_BUF_SIZE];
				}
				if(Verify_CRC8_Check_Sum(temp, GYRO_BUF_SIZE) == 1) 
				{
					GyroReceiveFunc(GYRO0_buffer,n);
				}
				break;
			}
		}
	}
}

void USART2_IRQHandler(void)
{
	if (USART_GetITStatus(USART2, USART_IT_IDLE) != RESET)
	{
		USART_ClearITPendingBit(USART2, USART_IT_IDLE);
  }
}

