#include "main.h"

//WiFi
//PA10-USART1_RX
//PA9-USART1_TX

unsigned char wifi_rx[wifi_buffer_size];

void USART1_Configuration(void)
{
	USART_InitTypeDef usart;
	GPIO_InitTypeDef  gpio;
	NVIC_InitTypeDef  nvic;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);
	
	gpio.GPIO_Pin = GPIO_Pin_10;
	gpio.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOA, &gpio);

	gpio.GPIO_Pin = GPIO_Pin_9;  
	gpio.GPIO_Mode = GPIO_Mode_AF_PP;
	gpio.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &gpio);
	
	gpio.GPIO_Pin = GPIO_Pin_8;
	gpio.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOA, &gpio);
	GPIO_SetBits(GPIOA, GPIO_Pin_8);

	USART_DeInit(USART1);
	usart.USART_BaudRate = 115200;
	usart.USART_WordLength = USART_WordLength_8b;
	usart.USART_StopBits = USART_StopBits_1;
	usart.USART_Parity = USART_Parity_No ;
	usart.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	usart.USART_HardwareFlowControl = USART_HardwareFlowControl_None;   
	USART_Init(USART1,&usart);
	
	USART_ITConfig(USART1, USART_IT_IDLE, ENABLE);   //开启串口闲时中断  
	USART_Cmd(USART1,ENABLE);
	USART_DMACmd(USART1, USART_DMAReq_Rx, ENABLE); 
	
	nvic.NVIC_IRQChannel = USART1_IRQn;
	nvic.NVIC_IRQChannelPreemptionPriority = 1;
	nvic.NVIC_IRQChannelSubPriority = 2;
	nvic.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&nvic);
  //接收dma
	{
		DMA_InitTypeDef  dma;
		
		RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
		
		DMA_DeInit(DMA1_Channel5);
		
		dma.DMA_PeripheralBaseAddr = (uint32_t)&(USART1->DR);
		dma.DMA_MemoryBaseAddr = (uint32_t)wifi_rx;
		dma.DMA_DIR = DMA_DIR_PeripheralSRC;
		dma.DMA_BufferSize = wifi_buffer_size;
		dma.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
		dma.DMA_MemoryInc = DMA_MemoryInc_Enable;
		dma.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
		dma.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
		dma.DMA_Mode = DMA_Mode_Circular;
		dma.DMA_Priority = DMA_Priority_VeryHigh;
		dma.DMA_M2M = DMA_M2M_Disable;
		
		DMA_Init(DMA1_Channel5, &dma);
		DMA_ITConfig(DMA1_Channel5,DMA_IT_TC,ENABLE);
		DMA_Cmd(DMA1_Channel5, ENABLE);
		
		nvic.NVIC_IRQChannel = DMA1_Channel5_IRQn;
		nvic.NVIC_IRQChannelPreemptionPriority = 1;
		nvic.NVIC_IRQChannelSubPriority = 3;
		nvic.NVIC_IRQChannelCmd = ENABLE;
		NVIC_Init(&nvic);
	}
}


u8 temp[wifi_buffer_size], wifi_temp[wifi_buffer_size*2];
void USART1_IRQHandler(void)
{   
	if (USART_GetITStatus(USART1, USART_IT_IDLE) != RESET)
	{
		int	n, i; 
		memcpy(wifi_temp + wifi_buffer_size, wifi_rx, wifi_buffer_size);
		for(n=0;n<wifi_buffer_size;n++)
		{
			if((wifi_temp[n] == '!') && (wifi_temp[n + wifi_buffer_size - 1]=='#'))
			{
				for(i=0;i<wifi_buffer_size;i++)
				{
					temp[i] = wifi_temp[n+i];
				}
				if(Verify_CRC8_Check_Sum(temp, wifi_buffer_size-1))
				{
					wifi_rec_task(temp);
				}
				break;
			}
		}
		memcpy(wifi_temp, wifi_temp + wifi_buffer_size, wifi_buffer_size);
		(void)USART1->SR;   //clear the IDLE int
		(void)USART1->DR;	
	}	
}

//DMA完成中断
void DMA1_Channel5_IRQHandler(void)
{
	if(DMA_GetFlagStatus(DMA1_FLAG_TC5) == SET)
	{
		DMA_ClearFlag(DMA1_FLAG_TC5);
	}
}

