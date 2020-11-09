#include "main.h"

//PB10-USART3_TX PB11-USART_RX ����ϵͳ

unsigned char JudgeReceiveBuffer[JudgeBufBiggestSize];
unsigned char JudgeSendBuffer[JudgeSendBufSize];
 
void USART3_Configuration(void)
{
	USART_InitTypeDef usart;
	GPIO_InitTypeDef  gpio;
	NVIC_InitTypeDef  nvic;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3,ENABLE);
	
	gpio.GPIO_Pin = GPIO_Pin_11;
	gpio.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOB,&gpio);

	gpio.GPIO_Pin = GPIO_Pin_10;  
	gpio.GPIO_Mode = GPIO_Mode_AF_PP;
	gpio.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB,&gpio);

	USART_DeInit(USART3);
	usart.USART_BaudRate = 115200;
	usart.USART_WordLength = USART_WordLength_8b;
	usart.USART_StopBits = USART_StopBits_1;
	usart.USART_Parity = USART_Parity_No ;
	usart.USART_Mode = USART_Mode_Rx;
	usart.USART_HardwareFlowControl = USART_HardwareFlowControl_None;   
	USART_Init(USART3,&usart);
	 
	USART_ITConfig(USART3, USART_IT_IDLE, ENABLE);   //����������ʱ�ж� 
	USART_Cmd(USART3,ENABLE);
	USART_DMACmd(USART3, USART_DMAReq_Rx, ENABLE); 

	nvic.NVIC_IRQChannel = USART3_IRQn;
	nvic.NVIC_IRQChannelPreemptionPriority = 0;
	nvic.NVIC_IRQChannelSubPriority = 3;
	nvic.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&nvic);
		
	//����dma
	{
		DMA_InitTypeDef  dma;
		
		RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
		
		DMA_DeInit(DMA1_Channel3);
		
		dma.DMA_PeripheralBaseAddr = (uint32_t)&(USART3->DR);
		dma.DMA_MemoryBaseAddr = (uint32_t)JudgeReceiveBuffer;
		dma.DMA_DIR = DMA_DIR_PeripheralSRC;
		dma.DMA_BufferSize = JudgeBufBiggestSize;
		dma.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
		dma.DMA_MemoryInc = DMA_MemoryInc_Enable;
		dma.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
		dma.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
		dma.DMA_Mode = DMA_Mode_Circular;
		dma.DMA_Priority = DMA_Priority_VeryHigh;
		dma.DMA_M2M = DMA_M2M_Disable;
		
		DMA_Init(DMA1_Channel3, &dma);
		DMA_ITConfig(DMA1_Channel3,DMA_IT_TC,ENABLE);
		DMA_Cmd(DMA1_Channel3, ENABLE);
		
		nvic.NVIC_IRQChannel = DMA1_Channel3_IRQn;
		nvic.NVIC_IRQChannelPreemptionPriority = 0;
		nvic.NVIC_IRQChannelSubPriority = 2;
		nvic.NVIC_IRQChannelCmd = ENABLE;
		NVIC_Init(&nvic);
	}
}

/*����DMA*/
void DMA1_Channel3_IRQHandler(void)
{
	if(DMA_GetFlagStatus(DMA1_FLAG_TC3) == SET)
	{
		judge_rec_task(JudgeReceiveBuffer);
		DMA_ClearFlag(DMA1_FLAG_TC3);
	}
}

void USART3_IRQHandler(void)
{
	if (USART_GetITStatus(USART3, USART_IT_IDLE) != RESET)
	{
    (void)USART3->SR;   //clear the IDLE int
		(void)USART3->DR;	
  }
}
