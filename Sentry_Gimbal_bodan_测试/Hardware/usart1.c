#include "main.h"

//TX2通信 中间红线是RX
//PA10-USART1_RX
//PA9-USART1_TX

unsigned char pc_gimbal_rx[pc_gimbal_buffer_size];
unsigned char pc_gimbal_tx[pc_gimbal_buffer_size];

void USART1_Configuration(void)
{
	USART_InitTypeDef usart;
	GPIO_InitTypeDef  gpio;
	NVIC_InitTypeDef  nvic;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);

	gpio.GPIO_Pin = GPIO_Pin_10;
	gpio.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOA,&gpio);

	gpio.GPIO_Pin = GPIO_Pin_9;  
	gpio.GPIO_Mode = GPIO_Mode_AF_PP;
	gpio.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA,&gpio);

	USART_DeInit(USART1);
	usart.USART_BaudRate = 115200;
	usart.USART_WordLength = USART_WordLength_8b;
	usart.USART_StopBits = USART_StopBits_2;
	usart.USART_Parity = USART_Parity_No ;
	usart.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	usart.USART_HardwareFlowControl = USART_HardwareFlowControl_None;   
	USART_Init(USART1,&usart);
	
	USART_ITConfig(USART1, USART_IT_IDLE, ENABLE);   //开启串口闲时中断  
	USART_Cmd(USART1,ENABLE);
	USART_DMACmd(USART1, USART_DMAReq_Rx|USART_DMAReq_Tx, ENABLE); 
	
	nvic.NVIC_IRQChannel = USART1_IRQn;
	nvic.NVIC_IRQChannelPreemptionPriority = 0;
	nvic.NVIC_IRQChannelSubPriority = 0;
	nvic.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&nvic);
	//接收dma
	{
		DMA_InitTypeDef  dma;
		
		RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
		
		DMA_DeInit(DMA1_Channel5);
		
		dma.DMA_PeripheralBaseAddr = (uint32_t)&(USART1->DR);
		dma.DMA_MemoryBaseAddr = (uint32_t)pc_gimbal_rx;
		dma.DMA_DIR = DMA_DIR_PeripheralSRC;
		dma.DMA_BufferSize = pc_gimbal_buffer_size;
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
	}
	//发送dma
	{
		DMA_InitTypeDef  dma;
		
		RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
		
		DMA_DeInit(DMA1_Channel4);
		
		dma.DMA_PeripheralBaseAddr = (uint32_t)&(USART1->DR);
		dma.DMA_MemoryBaseAddr = (uint32_t)pc_gimbal_tx;
		dma.DMA_DIR = DMA_DIR_PeripheralDST;
		dma.DMA_BufferSize = pc_gimbal_buffer_size;
		dma.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
		dma.DMA_MemoryInc = DMA_MemoryInc_Enable;
		dma.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
		dma.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
		dma.DMA_Mode = DMA_Mode_Circular;
		dma.DMA_Priority = DMA_Priority_VeryHigh;
		dma.DMA_M2M = DMA_M2M_Disable;
		
		DMA_Init(DMA1_Channel4, &dma);
		DMA_ITConfig(DMA1_Channel4,DMA_IT_TC,ENABLE);
		DMA_Cmd(DMA1_Channel4, DISABLE);
		
		nvic.NVIC_IRQChannel = DMA1_Channel4_IRQn;
		nvic.NVIC_IRQChannelPreemptionPriority = 0;
		nvic.NVIC_IRQChannelSubPriority = 1;
		nvic.NVIC_IRQChannelCmd = ENABLE;
		NVIC_Init(&nvic);
	}
}

int usart1rx;
unsigned char temp[pc_gimbal_buffer_size], temp_gimbal_pc[pc_gimbal_buffer_size*2];
void USART1_IRQHandler(void)
{
	short i, n;
	if (USART_GetITStatus(USART1, USART_IT_IDLE) != RESET)
	{
		usart1rx ++;
		memcpy(temp_gimbal_pc + pc_gimbal_buffer_size, pc_gimbal_rx, pc_gimbal_buffer_size);
		for(n=0;n<pc_gimbal_buffer_size;n++)
		{
			if(temp_gimbal_pc[n] == '!')
			{
				for(i=0;i<pc_gimbal_buffer_size;i++)
				{
					temp[i] = temp_gimbal_pc[n+i];
				}
				if(Verify_CRC16_Check_Sum(temp, pc_gimbal_buffer_size))
					pc_gimbal2mcu_send_task(temp);
				break;
			}
		}
		memcpy(temp_gimbal_pc, temp_gimbal_pc + pc_gimbal_buffer_size, pc_gimbal_buffer_size);
    (void)USART1->SR;   //clear the IDLE int
		(void)USART1->DR;	
  }
}

int c4_tx;
void DMA1_Channel4_IRQHandler(void)
{
	if(DMA_GetFlagStatus(DMA1_FLAG_TC4) == SET)
	{
		c4_tx ++;
		DMA_Cmd(DMA1_Channel4, DISABLE);
		DMA_ClearFlag(DMA1_FLAG_TC4);
	}
}
