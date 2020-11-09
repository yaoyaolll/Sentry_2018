#include "main.h"

//PC_CHASSIS
//PA2-USART2_TX PA3-USART2_RX 

unsigned char pc_chassis_rx[pc_chassis_buffer_size]; 
unsigned char pc_chassis_tx[pc_chassis_buffer_size];

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
	usart.USART_StopBits = USART_StopBits_2;
	usart.USART_Parity = USART_Parity_No ;
	usart.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	usart.USART_HardwareFlowControl = USART_HardwareFlowControl_None;   
	USART_Init(USART2,&usart);
	
	USART_ITConfig(USART2, USART_IT_IDLE, ENABLE);   //开启串口闲时中断  
	USART_Cmd(USART2,ENABLE);
	USART_DMACmd(USART2, USART_DMAReq_Rx|USART_DMAReq_Tx, ENABLE); 
	
	nvic.NVIC_IRQChannel = USART2_IRQn;
	nvic.NVIC_IRQChannelPreemptionPriority = 1;
	nvic.NVIC_IRQChannelSubPriority = 0;
	nvic.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&nvic);
	//接收dma
	{
		DMA_InitTypeDef  dma;
		
		RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
		
		DMA_DeInit(DMA1_Channel6);
		
		dma.DMA_PeripheralBaseAddr = (uint32_t)&(USART2->DR);
		dma.DMA_MemoryBaseAddr = (uint32_t)pc_chassis_rx;
		dma.DMA_DIR = DMA_DIR_PeripheralSRC;
		dma.DMA_BufferSize = pc_chassis_buffer_size;
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
	nvic.NVIC_IRQChannelPreemptionPriority = 1;
	nvic.NVIC_IRQChannelSubPriority = 1;
	nvic.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&nvic);
	//发送dma
	{
		DMA_InitTypeDef  dma;
		
		RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
		
		DMA_DeInit(DMA1_Channel7);
		
		dma.DMA_PeripheralBaseAddr = (uint32_t)&(USART2->DR);
		dma.DMA_MemoryBaseAddr = (uint32_t)pc_chassis_tx;
		dma.DMA_DIR = DMA_DIR_PeripheralDST;
		dma.DMA_BufferSize = pc_chassis_buffer_size;
		dma.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
		dma.DMA_MemoryInc = DMA_MemoryInc_Enable;
		dma.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
		dma.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
		dma.DMA_Mode = DMA_Mode_Normal;
		dma.DMA_Priority = DMA_Priority_VeryHigh;
		dma.DMA_M2M = DMA_M2M_Disable;
		
		DMA_ITConfig(DMA1_Channel7,DMA_IT_TC,ENABLE);
		DMA_Init(DMA1_Channel7, &dma);
		DMA_Cmd(DMA1_Channel7, DISABLE);
	}
}

//接收中断
unsigned char temp_chassis[pc_chassis_buffer_size], temp_chassis_pc[pc_chassis_buffer_size*2];
void USART2_IRQHandler(void)
{
	short i, n;
	if (USART_GetITStatus(USART2, USART_IT_IDLE) != RESET)
	{
		memcpy(temp_chassis_pc + pc_chassis_buffer_size, pc_chassis_rx, pc_chassis_buffer_size);
		for(n=0;n<pc_chassis_buffer_size;n++)
		{
			if(temp_chassis_pc[n] == '!')
			{
				for(i=0;i<pc_chassis_buffer_size;i++)
				{
					temp_chassis[i] = temp_chassis_pc[n+i];
				}
				if(Verify_CRC16_Check_Sum(temp_chassis, pc_chassis_buffer_size))
					pc_chassis2pc_gimbal_rec_task(temp_chassis); 
				break;
			}
		}
		memcpy(temp_chassis_pc, temp_chassis_pc + pc_chassis_buffer_size, pc_chassis_buffer_size);
    (void)USART2->SR;   //clear the IDLE int
		(void)USART2->DR;	
  }
}

int rx;
void DMA1_Channel6_IRQHandler(void)
{
	if(DMA_GetFlagStatus(DMA1_FLAG_TC6) == SET)
	{
		rx ++;
		DMA_ClearFlag(DMA1_FLAG_TC6);
	}
}

//发送完成中断
void DMA1_Channel7_IRQHandler(void)
{
	if(DMA_GetFlagStatus(DMA1_FLAG_TC7) == SET)
	{
		DMA_Cmd(DMA1_Channel7, DISABLE);
		DMA_ClearFlag(DMA1_FLAG_TC7);
	}
}

