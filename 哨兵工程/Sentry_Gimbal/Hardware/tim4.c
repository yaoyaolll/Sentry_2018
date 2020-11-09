#include "main.h"

/**
  * @brief  tim4 config
  * @param  None
  * @retval None
  */
void Tim4_Configuration(void)
{
	NVIC_InitTypeDef NVIC_InitStructure; 
	TIM_TimeBaseInitTypeDef   tim;
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
	tim.TIM_Prescaler = 4-1;//21MHz
	tim.TIM_CounterMode = TIM_CounterMode_Up;
	tim.TIM_Period = 42-1;//2us
	tim.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseInit(TIM4,&tim);
	
	NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;	  
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;	
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	TIM_ClearFlag(TIM4, TIM_FLAG_Update);
	TIM_ITConfig(TIM4,TIM_IT_Update,ENABLE);
}
/**
  * @brief  
  * @param  
  * @retval None
  */
extern uint32_t SRC_Buffer[];
uint16_t accelerator;
uint8_t BIT[16];
int _i;
extern short Shoot_Flag;
void TIM4_IRQHandler(void)
{
	if ( TIM_GetITStatus(TIM4 , TIM_IT_Update) != RESET ) 
	{
		TIM_Cmd(TIM4, DISABLE);
		TIM_ITConfig(TIM4,TIM_IT_Update,DISABLE);
		accelerator=LIMIT_MAX_MIN(accelerator,2000,0);//0~2048  速度太快需限幅
		BIT[0]   = (accelerator&0x400 ? 1 : 0);
		BIT[1]   = (accelerator&0x200 ? 1 : 0);
		BIT[2]   = (accelerator&0x100 ? 1 : 0);
		BIT[3]   = (accelerator&0x80  ? 1 : 0);
		BIT[4]   = (accelerator&0x40  ? 1 : 0);
		BIT[5]   = (accelerator&0x20  ? 1 : 0);
		BIT[6]   = (accelerator&0x10  ? 1 : 0);
		BIT[7]   = (accelerator&0x8   ? 1 : 0);
		BIT[8]   = (accelerator&0x4   ? 1 : 0);
		BIT[9]   = (accelerator&0x2   ? 1 : 0);
		BIT[10]  = (accelerator&0x1   ? 1 : 0);
		BIT[11] =0;
		//getCRC
		BIT[12] = BIT[0]^BIT[4]^BIT[8];
		BIT[13] = BIT[1]^BIT[5]^BIT[9];
		BIT[14] = BIT[2]^BIT[6]^BIT[10];
		BIT[15] = BIT[3]^BIT[7]^BIT[11];
		for(_i=0;_i<16;_i++)
		{
			SRC_Buffer[2*_i]=(BIT[_i]==1)?23:12;
			SRC_Buffer[2*_i+1]=(BIT[_i]==1)?23:12;
		}
		DMA_ClearFlag(DMA1_Stream7, DMA_FLAG_TCIF7);
		TIM_DMACmd(TIM2, TIM_DMA_Update, ENABLE);
		DMA_ITConfig(DMA1_Stream7,DMA_IT_TC,ENABLE);
		DMA_Cmd(DMA1_Stream7, ENABLE);
		TIM_ClearITPendingBit(TIM4 , TIM_FLAG_Update);    
	}	
}
