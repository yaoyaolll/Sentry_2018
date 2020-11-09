#include "main.h"

short tim2_update;

void TIM2Init(void)
{
    TIM_TimeBaseInitTypeDef tim2;
    NVIC_InitTypeDef NVIC_TIM;
    
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
    
    tim2.TIM_ClockDivision = TIM_CKD_DIV1;   
    tim2.TIM_Prescaler = 7200*5-1;          //10kHz 0.5ms   
    tim2.TIM_CounterMode = TIM_CounterMode_Up;
    tim2.TIM_RepetitionCounter = 0;
    tim2.TIM_Period = 9;                //5ms
	
		NVIC_TIM.NVIC_IRQChannel = TIM2_IRQn;
		NVIC_TIM.NVIC_IRQChannelCmd = ENABLE;
		NVIC_TIM.NVIC_IRQChannelPreemptionPriority = 2;
		NVIC_TIM.NVIC_IRQChannelSubPriority = 0;
		NVIC_Init(&NVIC_TIM);
			
    TIM_TimeBaseInit(TIM2, &tim2);
    TIM_ClearFlag(TIM2, TIM_FLAG_Update);
    TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE); 
    TIM_Cmd(TIM2, DISABLE);
}

void TIM2_IRQHandler(void)
{
    if(TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET)
    {
			tim2_update = 0;
			TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
			TIM_Cmd(TIM2, DISABLE);
    }
}

short time_long;
void rand_chassis_dir_generate(void)
{
	if(!tim2_update)
	{
		time_long = rand()%4 + 1;     //生成1到4之间的随机数
		tim2_update = 1;
		TIM2->ARR = time_long*2000 - 1;
		TIM2->CNT = 0;
		motor_setpoint.chassis_speed = -motor_setpoint.chassis_speed;   //反向
		TIM_Cmd(TIM2, ENABLE);
	}
}
