#include "main.h"

//PA15/S_TIM2_CH1
//PB3/S_TIM2_CH2

//PB8/S_TIM4_CH3
//PB9/S_TIM4_CH4
void FrictionWheel_Configuration(void)    //72MHz
{
	GPIO_InitTypeDef          gpio;
	TIM_TimeBaseInitTypeDef   tim;
	TIM_OCInitTypeDef         oc;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2 | RCC_APB1Periph_TIM4, ENABLE); 

//	gpio.GPIO_Pin = GPIO_Pin_15;
//	GPIO_Init(GPIOA,&gpio);
	
	gpio.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9;
	gpio.GPIO_Mode = GPIO_Mode_AF_PP;
	gpio.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB,&gpio);
	
//	GPIO_PinRemapConfig(GPIO_FullRemap_TIM2, ENABLE);   
	
	tim.TIM_Prescaler = 23;
	tim.TIM_CounterMode = TIM_CounterMode_Up;
	tim.TIM_Period = 60000;
	tim.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseInit(TIM2,&tim);
	
	TIM_TimeBaseInit(TIM4,&tim);

	oc.TIM_OCMode = TIM_OCMode_PWM2;
	oc.TIM_OutputState = TIM_OutputState_Enable;
	oc.TIM_Pulse = 3000;
	oc.TIM_OCPolarity = TIM_OCPolarity_Low;
	oc.TIM_OCIdleState = TIM_OCIdleState_Reset;
	oc.TIM_OCNIdleState = TIM_OCNIdleState_Set;
	TIM_OC1Init(TIM2,&oc);
	TIM_OC2Init(TIM2,&oc);
	TIM_OC1PreloadConfig(TIM2,TIM_OCPreload_Enable);
	TIM_OC2PreloadConfig(TIM2,TIM_OCPreload_Enable);
	
	TIM_OC3Init(TIM4,&oc);
	TIM_OC4Init(TIM4,&oc);
	TIM_OC3PreloadConfig(TIM4,TIM_OCPreload_Enable);
	TIM_OC4PreloadConfig(TIM4,TIM_OCPreload_Enable);
		
	TIM_ARRPreloadConfig(TIM2,ENABLE);
	TIM_CtrlPWMOutputs(TIM2,ENABLE);
	TIM_Cmd(TIM2,ENABLE);
	
	TIM_ARRPreloadConfig(TIM4,ENABLE);
	TIM_CtrlPWMOutputs(TIM4,ENABLE);
	TIM_Cmd(TIM4,ENABLE);
}

void FrictionWheel_Set(int speed)
{
	TIM_SetCompare3(TIM4,speed);
	TIM_SetCompare4(TIM4,speed);
}

//void FrictionWheel_
