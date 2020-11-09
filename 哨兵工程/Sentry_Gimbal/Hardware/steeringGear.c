#include "main.h"

//5V¶æ»ú½Ó¿Ú PA6¡¢PA7 TIM3_CH1 TIM3_CH2
//Ä¦²ÁÂÖ PB0 PB1  TIM3_CH3 TIM3_CH4
void SteeringGear_Configuration(void)
{
	GPIO_InitTypeDef          gpio;
	TIM_TimeBaseInitTypeDef   tim;
	TIM_OCInitTypeDef         oc;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA |RCC_APB2Periph_GPIOB| RCC_APB2Periph_AFIO, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE); 

	gpio.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
	gpio.GPIO_Mode = GPIO_Mode_AF_PP;
	gpio.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA,&gpio);
	
	gpio.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
	GPIO_Init(GPIOB,&gpio);
	
//	GPIO_PinRemapConfig(GPIO_FullRemap_TIM2, ENABLE);   
	
	tim.TIM_Prescaler = 7200-1;
	tim.TIM_CounterMode = TIM_CounterMode_Up;
	tim.TIM_Period = 200-1;         //0.1ms
	tim.TIM_ClockDivision = TIM_CKD_DIV1;
	
	TIM_TimeBaseInit(TIM3,&tim);

	oc.TIM_OCMode = TIM_OCMode_PWM2;
	oc.TIM_OutputState = TIM_OutputState_Enable;
	oc.TIM_Pulse = 15;
	oc.TIM_OCPolarity = TIM_OCPolarity_Low;
	oc.TIM_OCIdleState = TIM_OCIdleState_Reset;
	oc.TIM_OCNIdleState = TIM_OCNIdleState_Set;
	
	TIM_OC1Init(TIM3,&oc);
	TIM_OC2Init(TIM3,&oc);
	TIM_OC3Init(TIM3,&oc);
	TIM_OC4Init(TIM3,&oc);
	TIM_OC1PreloadConfig(TIM3,TIM_OCPreload_Enable);
	TIM_OC2PreloadConfig(TIM3,TIM_OCPreload_Enable);
	TIM_OC3PreloadConfig(TIM3,TIM_OCPreload_Enable);
	TIM_OC4PreloadConfig(TIM3,TIM_OCPreload_Enable);
	
	TIM_ARRPreloadConfig(TIM3,ENABLE);
	TIM_Cmd(TIM3,ENABLE);
}

void SteeringGear_Set1(int position)       
{
	TIM3->CCR1 = position;
}
void SteeringGear_Set2(int position)       
{
	TIM3->CCR2 = position;
}

//void FrictionWheel_Set(int speed)
//{
//	TIM_SetCompare3(TIM3,speed);
//	TIM_SetCompare4(TIM3,speed);
//}
