//#include "main.h"

////摩擦轮 PB0 PB1  TIM3_CH3 TIM3_CH4
///*sanil电调控制摩擦轮，速度较高时，需要慢慢开启，否则会报警*/

//void FrictionWheel_Configuration(void)
//{
//	GPIO_InitTypeDef          gpio;
//	TIM_TimeBaseInitTypeDef   tim;
//	TIM_OCInitTypeDef         oc;
//	
//	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB| RCC_APB2Periph_AFIO, ENABLE);
//	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE); 

//	gpio.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
//	gpio.GPIO_Mode = GPIO_Mode_AF_PP;
//	gpio.GPIO_Speed = GPIO_Speed_50MHz;
//	GPIO_Init(GPIOB,&gpio);  
//	
//	tim.TIM_Prescaler = 36-1;
//	tim.TIM_CounterMode = TIM_CounterMode_Up;
//	tim.TIM_Period = 2000-1;   //1000Hz         
//	tim.TIM_ClockDivision = TIM_CKD_DIV1;
//	
//	TIM_TimeBaseInit(TIM3,&tim);

//	oc.TIM_OCMode = TIM_OCMode_PWM2;
//	oc.TIM_OutputState = TIM_OutputState_Enable;
//	oc.TIM_Pulse = 250;//250~500
//	oc.TIM_OCPolarity = TIM_OCPolarity_Low;
//	oc.TIM_OCIdleState = TIM_OCIdleState_Reset;
//	oc.TIM_OCNIdleState = TIM_OCNIdleState_Set;
//	
//	TIM_OC3Init(TIM3,&oc);
//	TIM_OC4Init(TIM3,&oc);
//	TIM_OC3PreloadConfig(TIM3,TIM_OCPreload_Enable);
//	TIM_OC4PreloadConfig(TIM3,TIM_OCPreload_Enable);
//	
//	TIM_ARRPreloadConfig(TIM3,ENABLE);
//	TIM_Cmd(TIM3,ENABLE);
//}

//void FrictionWheel_Set(int speed)
//{
//	TIM_SetCompare3(TIM3,speed);
//	TIM_SetCompare4(TIM3,speed);
//}


#include "main.h"

/**
  * @brief  ???????
  * @param  None
  * @retval None
  */
void FrictionWheel_Configuration(void)             
{
	GPIO_InitTypeDef          gpio;
	TIM_TimeBaseInitTypeDef   tim;
	TIM_OCInitTypeDef         oc;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB| RCC_APB2Periph_AFIO, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE); 
	
	gpio.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
	gpio.GPIO_Mode = GPIO_Mode_AF_PP;
	gpio.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB,&gpio); 

	tim.TIM_Prescaler = 36-1;//2Mhz
	tim.TIM_CounterMode = TIM_CounterMode_Up;
//    tim.TIM_Period = 4000-1;//500hz		pwm
//		tim.TIM_Period = 2000-1;//1khz		oneshot125   //???1233
		tim.TIM_Period = 2000-1;//1khz		oneshot42
//	tim.TIM_Period = 1233-1;//1khz		mutishot
	tim.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseInit(TIM3,&tim);

	oc.TIM_OCMode = TIM_OCMode_PWM2;
	oc.TIM_OutputState = TIM_OutputState_Enable;
	oc.TIM_OutputNState = TIM_OutputNState_Enable;
//		oc.TIM_Pulse = 2000;//pwm(1000~2000us)
//		oc.TIM_Pulse = 250;//oneshot125(125~250us)
		oc.TIM_Pulse = 84;//oneshot42(42~84us)      ok
//	oc.TIM_Pulse = 10;//multishot(5~25us)
	oc.TIM_OCPolarity = TIM_OCPolarity_Low;
	oc.TIM_OCNPolarity = TIM_OCNPolarity_High;
	oc.TIM_OCIdleState = TIM_OCIdleState_Reset;
	oc.TIM_OCNIdleState = TIM_OCNIdleState_Set;
	TIM_OC3Init(TIM3,&oc);
	TIM_OC4Init(TIM3,&oc);
	TIM_OC3PreloadConfig(TIM3,TIM_OCPreload_Enable);
	TIM_OC4PreloadConfig(TIM3,TIM_OCPreload_Enable);
		
	TIM_ARRPreloadConfig(TIM3,ENABLE);
	TIM_Cmd(TIM3,ENABLE);
}


/**
  * @brief  ?????????
  * @param  speed:1024-2048
  * @retval None
  */
void FrictionWheel_Set(int speed)
{
	speed = LIMIT_MAX_MIN(speed, 168, 84);
	TIM_SetCompare3(TIM3,speed);
	TIM_SetCompare4(TIM3,speed);
}
