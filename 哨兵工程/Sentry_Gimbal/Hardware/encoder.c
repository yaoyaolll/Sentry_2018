#include "main.h"

/*PB4/ENC_TIM3_CH1
**PB5/ENC_TIM3_CH2
*/

void ENCODER_Configuration(void)
{
	GPIO_InitTypeDef GPIO_Initure;
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);
	
	GPIO_Initure.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5;
	GPIO_Initure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_Initure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_Initure);
	
	GPIO_PinRemapConfig(GPIO_PartialRemap_TIM3, ENABLE);   //²¿·ÖÖØÓ³Éä
	
	TIM_EncoderInterfaceConfig(TIM3, TIM_EncoderMode_TI2, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);
	
	TIM3->CNT = 0x7fff;
	TIM_Cmd(TIM3, ENABLE);
}

short Encoder_read(void)
{
	short count = 0;
	
	count = TIM3->CNT - 0x7fff;
	TIM3->CNT = 0x7fff;

	return count;
}

