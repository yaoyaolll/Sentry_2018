#include "main.h"

//PB4 PB5

unsigned char limit_sw_value[2];   //行程开关状态值

void LimitSW_Configuration(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4|GPIO_Pin_5;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
}


//limitSWValue[0]为背离电源一端行程开关，limitSWValue[1]为电源一端行程开关
void limit_sw_read(void)
{
	limit_sw_value[0] = GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_4);  //读取行程开关输入值
	limit_sw_value[1] = GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_5);
}

unsigned char* get_sw_value(void)
{
	return limit_sw_value;
}

