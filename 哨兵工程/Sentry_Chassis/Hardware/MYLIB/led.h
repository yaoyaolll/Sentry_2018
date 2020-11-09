#ifndef _LED_H_
#define _LED_H_

#define LED_PC_ON  GPIO_ResetBits(GPIOA, GPIO_Pin_15)
#define LED_GYRO_ON GPIO_ResetBits(GPIOB, GPIO_Pin_3)
#define LED_PC_OFF  GPIO_SetBits(GPIOA, GPIO_Pin_15)
#define LED_GYRO_OFF GPIO_SetBits(GPIOB, GPIO_Pin_3)

void LED_Configuration(void);

#endif
