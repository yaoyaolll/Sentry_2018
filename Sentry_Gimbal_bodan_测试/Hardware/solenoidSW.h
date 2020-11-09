#ifndef _SOLENOIDSW_H_
#define _SOLENOIDSW_H_

#define SOLENOIDSW_OFF GPIO_ResetBits(GPIOA, GPIO_Pin_6|GPIO_Pin_7)    //松开电磁开关，堵住子弹
#define SOLENOIDSW_ON  GPIO_SetBits(GPIOA, GPIO_Pin_6|GPIO_Pin_7)      //开启电磁开关，子弹落下

void SolenoidSW_Configuration(void);

#endif
