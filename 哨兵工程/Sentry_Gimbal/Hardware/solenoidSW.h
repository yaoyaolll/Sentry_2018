#ifndef _SOLENOIDSW_H_
#define _SOLENOIDSW_H_

#define SOLENOIDSW_OFF GPIO_ResetBits(GPIOA, GPIO_Pin_6|GPIO_Pin_7)    //�ɿ���ſ��أ���ס�ӵ�
#define SOLENOIDSW_ON  GPIO_SetBits(GPIOA, GPIO_Pin_6|GPIO_Pin_7)      //������ſ��أ��ӵ�����

void SolenoidSW_Configuration(void);

#endif
