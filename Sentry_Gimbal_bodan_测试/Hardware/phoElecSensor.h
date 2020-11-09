#ifndef _PHOELECSENSOR_H_
#define _PHOELECSENSOR_H_

/* if photoelectric switcher detects targets, PE_SENSOR will return 0, else return 1.
*/
#define  PE_SENSOR1  GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_1)  //PB1
#define  PE_SENSOR2  GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_4)  //PA4
#define  PE_SENSOR3  GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_0)  //PA0
#define  PE_SENSOR4  GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_5)  //PA5
#define  PE_SENSOR5  GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_1)  //PA0
#define  PE_SENSOR6  GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_0)  //PA1

void phoElecSensor_Configuration(void);

#endif
