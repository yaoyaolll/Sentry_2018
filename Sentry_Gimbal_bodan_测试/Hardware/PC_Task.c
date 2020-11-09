#include "main.h"
#include "string.h"

extern unsigned char pc_gimbal_tx[pc_gimbal_buffer_size];

/**
  * @brief  ��̨PC���ݽ���ת�������ذ�
  * @param  *str PC������������
  * @retval 
  */
int pc_gimbal2mcu_send_task_cnt;
void pc_gimbal2mcu_send_task(unsigned char *str)
{
	CanTxMsg tx_message;
	int i;
	GPIO_ResetBits(GPIOA, GPIO_Pin_5);                //PC���
	
	if(str[0] == '!' && Verify_CRC16_Check_Sum(str, 11) == 1)    
	{
		pc_gimbal2mcu_send_task_cnt ++;
		for(i = 0; i < 8; i ++)
			tx_message.Data[i] = str[i + 1];
		
		tx_message.IDE = CAN_ID_STD;    
		tx_message.RTR = CAN_RTR_DATA; 
		tx_message.DLC = 0x08;    
		tx_message.StdId = 0x703;    //�����ذ巢��PC���յ�������
		
		CAN_Transmit(CAN1,&tx_message);
	}
}

/**
  * @brief  ����PC����ת������̨PC
  * @param  *str PC������������
  * @retval 
  */
void pc_chassis2pc_gimbal_send_task(unsigned char *str)
{
	int i;
	pc_gimbal_tx[0] = '!';
	for(i=0;i<pc_gimbal_buffer_size;i++)
		pc_gimbal_tx[1+i] = str[i];
	Append_CRC16_Check_Sum(pc_gimbal_tx, pc_gimbal_buffer_size);
	
	DMA_Cmd(DMA1_Channel4, ENABLE);
}
