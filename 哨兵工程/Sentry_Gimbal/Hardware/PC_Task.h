#ifndef _PC_TASK_H_
#define _PC_TASK_H_

#define CAN_PC_ID 0x333

void pc_gimbal2mcu_send_task(unsigned char *str);
void pc_chassis2pc_gimbal_send_task(unsigned char *str);

#endif
