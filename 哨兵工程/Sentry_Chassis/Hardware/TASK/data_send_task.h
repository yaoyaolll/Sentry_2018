#ifndef _DATA_SEND_TASK_H_
#define _DATA_SEND_TASK_H_

//receive
#define motor_chassis_id1 0x201
#define motor_chassis_id2 0x202
#define motor_bodan_id    0x203
#define motor_yaw_id      0x206
#define motor_pitch_id    0x205
#define pc_gimbal2mcu_id  0x703
#define gyro_id           0x704
#define remote_id         0x705

//send
#define friction_wheel_id 0x701
#define pc_chassis2pc_gimbal_id    0x702
#define mcu2pc_gimbal_id  0x706

void _820r_send_task(short chassis_data1, short chassisdata2, short bodan_data);
void gimbal_send_task(int yaw, int pitch);
void friction_send_task(short speed);
void pc_chassis2pc_gimbal_send_task(void);
void mcu2pc_gimbal_send_task(void);


#endif
