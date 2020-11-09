#ifndef _CONTROL_TASK_H_
#define _CONTROL_TASK_H_

//#define TURN_CHASSIS_DIR_TEST 
#define TURN_CHASSIS_DIR_FIGHT 


#ifdef TURN_CHASSIS_DIR_TEST
#define all_encoder_point1 -200
#define all_encoder_point2 -400
#define all_encoder_point3 -3800
#define all_encoder_point4 -4000
#define all_encoder_end    -4200
#endif


#ifdef  TURN_CHASSIS_DIR_FIGHT
//全程
#define all_encoder_point1 -1000
#define all_encoder_point2 -1500
//#define all_encoder_point3 -49500
//#define all_encoder_point4 -50000
#define all_encoder_point3 -48500
#define all_encoder_point4 -49000
#define all_encoder_end    -52000
//第一段
#define first_encoder_point1 -1000
#define first_encoder_point2 -1500
#define first_encoder_point3 -15000
#define first_encoder_point4 -15500
//第二段
#define second_encoder_point1 -15000
#define second_encoder_point2 -15500
#define second_encoder_point3 -32000
#define second_encoder_point4 -32500
//第三段
#define third_encoder_point1 -32000
#define third_encoder_point2 -32500
#define third_encoder_point3 -49500
#define third_encoder_point4 -50000
#endif

#define patrol_state 0
#define shoot_state  1
#define dodge_state  2

typedef struct 
{
	float last_position;
	float cur_position;
	float last_relative_pos;
	float cur_relative_pos;
	float last_speed;
	float cur_speed;	
}target_para_t;

void gimbal_kalman_para_init(void);
void target_para_cal(void);
void target_pos_speed_cal(target_para_t *I, int pc_time_interval, int flag);
void pc_handler(void);
void user_mode(int time);
void sleep_mode(int time);
void auto_mode(int time);
void disconnect_handler(void);
int reduce_HP(void);
void other2patrol_state(void);
void other2dodge_state(void);
void other2shoot_state(void);
void HP_handler(void);
void turn_chassis_dir(unsigned char flag);
int heat_limit_task(void);

#endif
