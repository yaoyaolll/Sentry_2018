#include "main.h" 

float lg11 = 1.0413927f;
target_para_t target_pitch_para, target_yaw_para;
kalman_filter_t yaw_kalman_filter;
kalman_filter_t pitch_kalman_filter;
float filtered_pitch_degree;
float filtered_yaw_degree;
float filtered_pitch_speed;
float filtered_yaw_speed;
float speed_threshold = 20.0f;
float degree_threshold;
int last_sys_cnt;
int pc_time_interval;
unsigned char pc_disconnect_cnt;
int bodan_new_state;
int pitch_current_bias = 750;
float kalman_k;

extern unsigned char is_gyro_connect;
extern unsigned char is_pc_chassis_connect;
extern unsigned char wifi_choose_flag;

/* kalman param */
kalman_filter_init_t yaw_kalman_filter_para = {
  .P_data = {2, 0, 0, 2},
  .A_data = {1, 0.025, 0, 1},  //20ms
  .H_data = {1, 0, 0, 1},
  .Q_data = {1, 0, 0, 1},
  .R_data = {50, 0, 0, 50}
};

kalman_filter_init_t pitch_kalman_filter_para = 
{
  .P_data = {2, 0, 0, 2},
  .A_data = {1, 0.025, 0, 1},
  .H_data = {1, 0, 0, 1},
  .Q_data = {1, 0, 0, 1},
  .R_data = {100, 0, 0, 100}
};

/*-----------------------卡尔曼滤波--------------*/
//速度恒定模型
void gimbal_kalman_para_init(void)
{
	kalman_filter_init(&pitch_kalman_filter, &pitch_kalman_filter_para);
	kalman_filter_init(&yaw_kalman_filter, &yaw_kalman_filter_para);
}	
	
void target_pos_speed_cal(target_para_t *I, int pc_time_interval, int flag)
{
	pc_t* pc_gimbal = get_pc_gimbal(); 
	
//	if((pc_gimbal->gimbal_act_mode == 1 || pc_gimbal->gimbal_act_mode == 2)) //处于射击模式，云台运动
//	{
		if(flag == 0)     //pitch
		{
			I->cur_relative_pos = pc_gimbal->relative_pitch_degree;     //relative position
			I->cur_position = PITCH + pc_gimbal->relative_pitch_degree; //absolute position
			I->cur_speed = (I->cur_position - I->last_position)/pc_time_interval*1000;   //target speed
			degree_threshold = 10.0f;   
		}
		else if(flag == 1)//yaw
		{
//			I->cur_relative_pos = pc_gimbal->relative_yaw_degree*91.02f;          //relative position
//			I->cur_position = YAW + pc_gimbal->relative_yaw_degree*91.02f;        //absolute position
//			I->cur_speed = (I->cur_position - I->last_position)/pc_time_interval*1000;   //target speed
//			degree_threshold = 10.0f*91.02f;
			I->cur_relative_pos = pc_gimbal->relative_yaw_degree;          //relative position
			I->cur_position = YAW/91.02f + pc_gimbal->relative_yaw_degree;        //absolute position
			I->cur_speed = (I->cur_position - I->last_position)/pc_time_interval*1000;   //target speed
			degree_threshold = 10.0f;
		}
//	}
//	else
//	{
//		if(flag == 0)     //pitch
//		{
//			I->cur_relative_pos = 0;                                    //relative position
//			I->cur_position = PITCH;                                    //absolute position
//			I->cur_speed = 0;                                           //target speed
//			degree_threshold = 10.0f;   
//		}
//		else if(flag == 1)//yaw
//		{
//			I->cur_relative_pos = 0;                                     //relative position
//			I->cur_position = YAW;                                       //absolute position
//			I->cur_speed = 0;                                            //target speed
//			degree_threshold = 10.0f*91.02f;
//		}
//	}

	//限制角度跳变
	if(I->cur_position - I->last_position > degree_threshold)
		I->cur_position = I->last_position + degree_threshold;
	else if(I->cur_position - I->last_position < -degree_threshold)
		I->cur_position = I->last_position - degree_threshold;
	//限制速度跳变
	if(I->cur_speed - I->last_speed > speed_threshold)
		I->cur_speed = I->last_speed + speed_threshold;
	else if(I->cur_speed - I->last_speed < -speed_threshold)
		I->cur_speed = I->last_speed - speed_threshold;
	
	I->last_relative_pos = I->cur_relative_pos;
	I->last_position = I->cur_position;
	I->last_speed = I->cur_speed;
}

void target_para_cal(void)
{
	pc_time_interval = get_sys_cnt() - last_sys_cnt;

	if(last_sys_cnt != get_sys_cnt())
	{
		target_pos_speed_cal(&target_pitch_para, pc_time_interval, 0);
		target_pos_speed_cal(&target_yaw_para, pc_time_interval, 1);
	}

	last_sys_cnt = get_sys_cnt();
}

unsigned char last_gimbal_act_mode;
void pc_handler(void)
{
	pc_t* pc_gimbal = get_pc_gimbal();  
	float *yaw_kf_result, *pitch_kf_result;   
	static int yaw_predict_flag;
	
	target_para_cal();
	
	//kalman filter process, get x(k)、v(k), predict the present state.
	pitch_kf_result = kalman_filter_calc(&pitch_kalman_filter, target_pitch_para.cur_position, target_pitch_para.cur_speed);
	yaw_kf_result	= kalman_filter_calc(&yaw_kalman_filter, target_yaw_para.cur_position,  target_yaw_para.cur_speed);                                         
	
	if(ABS(target_yaw_para.cur_relative_pos) <= 3.000001f)
		kalman_k = 0;
	else
		kalman_k = log10(ABS(target_yaw_para.cur_relative_pos) - 2.000001f)/lg11;
	kalman_k = LIMIT_MAX_MIN(kalman_k, 1.0f, 0);
	
	
	//predict the next state.
	//使用x(k+1)的估计值
//	if((pc_gimbal->gimbal_act_mode == 1 || pc_gimbal->gimbal_act_mode == 2) && last_gimbal_act_mode != 1 && last_gimbal_act_mode != 2)   //云台第一次进入射击模式
//		yaw_predict_flag = 0;                                       //不使用预测
//	else
//	{
//		if(yaw_predict_flag == 0 && ABS(target_yaw_para.cur_relative_pos) <= 3.0f)
//			yaw_predict_flag = 1;
//		if(yaw_predict_flag)
//			yaw_kf_result = kalman_filter_calc(&yaw_kalman_filter, yaw_kf_result[0] + pc_time_interval/1000*yaw_kf_result[1]*kalman_k, yaw_kf_result[1]*kalman_k);                            
//	}
	
	if((pc_gimbal->gimbal_act_mode == 1 || pc_gimbal->gimbal_act_mode == 2) && (ABS(target_yaw_para.cur_relative_pos) < 10.0f))
		yaw_kf_result = kalman_filter_calc(&yaw_kalman_filter, yaw_kf_result[0] + pc_time_interval/1000*yaw_kf_result[1]*kalman_k, yaw_kf_result[1]*kalman_k);       
	
	//output degree
	filtered_pitch_degree = pitch_kf_result[0];
	filtered_yaw_degree   = yaw_kf_result[0]*91.02f;
	filtered_pitch_speed = pitch_kf_result[1];
	filtered_yaw_speed = yaw_kf_result[1];
	last_gimbal_act_mode = pc_gimbal->gimbal_act_mode;
}


/*---------------------手动模式-----------------*/ 
void user_mode(int time)              //手动模式
{
	static int friction_time_cnt;
	remote_t* remote = get_remote();
	
	if(remote->last_s1 == 3)
	{
		motor_setpoint.yaw_pos = YAW;
		motor_setpoint.pitch_pos = 0.0f;
		friction_time_cnt = 0;
	}
	
	motor_setpoint.chassis_speed = (1024 - remote->rc.ch1) * 7.5;
	motor_setpoint.yaw_pos += (ABS(1024 - remote->rc.ch2)<10?0:(1024 - remote->rc.ch2))*0.05f;
//	motor_setpoint.yaw_speed = (1024 - remote->rc.ch2)/120.0f;
	motor_setpoint.pitch_pos += (ABS(1024 - remote->rc.ch3)<10?0:(1024 - remote->rc.ch3))*0.0005f;
	motor_setpoint.pitch_pos = LIMIT_MAX_MIN(motor_setpoint.pitch_pos, 35.0f, 3.0f);
	
	if(time == 2)   //向底盘PC发送自身数据
	{
//		mcu2pc_gimbal_send_task();
	}
	if(time == 3)   //向底盘PC发送云台PC数据
	{
		pc_chassis2pc_gimbal_send_task(); 
	}
	if(time == 4)   //摩擦轮
	{
		if(friction_time_cnt < 125)  //500ms
		{
			friction_time_cnt ++;
			friction_send_task(84);    //关闭摩擦轮
		}
		else
		{
			if((1024 - remote->rc.ch0) > 600)
				friction_send_task(112);
			else if((1024 - remote->rc.ch0) < -600)
				friction_send_task(84);  
		}
	}
	else if(time == 5) //电机
	{
		short current_pitch;
		short current_yaw;
		short current_bodan;
		short *current_chassis;
		
		if(remote->rc.s2 == 1)
		{
			motor_setpoint.bodan_flag = 1;
			motor_setpoint.bodan_mode = 1;
		}
		else if(remote->rc.s2 == 3)
		{
			motor_setpoint.bodan_flag = 0;
		}
		else if(remote->rc.s2 == 2)
		{
			motor_setpoint.bodan_flag = 1;
			motor_setpoint.bodan_mode = 3;
		}
	
		current_bodan = bodan_task(motor_setpoint.bodan_flag, motor_setpoint.bodan_mode);
		current_chassis = chassis_task(motor_setpoint.chassis_speed);
		current_pitch = pitch_task(motor_setpoint.pitch_pos);
		current_yaw = yaw_task(motor_setpoint.yaw_pos, 2);
//		current_yaw = yaw_task(motor_setpoint.yaw_speed, 1);
//		
//		_820r_send_task(*current_chassis, *(current_chassis+1), current_bodan);
		gimbal_send_task(current_yaw, current_pitch);
//		_820r_send_task(0, 0, current_bodan);
//		gimbal_send_task(0, 1000);
	}
}


//断电模式
void sleep_mode(int time)       
{
	if(time == 3)   //云台PC发送底盘PC数据
	{
		pc_chassis2pc_gimbal_send_task(); 
	}
	else if(time == 4)
	{
		friction_send_task(84);     //关闭摩擦轮
	}
	else if(time == 5)
	{
		short current_bodan;
		
		current_bodan = bodan_task(0, 1);
		
		_820r_send_task(0, 0, current_bodan);
		gimbal_send_task(0, 0);
	}
}


/*--------------------自动模式-----------------*/
unsigned char cur_state = patrol_state; 
unsigned char next_state;
int error_HP;
int error_HP_all;
unsigned char error_HP_flag;
unsigned char HP_dodge_flag;
int error_HP_time_cnt;
int hurt_time_cnt1, hurt_time_cnt2;
int pc_gimbal_disconnect_cnt;
int yaw_num;
int yaw_bias;
int chassis_camera_flag;
float yaw_degree;    //折算角度
int yaw_num_mode3;
int yaw_num_mode4;
int yaw_bias_mode3;
int yaw_bias_mode4;
int yaw_angle_mode3;
int yaw_angle_mode4;
int kalman_time_cnt;

void auto_mode(int time)             
{
	remote_t* remote = get_remote();
	pc_t* pc_gimbal = get_pc_gimbal();
	judge_t* judge = get_judge();
	static int friction_time_cnt;
	
	yaw_degree = YAW/192.04f;
	
	if(remote->last_s1 == 3)
	{
		friction_time_cnt = 0;
		motor_setpoint.last_chassis_dir = 1;
		other2patrol_state();
//test code
//		motor_setpoint.yaw_pos = YAW;
//		motor_setpoint.pitch_pos = PITCH;
//		motor_setpoint.yaw_flag = 2;
		
		
		next_state = patrol_state;
	}
	
	cur_state = next_state;            //状态更新
	
	/*------------------损失血量计算-------------*/
	HP_handler();
	
	/*------------------卡尔曼滤波---------------*/
	if(pc_gimbal->pc_receive_flag == 1)   //每收到一次PC发来的数据就计算一次
	{
		pc_gimbal->pc_receive_flag = 0;
		pc_gimbal_disconnect_cnt = 0;
		pc_handler();
		if(pc_gimbal->gimbal_act_mode == 1 || pc_gimbal->gimbal_act_mode == 2)
			kalman_time_cnt ++;
	}
//	pc_gimbal_disconnect_cnt ++;
//	if(pc_gimbal_disconnect_cnt > 1000)   //1s内未收到云台PC发来的消息
//	{
//		pc_gimbal_disconnect_cnt = 0;
//		other2patrol_state();
//	}
	switch(cur_state)
	{				
		case patrol_state:       
			//patrol->shoot
			motor_setpoint.last_chassis_dir = motor_setpoint.chassis_speed/ABS(motor_setpoint.chassis_speed);
			if(pc_gimbal->chasiss_act_mode == 2 && pc_gimbal->gimbal_act_mode)     
			{
				other2shoot_state();
				next_state = shoot_state;
			}
			//patrol->dodge
			else if(HP_dodge_flag || judge->remain_HP <= REMAIN_LIMIT_HP)
			{
				other2dodge_state();
				next_state = dodge_state;
			}
			//normal
			else                                
			{
				turn_chassis_dir(1);
				//pitch在5°到15°摆动         
				motor_setpoint.pitch_pos = motor_setpoint.pitch_pos + motor_setpoint.pitch_dir*0.01f;
				if(motor_setpoint.pitch_pos >= 20.0f)	
					motor_setpoint.pitch_dir = -1;
				else if(motor_setpoint.pitch_pos <= 5.0f)
					motor_setpoint.pitch_dir = 1;
					
//				motor_setpoint.yaw_pos += 5;

//				if(encoder_all >= -3000)
//				{
//				}
//				else if(encoder_all <= -47000)
//				{
//				}
//				else
//				{
					if(YAW > 13000)
						motor_setpoint.yaw_speed = -ABS(PATROL_YAW_SPEED); 
					else if (YAW < -19767)
						motor_setpoint.yaw_speed = ABS(PATROL_YAW_SPEED);
//				}
				motor_setpoint.last_yaw_speed_dir = motor_setpoint.yaw_speed/ABS(motor_setpoint.yaw_speed);
			}
			break;
			
		case shoot_state:
			//shoot->dodge
			if(HP_dodge_flag || judge->remain_HP <= REMAIN_LIMIT_HP) 
			{
				other2dodge_state();
				next_state = dodge_state;
			}
			//shoot->patrol
			else if(pc_gimbal->chasiss_act_mode == 1 && pc_gimbal->gimbal_act_mode == 0 && !chassis_camera_flag)   //巡逻模式
//			else if(pc_gimbal->chasiss_act_mode == 1 && pc_gimbal->gimbal_act_mode == 0)   //巡逻模式
			{
				other2patrol_state();
				next_state = patrol_state;
			}
			//normal
			else    
			{
				if(pc_gimbal->gimbal_act_mode == 1 || pc_gimbal->gimbal_act_mode == 2)      //射击模式
				{
					
					/*------------------卡尔曼滤波---------------*/
					//初始化部分
//					if(kalman_init == 1)
//					{
//						pitch_kalman_filter.filtered_value[0] = PITCH;
//						pitch_kalman_filter.filtered_value[1] = 0;
//						yaw_kalman_filter.filtered_value[0] = YAW/91.02f;
//						yaw_kalman_filter.filtered_value[1] = 0;
//						target_pitch_para.last_position = target_pitch_para.last_position = PITCH;
//						target_yaw_para.last_position = target_pitch_para.cur_position = YAW/91.02f;
//					}
					

//					if(pc_gimbal->pc_receive_flag == 1)   //每收到一次PC发来的数据就计算一次
//					{
//						pc_gimbal->pc_receive_flag = 0;
//						pc_gimbal_disconnect_cnt = 0;
//						//if(pc_gimbal->gimbal_act_mode == 1|| pc_gimbal->gimbal_act_mode == 2)
//						pc_handler();
//						motor_setpoint.yaw_pos = YAW + pc_gimbal->relative_yaw_degree;
//						motor_setpoint.pitch_pos = PITCH + pc_gimbal->relative_pitch_degree;
//					}

				//	pc_gimbal_disconnect_cnt ++;
				//	if(pc_gimbal_disconnect_cnt > 1000)   //1s内未收到云台PC发来的消息
				//	{
				//		pc_gimbal_disconnect_cnt = 0;
				//		other2patrol_state();
				//	}
				
					chassis_camera_flag = 0;
					
					if(kalman_time_cnt >= 10)
					{
						motor_setpoint.yaw_pos = filtered_yaw_degree;
						motor_setpoint.pitch_pos = filtered_pitch_degree; 
					}
					else
					{
						motor_setpoint.yaw_pos = YAW;
						motor_setpoint.pitch_pos = PITCH;
					}
					
					if(pc_gimbal->gimbal_act_mode == 1)      //不发子弹,出现疑似目标
					{
						motor_setpoint.bodan_flag = 0;
					}
					else if(pc_gimbal->gimbal_act_mode == 2) //发子弹
					{
						if(ABS(pc_gimbal->relative_yaw_degree) < 1.0f)
							motor_setpoint.bodan_flag = 1;
						else
							motor_setpoint.bodan_flag = 0;
							
//						if(pc_gimbal->gimbal_shoot_mode == 0)
							motor_setpoint.bodan_mode = 3;       //三连发
//						else if(pc_gimbal->gimbal_shoot_mode == 1)
//							motor_setpoint.bodan_mode = 1;       //单发
					}
				}

				else if(pc_gimbal->gimbal_act_mode == 3 && !chassis_camera_flag)  //目标在基地区
				{
					//yaw轴角度5048
					motor_setpoint.bodan_flag = 0;
					
					chassis_camera_flag = 3;
	
					kalman_time_cnt = 0;
					
					yaw_num_mode3 = (int)(YAW/32767);
					yaw_bias_mode3 = YAW%32767;
					
					if(YAW >= 0)
					{
						if(yaw_bias_mode3 >= 21432)
							yaw_angle_mode3 = (yaw_num_mode3 + 1)*32767 + 5048 + 91.02f*pc_gimbal->relative_yaw_degree;
						else 
							yaw_angle_mode3 = yaw_num_mode3*32767 + 5048 + 91.02f*pc_gimbal->relative_yaw_degree;
					}
					else
					{
						if(0 >= yaw_bias_mode3 && yaw_bias_mode3 >= -11335)
							yaw_angle_mode3 = (yaw_num_mode3 + 1)*32767 - 27719 + 91.02f*pc_gimbal->relative_yaw_degree;
						else	
							yaw_angle_mode3 = yaw_num_mode3*32767 - 27719 + 91.02f*pc_gimbal->relative_yaw_degree;
					}
				}
			
				else if(pc_gimbal->gimbal_act_mode == 4 && !chassis_camera_flag)  //目标在荒地区，云台转过去
				{
					//yaw轴角度21432
					motor_setpoint.bodan_flag = 0;
					
					chassis_camera_flag = 4;
					
					kalman_time_cnt = 0;
					
					yaw_num_mode4 = (int)(YAW/32767);
					yaw_bias_mode4 = YAW%32767;
					
					if(YAW >= 0)
					{
						if(0 <= yaw_bias_mode4 && yaw_bias_mode4 < 5048)
							yaw_angle_mode4 = (yaw_num_mode4 - 1)*32767 + 21432 + 91.02f*pc_gimbal->relative_yaw_degree;
						else
							yaw_angle_mode4 = yaw_num_mode4*32767 + 21432 + 91.02f*pc_gimbal->relative_yaw_degree;
					}
					else 
					{
						if(-27719 >= yaw_bias_mode4 && yaw_bias_mode4 > -32767)
							yaw_angle_mode4 = (yaw_num_mode4 - 1)*32767 - 11335 + 91.02f*pc_gimbal->relative_yaw_degree;
						else
							yaw_angle_mode4 = yaw_num_mode4*32767 - 11335 + 91.02f*pc_gimbal->relative_yaw_degree;
					}
				}
			}

			if(chassis_camera_flag == 3 || chassis_camera_flag == 4)
			{
				if(pc_gimbal->gimbal_act_mode == 1 || pc_gimbal->gimbal_act_mode == 2)
				{
					chassis_camera_flag = 0;
				}
				else if(chassis_camera_flag == 3)     //21432，基地
				{	
					motor_setpoint.yaw_pos = yaw_angle_mode3; 
				}
				else if(chassis_camera_flag == 4)     //荒地
				{
					motor_setpoint.yaw_pos = yaw_angle_mode4;
				}
				
				if(ABS(YAW - motor_setpoint.yaw_pos) <= 500)    //判断是否达到相应位置
				{
					chassis_camera_flag = 0;
				}
			}
			break;
			
		case dodge_state:
			//血量太低
			if(judge->remain_HP <= REMAIN_LIMIT_HP)
			{	
				turn_chassis_dir(2);
			}
			//大子弹以及加成小子弹
			else if(HP_dodge_flag == 0x10)
			{
				hurt_time_cnt2 ++;
				if(hurt_time_cnt2 < 15000)
				{
					turn_chassis_dir(2);
				}
				else if(hurt_time_cnt2 >= 15000)
				{
					hurt_time_cnt2 = 0;
					HP_dodge_flag = 0;
					
					other2patrol_state();
					next_state = patrol_state;
				}
			}
			//小子弹在一定时间内造成大量伤害
			else if(HP_dodge_flag == 0x01)
			{		
				hurt_time_cnt1 ++;
				if(hurt_time_cnt1 < 2000)
				{
					motor_setpoint.chassis_speed = 0;
					motor_setpoint.yaw_speed = 2.0f;   //云台加速扫描
					if(pc_gimbal->chasiss_act_mode == 2 && (pc_gimbal->gimbal_act_mode == 1 || pc_gimbal->gimbal_act_mode == 2))   //射击模式打断
					{
						hurt_time_cnt1 = 0;
						
						other2shoot_state();
						next_state = shoot_state;
						
						HP_dodge_flag = 0;
					}
				}
				else if(hurt_time_cnt1 == 2000)
					motor_setpoint.chassis_speed = DODGE_CHASSIS_SPEED;
				else if(hurt_time_cnt1 < 8000)
				{
					motor_setpoint.yaw_speed = 0;
					rand_chassis_dir_generate();
					turn_chassis_dir(2);
				}
				else                                    //动作完成
				{
					hurt_time_cnt1 = 0;			
					HP_dodge_flag = 0;
					
					other2patrol_state();
					next_state = patrol_state;
				}
			}
			else
			{
				other2patrol_state();
				next_state = patrol_state;
			}
			break; 
	}
	
	
	if(time == 2)       //向云台PC发送自身数据
	{
//		pc_chassis2pc_gimbal_send_task(); 
	}
	else if(time == 3)  //云台PC发送底盘PC数据
	{
		pc_chassis2pc_gimbal_send_task(); 
	}
	else if(time == 4)   //摩擦轮
	{
		if(friction_time_cnt < 100)  //500ms
		{
			friction_time_cnt ++;
			friction_send_task(84);    //关闭摩擦轮
		}
		else
		{
			friction_time_cnt = 100;
			friction_send_task(84);   //开启摩擦轮
		}
	}
	else if(time == 5)
	{
		short current_pitch;
		short current_yaw;
		short current_bodan;
		short *current_chassis;
		
		current_bodan = bodan_task(motor_setpoint.bodan_flag, motor_setpoint.bodan_mode);
		current_chassis = chassis_task(motor_setpoint.chassis_speed);
		current_pitch = pitch_task(motor_setpoint.pitch_pos);
		if(motor_setpoint.yaw_flag == YAW_SINGLE)  
			current_yaw = yaw_task(motor_setpoint.yaw_speed, YAW_SINGLE);
		else if(motor_setpoint.yaw_flag == YAW_DOUBLE)
			current_yaw = yaw_task(motor_setpoint.yaw_pos, YAW_DOUBLE);
		
		_820r_send_task(*current_chassis, *(current_chassis+1), current_bodan);
		gimbal_send_task(current_yaw, 0);
// 		_820r_send_task(0,0,current_bodan);
//		gimbal_send_task(0,0);
	}
}

/*---------------------断线检测-----------------*/
void disconnect_handler(void)
{
	LED_PC_OFF;                    
	LED_GYRO_OFF;
	
	is_gyro_connect ++;
	is_pc_chassis_connect ++;
	
	if(is_gyro_connect >= 10)
	{
		is_gyro_connect = 0;
		gimbal_send_task(0,0);
		CAN_Configuration();    //重启CAN
	}
	
	if(is_pc_chassis_connect >= 100)  //1s
	{
		is_pc_chassis_connect = 90;    //0.1s
		pc_disconnect_cnt ++;
		
		if(pc_disconnect_cnt < 6)
		{
			DMA_Cmd(DMA1_Channel6, DISABLE);
			DMA_Cmd(DMA1_Channel7, DISABLE);
			USART2_Configuration();                  //重启串口2
		}
		else 
		{
			pc_disconnect_cnt = 6;                   //重启失败，向底盘PC发送0
		}
	}
	else if(is_pc_chassis_connect < 90)
	{
		pc_disconnect_cnt = 0;
	}
}


/*-----------血量损失检测及处理函数--------*/
int last_HP;
int cur_HP;
int reduce_HP(void)
{
	int error_HP;
	judge_t* judge = get_judge();
	cur_HP = judge->remain_HP;
	error_HP = last_HP - cur_HP;	
	last_HP = cur_HP;
	return error_HP;
}

void HP_handler(void)
{
	error_HP = reduce_HP();
	if(!error_HP_flag)
	{
		if(error_HP > 0)
		{
			error_HP_flag = 1;             //受到伤害
			error_HP_all = 0;
		}
	}
	if(error_HP_flag)  
	{
		error_HP_all += error_HP;
		error_HP_time_cnt ++;
		if(!HP_dodge_flag)
		{
			if(error_HP_all >= 100)          //累计伤害100，躲避
				HP_dodge_flag = 0x01;          //置位伤害标志
		}
		if(error_HP_time_cnt >= 1000)      //受到伤害1s，终止计时
		{
			error_HP_flag = 0;
			error_HP_time_cnt = 0;
		}
	}
	if(error_HP >= 150)                  //认为受到大子弹或者加成小子弹伤害  
		HP_dodge_flag = 0x10;
}

//transform other state to patrol state
void other2patrol_state(void)
{
	motor_setpoint.chassis_speed = motor_setpoint.last_chassis_dir*PATROL_CHASSIS_SPEED;
	motor_setpoint.yaw_speed = motor_setpoint.last_yaw_speed_dir*PATROL_YAW_SPEED;
	motor_setpoint.yaw_flag = YAW_SINGLE;
//	motor_setpoint.yaw_flag = YAW_DOUBLE;
	motor_setpoint.yaw_pos = YAW;
	motor_setpoint.pitch_pos = PITCH;
	motor_setpoint.bodan_flag = 0;
}

//transform other state to dodge state
void other2dodge_state(void)
{
	motor_setpoint.chassis_speed = DODGE_CHASSIS_SPEED;
	motor_setpoint.yaw_speed = 0;
	motor_setpoint.yaw_flag = YAW_SINGLE;
	motor_setpoint.pitch_pos = PATROL_PITCH_POS_INIT;
	motor_setpoint.bodan_flag = 0;
}

//transform other state to shoot state
void other2shoot_state(void)
{
	pc_t* pc_gimbal = get_pc_gimbal();
	motor_setpoint.chassis_speed = 0;   //底盘静止
	//pitch、yaw保持当前位置
	motor_setpoint.yaw_pos = YAW;
	motor_setpoint.yaw_flag = YAW_DOUBLE;
	motor_setpoint.pitch_pos = PITCH;
	chassis_camera_flag = 0;
	kalman_time_cnt = 0;
}

/**
  * @brief  行程开关检测到两端柱子，转向
  * @param  flag = 1, patrol; flag = 2, dodge.
  * @retval None
  */
void turn_chassis_dir(unsigned char flag)
{
	unsigned char *limit_sw_value = get_sw_value();
	int encoder_point1, encoder_point2, encoder_point3, encoder_point4;
	
	if(wifi_choose_flag == 4)      //全程
	{
		encoder_point1 = all_encoder_point1;
		encoder_point2 = all_encoder_point2;
		encoder_point3 = all_encoder_point3;
		encoder_point4 = all_encoder_point4;
	}
#ifdef TURN_CHASSIS_DIR_FIGHT
	else if(wifi_choose_flag == 1)  //第一段
	{
		encoder_point1 = first_encoder_point1;
		encoder_point2 = first_encoder_point2;
		encoder_point3 = first_encoder_point3;
		encoder_point4 = first_encoder_point4;
	}
	else if(wifi_choose_flag == 2)  //第二段
	{
		encoder_point1 = second_encoder_point1;
		encoder_point2 = second_encoder_point2;
		encoder_point3 = second_encoder_point3;
		encoder_point4 = second_encoder_point4;
	}
	else if(wifi_choose_flag == 3)  //第三段
	{
		encoder_point1 = third_encoder_point1;
		encoder_point2 = third_encoder_point2;
		encoder_point3 = third_encoder_point3;
		encoder_point4 = third_encoder_point4;
	}
#endif
	
	/*-------------行程开关----------------*/
	if(limit_sw_value[1] == 0)       //到达顶端
	{
		encoder_all = 0;
	}
	if(limit_sw_value[0] == 0)
	{
		encoder_all = -52000;
	}
	
	/*-------------编码器------------------*/
	if(encoder_all <= encoder_point1 && encoder_all >= encoder_point2)  //到达顶端减速 
	{
		if(motor_setpoint.chassis_speed >= 0)  
		{
			if(flag == 1)
				motor_setpoint.chassis_speed = (ABS(PATROL_CHASSIS_SPEED) - 1000)/(encoder_point2 - encoder_point1)*(encoder_all - encoder_point1) + 1000;
			else if(flag == 2)
				motor_setpoint.chassis_speed = (ABS(DODGE_CHASSIS_SPEED) - 1000)/(encoder_point2 - encoder_point1)*(encoder_all - encoder_point1) + 1000;
		}
		else if(motor_setpoint.chassis_speed < 0)
		{
			if(flag == 1)
				motor_setpoint.chassis_speed = (-ABS(PATROL_CHASSIS_SPEED)+1000)/(encoder_point2 - encoder_point1)*(encoder_all- encoder_point1) - 1000;
			else if(flag == 2)
				motor_setpoint.chassis_speed = (-ABS(DODGE_CHASSIS_SPEED)+1000)/(encoder_point2 - encoder_point1)*(encoder_all- encoder_point1) - 1000;
		}
	}
	else if((encoder_point1 + 100) >= encoder_all && encoder_all> encoder_point1)
	{
		if(flag == 1)
			motor_setpoint.chassis_speed = -1000;//-ABS(PATROL_CHASSIS_SPEED);
		else if(flag == 2)
			motor_setpoint.chassis_speed = -1000;//-ABS(DODGE_CHASSIS_SPEED);
	}
	else if(encoder_all > (encoder_point1 + 100))
	{
		if(wifi_choose_flag != 4)
			motor_setpoint.chassis_speed = -8000;
		else 
		{	
			motor_setpoint.chassis_speed = -ABS(PATROL_CHASSIS_SPEED);
		}
	}
	
	
	if(encoder_all <= encoder_point3 && encoder_all >= encoder_point4)
	{
		if(motor_setpoint.chassis_speed <= 0)  
		{
			if(flag == 1)
				motor_setpoint.chassis_speed = (-ABS(PATROL_CHASSIS_SPEED) + 1000)/(encoder_point3 - encoder_point4)*(encoder_all - encoder_point4) - 1000;
			else if(flag == 2)
				motor_setpoint.chassis_speed = (-ABS(DODGE_CHASSIS_SPEED) + 1000)/(encoder_point3 - encoder_point4)*(encoder_all - encoder_point4) - 1000;
		}
		else if(motor_setpoint.chassis_speed > 0)
		{
			if(flag == 1)
				motor_setpoint.chassis_speed = (ABS(PATROL_CHASSIS_SPEED) - 1000)/(encoder_point3 - encoder_point4)*(encoder_all - encoder_point4) + 1000;
			else if(flag == 2)
				motor_setpoint.chassis_speed = (ABS(DODGE_CHASSIS_SPEED) - 1000)/(encoder_point3 - encoder_point4)*(encoder_all - encoder_point4) + 1000;
		}
	}
	else if((encoder_point4 - 100) <= encoder_all && encoder_all < encoder_point4)
	{
		if(flag == 1)
			motor_setpoint.chassis_speed = 1000;
		else if(flag == 2)
			motor_setpoint.chassis_speed = 1000;
	}
	else if(encoder_all < (encoder_point4 - 100))
	{
		if(wifi_choose_flag != 4)
			motor_setpoint.chassis_speed = 8000;
		else	
			motor_setpoint.chassis_speed = ABS(PATROL_CHASSIS_SPEED);
	}
}

/**
  * @brief  热量限制，上限热量480，每秒冷却160
  * @param  None
  * @retval None
  */
int heat_limit_task(void)
{
	static int time_stop_length;
	static int stop_on;
	judge_t* judge = get_judge();
	
	if(judge->shooter_heat17mm >= 450)      //留30的余量
	{
		bodan_new_state = 0;
		time_stop_length = 0;
	}
	if(!bodan_new_state)
	{
		time_stop_length ++;
		if(time_stop_length == 50)           //停止射击250ms
		{
			bodan_new_state = 1;
			time_stop_length = 0;
		}
	}
	return bodan_new_state;
}


/**
  * @brief  底盘堵转检测，防止编码器移位后底盘未撞到行程开关。
  * @param  None
  * @retval None
  */
void motor_locked(void)
{

}

