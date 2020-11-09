#ifndef _DATA_REC_TASK_H_
#define _DATA_REC_TASK_H_
#include "stdint.h"

typedef struct{
	struct
	{
		unsigned short ch0;
		unsigned short ch1;
		unsigned short ch2;
		unsigned short ch3;
		unsigned char s1;
		unsigned char s2;
	}rc;
	struct
	{
		short x;
		short y;
		short z;
		unsigned char press_l;
		unsigned char press_r;
	}mouse;
	struct
	{
		unsigned short w,s,a,d,q,e,r,f,g,z,x,c,v,b,shift,ctrl;
	}key;
	volatile char RCrecvd,RCDisconnectCnt;
	unsigned char last_s1;
}remote_t;

typedef struct
{
	uint16_t BulletNum;
	uint16_t WifiYaw;
	uint16_t WifiPitch;
	short speedx;
	short speedy;
	short WifiDiscont;
	short WifiConnect;
}wifi_t;

typedef struct{
	unsigned char source_address;   //Byte[0]:0~3bit, 0:from gimbal TX2, 1:from chassis TX2, 2:from lower mcu
	unsigned char destin_address;   //Byte[0]:4~7bit, 0:send to gimbal TX2, 1:send to chassis TX2, 2:send to lower mcu
	unsigned char chasiss_act_mode; //Byte[1]:0~3bit, 0:hide mode, 1:patrol mode, 2:sleep mode, 3:move mode(receive the position from TX2)
	unsigned char gimbal_act_mode;  //Byte[1]:4~7bit, 0:patrol mode, 1:shoot mode without firing bullets, 2:shoot mode with firing
	unsigned char chassis_move_pos; //Byte[2]:0~3bit, 0~6:move chassis to the position
	unsigned char gimbal_shoot_mode;  //Byte[2]:4~7bit, wait to be defined
	float relative_pitch_degree;    //Byte[3] Byte[4]
	float relative_yaw_degree;      //Byte[5] Byte[6]
	
	unsigned char pc_receive_flag;
}pc_t;

typedef struct{
	//cmdid = 0x0001
	unsigned short stage_remain_time;
	unsigned short remain_HP;
	
	//cmdid = 0x0002
	unsigned char armor_type;   //装甲伤害时，标识装甲ID，0、1号装甲，0号朝向基地区
	unsigned char hurt_type;    //血量变化类型，0是装甲伤害，1是模块掉线
	
	//cmdid = 0x0003
	unsigned short bullet_freq;   //射频
	float bullet_speed;           //射速
	
	//cmdid = 0x0004
	unsigned short shooter_heat17mm;  //17mm枪口热量
	
}judge_t;
#define JudgeBufBiggestSize 34
#define JudgeSendBufSize    22

remote_t* get_remote(void);
void remote_rec_task(volatile unsigned char rx_buffer[]);
pc_t* get_pc_gimbal(void);
void pc_gimbal2mcu_rec_task(unsigned char *str);
void pc_chassis2pc_gimbal_rec_task(unsigned char *str);
judge_t* get_judge(void);
void judge_rec_task(unsigned char ReceiveBuffer[]);
void wifi_rec_task(unsigned char *str);
	
#endif
