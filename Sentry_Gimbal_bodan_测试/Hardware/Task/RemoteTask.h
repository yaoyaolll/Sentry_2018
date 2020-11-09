#ifndef _REMOTETASK_H_
#define _REMOTETASK_H_

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
}RemoteRec_TypeDef;

RemoteRec_TypeDef* Get_RemoteData(void);
void RemoteReceiveTask(volatile unsigned char rx_buffer[]);
void RC_Init(void);

#endif
