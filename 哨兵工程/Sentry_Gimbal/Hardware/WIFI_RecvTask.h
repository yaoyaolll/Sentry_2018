#ifndef _WIFI_RECVTASK_H_
#define _WIFI_RECVTASK_H_

#include "stdint.h"

typedef struct
{
	uint16_t BulletNum;
	uint16_t WifiYaw;
	uint16_t WifiPitch;
	short speedx;
} WifiId_t;

typedef
	struct{
	 WifiId_t id[8]; 
	 short WifiDiscont;
	 short WifiConnect;
	} WifiStruct_t;

#endif 
