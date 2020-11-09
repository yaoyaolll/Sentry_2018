#include "main.h"

WifiStruct_t wifiGet;

/**
  * @brief  wifi模块接收函数
  * @param  *str  WiFi收到的字符串
  * @retval Yaw角度
  */
void WifiRecv(unsigned char *str)
{
	switch(str[1])
	{
		case 0x01:
				wifiGet.id[0].BulletNum=(str[2]<<8)|str[3];
				wifiGet.id[0].WifiPitch=(str[4]<<8)|str[5];
				wifiGet.id[0].WifiYaw=(str[6]<<8)|str[7];
				wifiGet.id[0].speedx=(str[8]<<8)|str[9];
				 break;
		case 0x02:
				wifiGet.id[1].BulletNum=(str[0]<<8)|str[1];
				wifiGet.id[1].WifiPitch=(str[2]<<8)|str[3];
				wifiGet.id[1].WifiYaw=(str[4]<<8)|str[5];
				wifiGet.id[1].speedx=(str[6]<<8)|str[7];
				break;
		case 0x04:
				
				break;
		case 0x08:
				
				break;
		case 0x11:
				
				break;
		case 0x12:
				
				break;
		case 0x14:
				
				break;
		case 0x18:
				
				break;
		default:
				break;
	}
	wifiGet.WifiConnect=1;
	wifiGet.WifiDiscont=0;
}

///**
//* @para String str 长度限制为9
//* @retval none
//*
//*/
//unsigned char WifiSendBuf[StrLength];
//void Uart4WifiSend(unsigned char* str,char id)
//{
//	int i;
//	WifiSendBuf[0]='!';
//	WifiSendBuf[1]=id;
//	WifiSendBuf[2]=str[0];
//	WifiSendBuf[3]=str[1];
//	WifiSendBuf[4]=str[2];
//	WifiSendBuf[5]=str[3];
//	WifiSendBuf[6]=str[4];
//	WifiSendBuf[7]=str[5];
//	WifiSendBuf[8]=str[6];
//	WifiSendBuf[9]=str[7];
//	WifiSendBuf[10]=str[8];
////	WifiSendBuf[11]='#';
//	Append_CRC8_Check_Sum(WifiSendBuf,StrLength);
//	for(i=0;i<StrLength;i++)
//	{
//		USART_SendData(UART4,WifiSendBuf[i]);
//		while(USART_GetFlagStatus(UART4,USART_FLAG_TC)!=SET);
//	}
//}
