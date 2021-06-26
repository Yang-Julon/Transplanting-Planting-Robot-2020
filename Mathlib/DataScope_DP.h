

#ifndef __DATA_PRTOCOL_H
#define __DATA_PRTOCOL_H
 
#include "stm32f4xx.h"
//extern unsigned char DataScope_OutPut_Buffer[42];	   //待发送帧数据缓存区

typedef struct
{
	unsigned char DataScope_OutPut_Buffer[42];	   //串口发送缓冲区
	unsigned char Send_Count; //串口需要发送的数据个数
	unsigned char DataCnt;          //计数变量
}DataTypedfef;


void DataScope_Get_Channel_Data(float Data,unsigned char Channel);    // 写通道数据至 待发送帧数据缓存区

unsigned char DataScope_Data_Generate(unsigned char Channel_Number);  // 发送帧数据生成函数 
void DataWave(UART_HandleTypeDef* huart);
 
extern DataTypedfef CK;
#endif 



