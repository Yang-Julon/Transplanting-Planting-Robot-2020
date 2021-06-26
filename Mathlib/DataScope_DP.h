

#ifndef __DATA_PRTOCOL_H
#define __DATA_PRTOCOL_H
 
#include "stm32f4xx.h"
//extern unsigned char DataScope_OutPut_Buffer[42];	   //������֡���ݻ�����

typedef struct
{
	unsigned char DataScope_OutPut_Buffer[42];	   //���ڷ��ͻ�����
	unsigned char Send_Count; //������Ҫ���͵����ݸ���
	unsigned char DataCnt;          //��������
}DataTypedfef;


void DataScope_Get_Channel_Data(float Data,unsigned char Channel);    // дͨ�������� ������֡���ݻ�����

unsigned char DataScope_Data_Generate(unsigned char Channel_Number);  // ����֡�������ɺ��� 
void DataWave(UART_HandleTypeDef* huart);
 
extern DataTypedfef CK;
#endif 



