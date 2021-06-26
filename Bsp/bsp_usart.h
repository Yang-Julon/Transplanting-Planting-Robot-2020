#ifndef __BSP_USART_H
#define __BSP_USART_H

#include "main.h"
#include "string.h"



#define USART_BUF_SIZE              64
#define TFMINI_DATA_Len             9
#define TFMINT_DATA_HEAD            0x59
#define TOF_DATA_LEN								7
#define TOF_ID_00										0x00
#define TOF_ID_01										0x01
#define TOF_ID_02										0x02
#define TOF_ORDER_NUMBER						0x03

#define USART_BUF_SIZE              64
#define WT101_DATA_Len              11
#define WT101_DATA_HEAD            	0x55
#define WT101_PALSTANCE_HEAD        0x52
#define WT101_ANGLE_HEAD            0x53

#define USART_BUF_SIZE						  64
#define VISION_DATA_Len 						5
#define VISION_DATA_HEAD 						0x20

typedef enum
{
	Dist = 0,
  Strength
	
}minitf_e;

typedef enum
{
	Forward = 0,
	Left,
	Back,
	Right,
	
}laser_e;

typedef enum
{
	Angle = 0,
	Palstance,

}gyro_e;

extern float laser_buf[4];
extern float vision_buf[2];
extern float gyro_buf[2];

void TFmini_init(void);
void WT101_init(void);
void Vision_init(void);
void USART_RX_Callback(UART_HandleTypeDef *huart);
#endif
