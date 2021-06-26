#ifndef __TFMINIPLUS_H_
#define __TFMINIPLUS_H_

#include "main.h"
#include "string.h"

#define USART_BUF_SIZE              64
#define TFMINI_DATA_Len             9
#define TFMINT_DATA_HEAD            0x59
#define TOF_DATA_LEN								7
#define TOF_ID_00										0x00
#define TOF_ID_01										0x01
#define TOF_ORDER_NUMBER						0x03

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
	
}laser_e;

extern float laser_data[3];

void TFmini_init(void);

#endif

