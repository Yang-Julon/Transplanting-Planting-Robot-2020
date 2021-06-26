#ifndef __WT101_H__
#define __WT101_H__

#include "main.h"
#include "string.h"
#include "sensor.h"


#define USART_BUF_SIZE              64
#define WT101_DATA_Len              11
#define WT101_DATA_HEAD            	0x55
#define WT101_PALSTANCE_HEAD        0x52
#define WT101_ANGLE_HEAD            0x53

typedef enum
{
	Angle = 0,
	Palstance,

}gyro_e;

extern float gyro_buf[2];

void WT101_init(void);
void Gyro_Data_Get(sensor_t *rxbuf);

#endif

