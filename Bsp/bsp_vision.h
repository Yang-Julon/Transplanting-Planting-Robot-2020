#ifndef __BSP_VISION_H
#define __BSP_VISION_H

#include "main.h"
#include "string.h"

#define USART_BUF_SIZE 64
#define VISION_DATA_Len 5
#define VISION_DATA_HEAD 0x20

void Vision_init(void);
extern float vision_buf[2];

#endif

