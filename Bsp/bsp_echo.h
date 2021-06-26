#ifndef __TRIGCHO_H_
#define __TRIGCHO_H_

#include "main.h"
#include "tim.h"
#include "sensor.h"
#include "bsp_delay.h"

typedef struct
{
	uint8_t CAPTURE_STA;
	uint16_t CAPTURE_VAL_START;
	uint16_t CAPTURE_VAL_END;
	uint16_t time;
	int8_t count;
	uint16_t buf;
	uint16_t distance;
}trig_echo_t;

typedef enum
{
	LF = 0,
	RF,
	LB,
	RB
}echo_e;

extern trig_echo_t trig_echo[4];;


void Echo_RX_Calc(void);
void trigProduce(void);

#endif

