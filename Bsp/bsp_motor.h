#ifndef __MOTOR_H_
#define __MOTOR_H_

#include "main.h"

typedef struct
{
    float ecd;
	  float last_ecd;
		float err_ecd;
	
} motor_measure_t;

extern motor_measure_t motor_rxbuf[5];

void Motor_init(void);
void Motor_Control(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);
void Guide_Motor_Control(int16_t motor);
void Servo_Control(TIM_HandleTypeDef *htim, uint32_t channel, float angle);
void TIM_Interrupt_Callback(TIM_HandleTypeDef *htim);
#endif
