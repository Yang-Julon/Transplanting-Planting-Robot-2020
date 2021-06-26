#include "servo_task.h"
#include "chassis_task.h"
#include "plan_task.h"
#include "tim.h"
#include "cmsis_os.h"

#define SERVO_TASK_INIT_TIME 50
extern chassis_move_t chassis_move;

void ServoTask(void const * argument)
{
	vTaskDelay(SERVO_TASK_INIT_TIME);
	
	while(1)
	{
		if(chassis_flag.iteration == false && chassis_Distance_Detection(chassis_move.guide_length, GUIDE_LENGTH, 0.01))
		{
			HAL_Delay(100);
			if(chassis_flag.iteration == false && chassis_Distance_Detection(chassis_move.guide_length, GUIDE_LENGTH, 0.01))
			{
				Servo_Control(&htim8, TIM_CHANNEL_1, 155);
				HAL_Delay(200);
				Servo_Control(&htim8, TIM_CHANNEL_1, 175);
				HAL_Delay(200);
				Servo_Control(&htim8, TIM_CHANNEL_1, 90);
				HAL_Delay(200);
				
				chassis_flag.iteration = true;
			}
		}
		
		if(chassis_flag.iteration == true && chassis_Distance_Detection(chassis_move.guide_length, 0, 0.05))
		{
		
				Servo_Control(&htim8, TIM_CHANNEL_1, 175);
				HAL_Delay(500);
				chassis_flag.iteration = false;
				chassis_flag.work = false;
				chassis_flag.ready = false;
				Servo_Control(&htim8, TIM_CHANNEL_2, 50);
				HAL_Delay(500);
				Servo_Control(&htim8, TIM_CHANNEL_2, 10);
		}
		
		vTaskDelay(SERVO_TASK_INIT_TIME);
	}
}

