#ifndef __PLAN_TASK_H
#define __PLAN_TASK_H

#include "cmsis_os.h"
#include "main.h"

extern uint16_t route; 
extern bool enter_flag;
extern bool work_flag;

void PlanTask(void const * argument);
bool chassis_Speed_Detection(float get, float set);
bool chassis_Distance_Detection(float get, float set, float range);

#endif

