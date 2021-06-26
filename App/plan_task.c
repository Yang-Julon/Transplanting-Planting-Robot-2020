#include "plan_task.h"
#include "chassis_task.h"
#include "bsp_usart.h"
#include "tim.h"
#define PLAN_TASK_INIT_TIME  500
#define PLAN_CONTROL_TIME   10

extern chassis_move_t chassis_move;
extern flag_t chassis_flag;
extern bool ones;
uint16_t route = 0; 														//路径规划
bool enter_flag = false;


/**
  * @brief          底盘路径规划
	* @param[out]     chassis_move_plan:"chassis_move"变量指针.
  * @retval         none
  */
static void path_planning(chassis_move_t *chassis_move_plan);
/**
  * @brief          视觉数据重置
	* @param[out]     chassis_move_plan:"chassis_move"变量指针.
  * @retval         none
  */
static void vision_data_reset(chassis_move_t *chassis_data);



void PlanTask(void const * argument)
{
	vTaskDelay(PLAN_TASK_INIT_TIME);
	
	while(1)
	{
		path_planning(&chassis_move);
		vTaskDelay(PLAN_CONTROL_TIME);
	}

}

static void path_planning(chassis_move_t *chassis_move_plan)	//路径规划
{
	
	vision_data_reset(chassis_move_plan);
	
	
	//标志初始化成功,待完善...
	if(route == 0)
	{
		if(chassis_move_plan->chassis_laser[Forward] != 0)//前激光启动慢
		{
			route ++;
			chassis_flag.init = false;
		}
	}
	
	else if(route == 1)
	{
		chassis_flag.forward = true;
		if(!infared_LF && chassis_Distance_Detection(chassis_move_plan->chassis_laser[Forward], LASER_DISTANCE_TO_WALL+3, 1))
		{
			HAL_Delay(10);
			if(!infared_LF && chassis_Distance_Detection(chassis_move_plan->chassis_laser[Forward], LASER_DISTANCE_TO_WALL+3, 1))
			{
				chassis_flag.forward = false;
				route ++;
			}
		}
	}
	
	else if(route == 2)
	{
		chassis_flag.stop = true;
		if(chassis_Speed_Detection(chassis_move_plan->wz, 0) && chassis_Distance_Detection(chassis_move_plan->chassis_INS_angle[0], -90, 10))
		{
			HAL_Delay(10);
			if(chassis_Speed_Detection(chassis_move_plan->wz, 0) && chassis_Distance_Detection(chassis_move_plan->chassis_INS_angle[0], -90, 10))
			{
				chassis_flag.stop = false;
				route ++;
			}
		}
	}
	
	else if(route == 3  || route == 9 || route == 15)
	{
		chassis_flag.forward = true;
		if(!infared_LF && chassis_Distance_Detection(chassis_move_plan->chassis_laser[Forward], LASER_DISTANCE_TO_WALL+2, 1))
		{
			HAL_Delay(10);
			if(!infared_LF && chassis_Distance_Detection(chassis_move_plan->chassis_laser[Forward], LASER_DISTANCE_TO_WALL+2, 1))
			{
				chassis_flag.forward = false;
				route ++;
			}
		}
	}
	
		else if(route == 4 || route == 10 || route == 16)
	{
		chassis_flag.right = true;		
		if(!infared_LB && !enter_flag)
		{
			HAL_Delay(10);
			if(!infared_LB && !enter_flag)
			{
				enter_flag = true;
			}
		}
		
		if((infared_LB && infared_RB) && enter_flag)
		{
			HAL_Delay(50);
			if((infared_LB && infared_RB) && enter_flag)
			{
				chassis_flag.right = false;
				route ++;
				enter_flag = false;
			}
		}
	}	
	
	
	else if(route == 5 || route == 11 || route == 17)
	{
		chassis_flag.stop = true;
		if(chassis_Speed_Detection(chassis_move_plan->vx, 0) && chassis_Speed_Detection(chassis_move_plan->vy, 0))
		{
			HAL_Delay(10);
			if(chassis_Speed_Detection(chassis_move_plan->vx, 0) && chassis_Speed_Detection(chassis_move_plan->vy, 0))
			{
				chassis_flag.stop = false;
				route ++;
			}
		}
	}
		
	else if(route == 6 || route ==  12 || route == 18)
	{
		chassis_flag.back = true;
		if(!infared_LB && chassis_Distance_Detection(chassis_move_plan->chassis_laser[Back], LASER_DISTANCE_TO_WALL+2, 2))
		{
			HAL_Delay(10);
			if(!infared_LB && chassis_Distance_Detection(chassis_move_plan->chassis_laser[Back], LASER_DISTANCE_TO_WALL+2, 2))
			{
				chassis_flag.back = false;
				route ++;
			}
		}
	}

	else if(route == 7 || route == 13)
	{
		chassis_flag.right = true;
		if(!infared_LF && !enter_flag)
		{
			HAL_Delay(10);
			if(!infared_LF && !enter_flag)
			{
				enter_flag = true;
			}
		}
		
		if((infared_LF && enter_flag))//infared_RF) && enter_flag)
		{
			HAL_Delay(10);
			if((infared_LF && enter_flag))//infared_RF) && enter_flag)
			{
				chassis_flag.right = false;
				route ++;
				enter_flag = false;
			}
		}	
	}
	
	else if(route == 8 || route == 14)
	{
		chassis_flag.stop = true;
		if(chassis_Speed_Detection(chassis_move_plan->vx, 0) && chassis_Speed_Detection(chassis_move_plan->vy, 0))
		{
			HAL_Delay(10);
			if(chassis_Speed_Detection(chassis_move_plan->vx, 0) && chassis_Speed_Detection(chassis_move_plan->vy, 0))
			{
				chassis_flag.stop = false;
				route ++;
			}
		}
	}
	
	else if(route == 19)
	{
		chassis_flag.right = true;
		if(infared_LB)
		{
			HAL_Delay(150);//100
			if(infared_LB)
			{
				chassis_flag.right = false;
				route ++;
			}
		}
	}

	else 
		chassis_flag.stop = true;

}

static void vision_data_reset(chassis_move_t *chassis_data)
{
	if(chassis_Distance_Detection(chassis_move.guide_length,0,0.05))
	{
		//	HAL_Delay(90);//100
			if(chassis_flag.vision == false && chassis_data->vision_data[0] == 0)
					{
						if(chassis_data->vx != 0)
						{
							chassis_flag.ready = true;
							chassis_flag.work = false;
							ones = true;
						}
					}
	}
}

bool chassis_Speed_Detection(float get, float set)
{
	if(get >= (-0.005f+set) && get <= (0.005f+set))
		return true;
	return false;
}

bool chassis_Distance_Detection(float get, float set, float range)
{
	if(get >= (-range+set) && get <= (range+set))
		return true;
	return false;
}
