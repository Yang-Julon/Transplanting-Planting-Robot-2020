#ifndef __CHASSIS_TASK_H_
#define __CHASSIS_TASK_H_

#include "pid.h"
#include "main.h"
#include "sensor.h"
#include "bsp_motor.h"
#include "stdbool.h"


#define CHASSIS_TASK_INIT_TIME							500												//底盘任务初始化时间
#define CHASSIS_CONTROL_TIME_MS							10												//底盘控制的时间

#define MAX_WHEEL_SPEED 										4.0f											//单个底盘电机最大速度	
#define NORMAL_MAX_CHASSIS_SPEED_X 					2.0f											//底盘运动过程最大前进速度
#define NORMAL_MAX_CHASSIS_SPEED_Y 					1.5f											//底盘运动过程最大平移速度
#define MOTOR_SPEED_TO_CHASSIS_SPEED_VX 		0.25f
#define MOTOR_SPEED_TO_CHASSIS_SPEED_VY 		0.25f
#define MOTOR_SPEED_TO_CHASSIS_SPEED_WZ			0.25f
#define CHASSIS_WZ_SET_SCALE							  2.0f
#define MOTOR_DISTANCE_TO_CENTER 					  1.0f
#define LASER_DISTANCE_TO_WALL							10.0f
#define GUIDE_LENGTH                        -0.168		//-0.16

#define GET_ENCODER_TIME										0.005f										//读取编码器时间间隔 单位s(秒)
#define PI																	3.1415926535f

#define CHASSIS_MOTOR_COUNT_TO_SPE					(PI*WHEEL_RADIUS/8422.4f)	//编码值转化速度
#define WHEEL_RADIUS												0.035f										//轮子半径

#define GUIDE_MOTOR_COUNT_TO_SPE						(PI*GEAR_RADIUS/8422.4f)		//编码值转化速度
#define GEAR_RADIUS													0.02f											//齿轮半径

#define infared_LF 	HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_1)
#define infared_RF 	HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_7)
#define infared_LB 	HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_5)
#define infared_RB 	HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_7)

typedef enum
{
	CHASSIS_INIT,
	CHASSIS_STOP,				 //底盘停止
  CHASSIS_FORWARD,	   //底盘有底盘角度控制闭环
	CHASSIS_BACK,
  CHASSIS_LEFT,        //底盘有旋转速度控制
	CHASSIS_RIGHT,
	CHASSIS_VISION,			 //底盘追随视觉
	
} chassis_mode_e;

typedef struct
{
  const motor_measure_t *chassis_motor_measure;
  float accel;
  float speed;
  float speed_set;
  int16_t give_current;
} chassis_motor_t;

typedef struct
{
  const float *chassis_INS_angle;             //获取陀螺仪解算出的欧拉角指针
	const float *chassis_laser;
	const float *vision_data;
  chassis_mode_e chassis_mode;              	//底盘控制状态机
  chassis_mode_e last_chassis_mode;           //底盘上次控制状态机
  chassis_motor_t motor_chassis[4];           //底盘电机数据
	chassis_motor_t motor_guide;
	
  pid_t motor_speed_pid[4];            				//底盘电机速度pid
  pid_t chassis_angle_pid;             				//底盘跟随角度pid
	pid_t chassis_laser_wall_pid;
	pid_t vision_pid;
	pid_t guide_speed_pid;
	pid_t guide_length_pid;


  float vx;                          //底盘速度 前进方向 前为正，单位 m/s
  float vy;                          //底盘速度 左右方向 左为正  单位 m/s
  float wz;                          //底盘旋转角速度，逆时针为正 单位 rad/s
  float vx_set;                      //底盘设定速度 前进方向 前为正，单位 m/s
  float vy_set;                      //底盘设定速度 左右方向 左为正，单位 m/s
  float wz_set;                      //底盘设定旋转角速度，逆时针为正 单位 rad/s
  float chassis_yaw_set;   
	float guide_length;
	float guide_length_set;
		
  float vx_max_speed;  //前进方向最大速度 单位m/s
  float vx_min_speed;  //后退方向最大速度 单位m/s
  float vy_max_speed;  //左方向最大速度 单位m/s
  float vy_min_speed;  //右方向最大速度 单位m/s
  float chassis_yaw;   //底盘的yaw角度

} chassis_move_t;

typedef struct 
{
	bool forward;
	bool left;
	bool right;
	bool stop;
	bool back;
	bool Lrotation;
	bool Rrotation;
	bool vision;
	bool init;
	bool iteration;
	bool work;
	bool ready;
}flag_t;

extern flag_t chassis_flag;
extern chassis_move_t chassis_move;

	

#endif

