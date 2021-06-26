#ifndef __CHASSIS_TASK_H_
#define __CHASSIS_TASK_H_

#include "pid.h"
#include "main.h"
#include "sensor.h"
#include "bsp_motor.h"
#include "stdbool.h"


#define CHASSIS_TASK_INIT_TIME							500												//���������ʼ��ʱ��
#define CHASSIS_CONTROL_TIME_MS							10												//���̿��Ƶ�ʱ��

#define MAX_WHEEL_SPEED 										4.0f											//�������̵������ٶ�	
#define NORMAL_MAX_CHASSIS_SPEED_X 					2.0f											//�����˶��������ǰ���ٶ�
#define NORMAL_MAX_CHASSIS_SPEED_Y 					1.5f											//�����˶��������ƽ���ٶ�
#define MOTOR_SPEED_TO_CHASSIS_SPEED_VX 		0.25f
#define MOTOR_SPEED_TO_CHASSIS_SPEED_VY 		0.25f
#define MOTOR_SPEED_TO_CHASSIS_SPEED_WZ			0.25f
#define CHASSIS_WZ_SET_SCALE							  2.0f
#define MOTOR_DISTANCE_TO_CENTER 					  1.0f
#define LASER_DISTANCE_TO_WALL							10.0f
#define GUIDE_LENGTH                        -0.168		//-0.16

#define GET_ENCODER_TIME										0.005f										//��ȡ������ʱ���� ��λs(��)
#define PI																	3.1415926535f

#define CHASSIS_MOTOR_COUNT_TO_SPE					(PI*WHEEL_RADIUS/8422.4f)	//����ֵת���ٶ�
#define WHEEL_RADIUS												0.035f										//���Ӱ뾶

#define GUIDE_MOTOR_COUNT_TO_SPE						(PI*GEAR_RADIUS/8422.4f)		//����ֵת���ٶ�
#define GEAR_RADIUS													0.02f											//���ְ뾶

#define infared_LF 	HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_1)
#define infared_RF 	HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_7)
#define infared_LB 	HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_5)
#define infared_RB 	HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_7)

typedef enum
{
	CHASSIS_INIT,
	CHASSIS_STOP,				 //����ֹͣ
  CHASSIS_FORWARD,	   //�����е��̽Ƕȿ��Ʊջ�
	CHASSIS_BACK,
  CHASSIS_LEFT,        //��������ת�ٶȿ���
	CHASSIS_RIGHT,
	CHASSIS_VISION,			 //����׷���Ӿ�
	
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
  const float *chassis_INS_angle;             //��ȡ�����ǽ������ŷ����ָ��
	const float *chassis_laser;
	const float *vision_data;
  chassis_mode_e chassis_mode;              	//���̿���״̬��
  chassis_mode_e last_chassis_mode;           //�����ϴο���״̬��
  chassis_motor_t motor_chassis[4];           //���̵������
	chassis_motor_t motor_guide;
	
  pid_t motor_speed_pid[4];            				//���̵���ٶ�pid
  pid_t chassis_angle_pid;             				//���̸���Ƕ�pid
	pid_t chassis_laser_wall_pid;
	pid_t vision_pid;
	pid_t guide_speed_pid;
	pid_t guide_length_pid;


  float vx;                          //�����ٶ� ǰ������ ǰΪ������λ m/s
  float vy;                          //�����ٶ� ���ҷ��� ��Ϊ��  ��λ m/s
  float wz;                          //������ת���ٶȣ���ʱ��Ϊ�� ��λ rad/s
  float vx_set;                      //�����趨�ٶ� ǰ������ ǰΪ������λ m/s
  float vy_set;                      //�����趨�ٶ� ���ҷ��� ��Ϊ������λ m/s
  float wz_set;                      //�����趨��ת���ٶȣ���ʱ��Ϊ�� ��λ rad/s
  float chassis_yaw_set;   
	float guide_length;
	float guide_length_set;
		
  float vx_max_speed;  //ǰ����������ٶ� ��λm/s
  float vx_min_speed;  //���˷�������ٶ� ��λm/s
  float vy_max_speed;  //��������ٶ� ��λm/s
  float vy_min_speed;  //�ҷ�������ٶ� ��λm/s
  float chassis_yaw;   //���̵�yaw�Ƕ�

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

