#include "cmsis_os.h"
#include "math.h"

#include "chassis_task.h"
#include "laser_task.h"
#include "bsp_usart.h"
#include "math_calc.h"
#include "pid.h"
#include "bsp_vision.h"
#include "plan_task.h"


extern TIM_HandleTypeDef htim8;
chassis_move_t chassis_move;								//��������
flag_t chassis_flag;
bool ones = true;


/**
  * @brief          ��ʼ�����Ʋ���
  * @param[out]     chassis_move_control_loop:"chassis_move"����ָ��.
  * @retval         none
  */
static void chassis_init(chassis_move_t *chassis_move_init);

/**
  * @brief          ���õ��̿���ģʽ����Ҫ��'chassis_behaviour_mode_set'�����иı�
  * @param[out]     chassis_move_mode:"chassis_move"����ָ��.
  * @retval         none
  */
static void chassis_set_mode(chassis_move_t *chassis_move_mode);
	
/**
  * @brief          ����ģʽ�ı䣬��Щ������Ҫ�ı䣬������̿���yaw�Ƕ��趨ֵӦ�ñ�ɵ�ǰ����yaw�Ƕ�
  * @param[out]     chassis_move_transit:"chassis_move"����ָ��.
  * @retval         none
  */
static void chassis_mode_change_control_transit(chassis_move_t *chassis_move_transit);

/**
  * @brief          ���̲������ݸ��£���������ٶȣ�ŷ���Ƕȣ��������ٶ�
  * @param[out]     chassis_move_update:"chassis_move"����ָ��.
  * @retval         none
  */
static void chassis_feedback_update(chassis_move_t *chassis_move_update);

/**
  * @brief          ���õ��̿�������ֵ, ���˶�����ֵ��ͨ��chassis_behaviour_control_set�������õ�
  * @param[out]     chassis_move_update:"chassis_move"����ָ��.
  * @retval         none
  */
static void chassis_set_contorl(chassis_move_t *chassis_move_control);

/**
  * @brief          �ĸ������ٶ���ͨ�������������������
  * @param[in]      vx_set: �����ٶ�
  * @param[in]      vy_set: �����ٶ�
  * @param[in]      wz_set: ��ת�ٶ�
  * @param[out]     wheel_speed: �ĸ������ٶ�
  * @retval         none
  */
static void chassis_vector_to_mecanum_wheel_speed(const float vx_set, const float vy_set, const float wz_set, float wheel_speed[4]);

/**
  * @brief          ����ѭ�������ݿ����趨ֵ������������ֵ�����п���
  * @param[out]     chassis_move_control_loop:"chassis_move"����ָ��.
  * @retval         none
  */
static void chassis_control_loop(chassis_move_t *chassis_move_control_loop);

/**
  * @brief          PID������ʼ��
  * @param[out]     chassis_move_control_loop:"chassis_move"����ָ��.
  * @retval         none
  */
static void PID_param_init(chassis_move_t *chassis_pid_init);


void ChassisTask(void const * argument)
{
	
		vTaskDelay(CHASSIS_TASK_INIT_TIME);
		chassis_init(&chassis_move);
	  while (1)
		{
        //���õ��̿���ģʽ
        chassis_set_mode(&chassis_move);
        //ģʽ�л����ݱ���
        chassis_mode_change_control_transit(&chassis_move);
        //�������ݸ���
        chassis_feedback_update(&chassis_move);
        //���̿���������
        chassis_set_contorl(&chassis_move);
        //���̿���PID����
        chassis_control_loop(&chassis_move);
				//���Ƶ��
				Motor_Control(chassis_move.motor_chassis[0].give_current, chassis_move.motor_chassis[1].give_current,
										chassis_move.motor_chassis[2].give_current, chassis_move.motor_chassis[3].give_current);
				Guide_Motor_Control(chassis_move.motor_guide.give_current);
				//ϵͳ��ʱ
        vTaskDelay(CHASSIS_CONTROL_TIME_MS);
		}

}

static void PID_param_init(chassis_move_t *chassis_pid_init)
{
	//��ʼ���ٶȻ�PID 
	for(int8_t i=0; i<4; i++)
	{
		PID_struct_init(&chassis_pid_init->motor_speed_pid[i], POSITION_PID, 2000.0f, 2000.0f, 3300.0f, 450.0f, 0.0f);//3700 600
	}
	
	PID_struct_init(&chassis_pid_init->guide_speed_pid, POSITION_PID, 2000.0f, 370.0f, 1500.0f, 110.0f, 0.0f);
	PID_struct_init(&chassis_pid_init->guide_length_pid, POSITION_PID, 1.0f, 1.0f, 8.0f, 0.0f, 0.01f);
	
	//��ʼ���ǶȻ�PID 
	PID_struct_init(&chassis_pid_init->chassis_angle_pid, POSITION_PID, 1.0f, 1.0f, 0.01f, 0.0f, 0.0f);//0.008f, 0.0f, 0.005f
	
	//��ʼ����ǽ�ǶȻ�PID 
	PID_struct_init(&chassis_pid_init->chassis_laser_wall_pid, POSITION_PID, 1.0f, 1.0f, 0.02f, 0.0f, 0.0f);//1.0f, 1.0f, 0.02f, 0.0f, 0.0f
	
	//��ʼ���Ӿ�PID
	PID_struct_init(&chassis_pid_init->vision_pid, POSITION_PID, 1.0f, 1.0f, 0.003f, 0.0f, 0.0f);		//0.0015f, 0.0f, 0.0f
}


static void chassis_init(chassis_move_t *chassis_move_init)
{
	Motor_init();
	PID_param_init(chassis_move_init);
	
	//��ȡ������ָ��
	chassis_move_init->chassis_INS_angle = gyro_buf;
	
	//��ȡ����ָ��
	chassis_move_init->chassis_laser = laser_buf;
	
	//��ȡ�Ӿ�ָ��
	chassis_move_init->vision_data = vision_buf;
	
	//��ȡ���̵������ָ��
	for(int8_t i=0; i<4; i++)
	{
		chassis_move_init->motor_chassis[i].chassis_motor_measure = &motor_rxbuf[i];
	}
	
	chassis_move_init->motor_guide.chassis_motor_measure = &motor_rxbuf[4];
	chassis_move_init->guide_length = 0;
	chassis_move_init->guide_length_set = 0;
	
	//��ʼ������ģʽ
	chassis_flag.init = true;
	chassis_move_init->chassis_mode = CHASSIS_INIT;

	//�����˶��ٶ��޷�
	chassis_move_init->vx_max_speed = NORMAL_MAX_CHASSIS_SPEED_X;
	chassis_move_init->vx_min_speed = -NORMAL_MAX_CHASSIS_SPEED_X;
	chassis_move_init->vy_max_speed = NORMAL_MAX_CHASSIS_SPEED_Y;
	chassis_move_init->vy_min_speed = -NORMAL_MAX_CHASSIS_SPEED_Y;
	
	//��ʼ����ʼ�Ƕ�
	chassis_move_init->chassis_yaw = chassis_move_init->chassis_INS_angle[Angle];
	
	

	Servo_Control(&htim8, TIM_CHANNEL_1, 175);
	Servo_Control(&htim8, TIM_CHANNEL_2, 10);


	chassis_flag.vision = false;
	chassis_flag.work = false;
	chassis_flag.ready = true;
	chassis_flag.iteration = false;
}

static void chassis_set_mode(chassis_move_t *chassis_move_mode)
{
    if (chassis_move_mode == NULL)
    {
        return;
    }
		
		
		chassis_move_mode->last_chassis_mode = chassis_move_mode->chassis_mode;
		
		//�յ��Ӿ��������Ӿ�ģʽ
		if(chassis_flag.vision)
		{
			chassis_move_mode->chassis_mode = CHASSIS_VISION;
			return;
		}
		
		//��ʼ���ɹ���������̽Ƕȿ���
		if(chassis_flag.init && chassis_move_mode->last_chassis_mode != CHASSIS_INIT)
		{
			chassis_move_mode->chassis_mode = CHASSIS_INIT;
		}
		
		//����ǰ���ƶ�
		else if(chassis_flag.forward && chassis_move_mode->last_chassis_mode != CHASSIS_FORWARD)
		{
			chassis_move_mode->chassis_mode = CHASSIS_FORWARD;
		}
		
		//�������ƶ�
		else if(chassis_flag.right && chassis_move_mode->last_chassis_mode != CHASSIS_RIGHT)
		{
			chassis_move_mode->chassis_mode = CHASSIS_RIGHT;
		}
		
		else if(chassis_flag.left && chassis_move_mode->last_chassis_mode != CHASSIS_LEFT)
		{
			chassis_move_mode->chassis_mode = CHASSIS_LEFT;
		}
		
		else if(chassis_flag.back && chassis_move_mode->last_chassis_mode != CHASSIS_BACK)
		{
			chassis_move_mode->chassis_mode = CHASSIS_BACK;
		}
		
		else if(chassis_flag.stop && chassis_move_mode->last_chassis_mode != CHASSIS_STOP)
		{
			chassis_move_mode->chassis_mode = CHASSIS_STOP;
		}
}


static void chassis_mode_change_control_transit(chassis_move_t *chassis_move_transit)
{
    if (chassis_move_transit == NULL)
    {
			return;
    }

    if (chassis_move_transit->last_chassis_mode == chassis_move_transit->chassis_mode)
    {
      return;
    }

		else if(chassis_move_transit->last_chassis_mode != CHASSIS_STOP && chassis_move_transit->chassis_mode == CHASSIS_STOP)
		{
			chassis_move_transit->chassis_yaw_set = chassis_move_transit->chassis_yaw;
		}
		
    //���������̽Ƕ�ģʽ
    else if (chassis_move_transit->last_chassis_mode != CHASSIS_FORWARD && chassis_move_transit->chassis_mode == CHASSIS_FORWARD)
    {
        chassis_move_transit->chassis_yaw_set = chassis_move_transit->chassis_yaw;
    }

}

static void chassis_feedback_update(chassis_move_t *chassis_move_update)
{
    if (chassis_move_update == NULL)
    {
        return;
    }
		
    uint8_t i = 0;
    for (i = 0; i < 4; i++)
    {
        //���µ���ٶȣ����ٶ����ٶȵ�PID΢��
        chassis_move_update->motor_chassis[i].speed = chassis_move_update->motor_chassis[i].chassis_motor_measure->err_ecd * CHASSIS_MOTOR_COUNT_TO_SPE / GET_ENCODER_TIME; 
        chassis_move_update->motor_chassis[i].accel = chassis_move_update->motor_speed_pid[i].d_error * GET_ENCODER_TIME;
    }
		
		chassis_move_update->motor_guide.speed = chassis_move_update->motor_guide.chassis_motor_measure->err_ecd * GUIDE_MOTOR_COUNT_TO_SPE / GET_ENCODER_TIME; 
		
    //���µ��������ٶ� x�� ƽ���ٶ�y����ת�ٶ�wz������ϵΪ����ϵ
    chassis_move_update->vx = (-chassis_move_update->motor_chassis[0].speed + chassis_move_update->motor_chassis[1].speed + chassis_move_update->motor_chassis[2].speed - chassis_move_update->motor_chassis[3].speed) * MOTOR_SPEED_TO_CHASSIS_SPEED_VX;
    chassis_move_update->vy = (-chassis_move_update->motor_chassis[0].speed - chassis_move_update->motor_chassis[1].speed + chassis_move_update->motor_chassis[2].speed + chassis_move_update->motor_chassis[3].speed) * MOTOR_SPEED_TO_CHASSIS_SPEED_VY;
    chassis_move_update->wz = (-chassis_move_update->motor_chassis[0].speed - chassis_move_update->motor_chassis[1].speed - chassis_move_update->motor_chassis[2].speed - chassis_move_update->motor_chassis[3].speed) * MOTOR_SPEED_TO_CHASSIS_SPEED_WZ / MOTOR_DISTANCE_TO_CENTER;

    //���������̬�Ƕ�
    chassis_move_update->chassis_yaw = *(chassis_move_update->chassis_INS_angle);

}

static void chassis_set_contorl(chassis_move_t *chassis_move_control)
{

    if (chassis_move_control == NULL)
    {
        return;
    }

    float vx_set = 0.0f, vy_set = 0.0f, wz_set = 0.0f, angle_set = 0.0f, delat_angle = 00.0f;
		
		//���õ��̿��ƵĽǶ�
		if(route <= 1)
			angle_set = 0;
		else
			angle_set = -90.0f;
		
		//������ת�Ľ��ٶ�
    chassis_move_control->chassis_yaw_set = angle_set;
    delat_angle = chassis_move_control->chassis_yaw_set - chassis_move_control->chassis_yaw;
		wz_set = pid_calc(&chassis_move_control->chassis_angle_pid, delat_angle, 0.0f);
		
		//�ٶ��޷�
		abs_limit(&wz_set, 0.4, 0);		//0.3
    chassis_move_control->wz_set = wz_set;
		
		if (chassis_move_control->chassis_mode == CHASSIS_STOP || chassis_move_control->chassis_mode == CHASSIS_INIT)
		{								
			
			  chassis_move_control->vx_set = 0;
        chassis_move_control->vy_set = 0;
			
		}
		
    else if (chassis_move_control->chassis_mode == CHASSIS_FORWARD)
    {
//				if(!(chassis_Distance_Detection(chassis_move.guide_length, 0, 0.01) && chassis_move.guide_length_set == 0))return;
			
				//��ȡ������������ֵ
//				vx_set = 1.3f;
//				if(route != 1)
//					vx_set = 0.8f;
				vx_set = 1.2f;
				if(route != 1)
					vx_set = 1.0f;		//1.0
				
				if(!infared_LF)
				{
					if(route == 1)
						vx_set = -pid_calc(&chassis_move_control->chassis_laser_wall_pid, chassis_move_control->chassis_laser[Forward], LASER_DISTANCE_TO_WALL+3);
					else
						vx_set = -pid_calc(&chassis_move_control->chassis_laser_wall_pid, chassis_move_control->chassis_laser[Forward], LASER_DISTANCE_TO_WALL+2);
					abs_limit(&vx_set, 0.3, 0);
				}
			
				//������ǽ�ľ������
				vy_set = 0;
				if(route == 1 && chassis_move_control->chassis_laser[Left] <= (LASER_DISTANCE_TO_WALL+20))//��ʼ��ǽ
					vy_set = pid_calc(&chassis_move_control->chassis_laser_wall_pid, chassis_move_control->chassis_laser[Left], LASER_DISTANCE_TO_WALL+5);
			
				if(route != 1 && chassis_move_control->chassis_laser[Right] <= (LASER_DISTANCE_TO_WALL+20))//��¢��ǽ
					vy_set = -pid_calc(&chassis_move_control->chassis_laser_wall_pid, chassis_move_control->chassis_laser[Right], LASER_DISTANCE_TO_WALL+3);
				
    }
		
		else if (chassis_move_control->chassis_mode == CHASSIS_BACK)
    {

//			if(!(chassis_Distance_Detection(chassis_move.guide_length, 0, 0.01) && chassis_move.guide_length_set == 0))return;
			
				//��ȡ������������ֵ 
				vx_set = -1.0f;
				if(route != 18)
					vx_set = -1.0f;			//1.0
				
				if(!infared_LB)
				{
					vx_set = pid_calc(&chassis_move_control->chassis_laser_wall_pid, chassis_move_control->chassis_laser[Back], LASER_DISTANCE_TO_WALL+2);
					abs_limit(&vx_set, 0.3, 0);
				}
				
				//������ǽ�ľ������
				vy_set = 0;
				if(chassis_move_control->chassis_laser[Right] <= (LASER_DISTANCE_TO_WALL+20) && route!=18)//��¢��ǽ
					vy_set = -pid_calc(&chassis_move_control->chassis_laser_wall_pid, chassis_move_control->chassis_laser[Right], LASER_DISTANCE_TO_WALL+3);
				
		    if(chassis_move_control->chassis_laser[Right] <= (LASER_DISTANCE_TO_WALL+20) && route==18)//����¢��ǽ
					vy_set = -pid_calc(&chassis_move_control->chassis_laser_wall_pid, chassis_move_control->chassis_laser[Right], LASER_DISTANCE_TO_WALL+6);
							
    }
		
		else if (chassis_move_control->chassis_mode == CHASSIS_RIGHT)
		{
				//��ȡ������������ֵ
				vy_set = 0.7;
				if(route == 16)
					vy_set = 0.6f;
//				if(route == 16  && (chassis_move_control->chassis_laser[Right] <= (LASER_DISTANCE_TO_WALL+15)))//��ֹ��������(�����һ��¢��ʱ)
//					vy_set = -pid_calc(&chassis_move_control->chassis_laser_wall_pid, chassis_move_control->chassis_laser[Right], LASER_DISTANCE_TO_WALL+5);//+6
				
				//������¢���ľ�����ƣ����������ǽ
				vx_set = 0;
				if(!infared_LF)
					vx_set = -pid_calc(&chassis_move_control->chassis_laser_wall_pid, chassis_move_control->chassis_laser[Forward], LASER_DISTANCE_TO_WALL);

				if(!infared_LB)
					vx_set = pid_calc(&chassis_move_control->chassis_laser_wall_pid, chassis_move_control->chassis_laser[Back], LASER_DISTANCE_TO_WALL);
		}
		
		else if (chassis_move_control->chassis_mode == CHASSIS_LEFT)
		{
				//��ȡ������������ֵ
				vy_set = -0.6f;
				
				//������¢���ľ������
				vx_set = 0;

		}
		
		else if (chassis_move_control->chassis_mode == CHASSIS_VISION)
		{
				//��ȡ������������ֵ
				vx_set = 0;
				vx_set = pid_calc(&chassis_move_control->vision_pid, chassis_move_control->vision_data[0], 340);
				abs_limit(&vx_set, 0.6, 0);
			
			
				//������¢���ľ������
				vy_set = 0;
				if(chassis_move_control->chassis_laser[Right] <= (LASER_DISTANCE_TO_WALL+30))
					vy_set = -pid_calc(&chassis_move_control->chassis_laser_wall_pid, chassis_move_control->chassis_laser[Right], LASER_DISTANCE_TO_WALL-2);

					
//		//��׼ǰ�Ӿ���ʧ �����Ӿ�ģʽ
//		if(chassis_flag.ready && !chassis_flag.work)
//		{
//			if(chassis_move_control->vision_data[0] == 0)
//				chassis_flag.vision = false;
//		}
		
		//�ж϶�׼
		if(ones)
		{
			if(chassis_Speed_Detection(chassis_move_control->vx, 0) && chassis_Speed_Detection(chassis_move_control->vy, 0))
			{
				if(chassis_Distance_Detection(chassis_move_control->vision_data[0], 340, 5) && (chassis_move_control->vision_data[0] <= 380) && chassis_Distance_Detection(chassis_move_control->chassis_laser[Right], LASER_DISTANCE_TO_WALL-2, 0.5))
				{
					chassis_flag.work = true;
					ones = false;
				}
			}
		}	
		
			if(chassis_flag.work)
			{
				vx_set = 0;
				vy_set = 0;
				
				if(chassis_flag.iteration)
					chassis_move_control->guide_length_set = 0;
				else
					chassis_move_control->guide_length_set = GUIDE_LENGTH;
			}
		
		}
		
		//�ٶ��޷�
		abs_limit(&vx_set, chassis_move_control->vx_max_speed, 0);
		abs_limit(&vy_set, chassis_move_control->vy_max_speed, 0);
    chassis_move_control->vx_set = vx_set;
    chassis_move_control->vy_set = vy_set;
}

static void chassis_control_loop(chassis_move_t *chassis_move_control_loop)
{
    float max_vector = 0.0f, vector_rate = 0.0f;
    float temp = 0.0f;
    float wheel_speed[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    uint8_t i = 0;

    //�����˶��ֽ�
    chassis_vector_to_mecanum_wheel_speed(chassis_move_control_loop->vx_set,
                                          chassis_move_control_loop->vy_set, 
																					chassis_move_control_loop->wz_set, wheel_speed);


    //�������ӿ�������ٶȣ�������������ٶ�
    for (i = 0; i < 4; i++)
    {
        chassis_move_control_loop->motor_chassis[i].speed_set = wheel_speed[i];
        temp = fabs(chassis_move_control_loop->motor_chassis[i].speed_set);
        if (max_vector < temp)
        {
            max_vector = temp;
        }
    }

    if (max_vector > MAX_WHEEL_SPEED)
    {
        vector_rate = MAX_WHEEL_SPEED / max_vector;
        for (i = 0; i < 4; i++)
        {
            chassis_move_control_loop->motor_chassis[i].speed_set *= vector_rate;
        }
    }

		
    //����pid
    for (i = 0; i < 4; i++)
    {
        pid_calc(&chassis_move_control_loop->motor_speed_pid[i], chassis_move_control_loop->motor_chassis[i].speed, chassis_move_control_loop->motor_chassis[i].speed_set);
    }

		
    //��ֵ����ֵ
    for (i = 0; i < 4; i++)
    {
        chassis_move_control_loop->motor_chassis[i].give_current = (int16_t)(chassis_move_control_loop->motor_speed_pid[i].pos_out);
    }
		
		pid_calc(&chassis_move_control_loop->guide_length_pid, chassis_move_control_loop->guide_length, chassis_move_control_loop->guide_length_set);
		
		chassis_move_control_loop->motor_guide.speed_set = chassis_move_control_loop->guide_length_pid.pos_out;
		pid_calc(&chassis_move_control_loop->guide_speed_pid, chassis_move_control_loop->motor_guide.speed, chassis_move_control_loop->motor_guide.speed_set);
		chassis_move_control_loop->motor_guide.give_current = (int16_t)(chassis_move_control_loop->guide_speed_pid.pos_out);
		
}

static void chassis_vector_to_mecanum_wheel_speed(const float vx_set, const float vy_set, const float wz_set, float wheel_speed[4])
{
    //��ת��ʱ�� ������̨��ǰ��������ǰ������ 0 ��1 ��ת���ٶȱ����� �������� 2,3 ��ת���ٶȱ��
    wheel_speed[0] = vx_set + vy_set + (CHASSIS_WZ_SET_SCALE - 1.0f) * MOTOR_DISTANCE_TO_CENTER * wz_set;
    wheel_speed[1] = vx_set - vy_set + (-CHASSIS_WZ_SET_SCALE + 1.0f) * MOTOR_DISTANCE_TO_CENTER * wz_set;
    wheel_speed[2] = vx_set - vy_set + (CHASSIS_WZ_SET_SCALE - 1.0f) * MOTOR_DISTANCE_TO_CENTER * wz_set;
    wheel_speed[3] = vx_set + vy_set + (-CHASSIS_WZ_SET_SCALE + 1.0f) * MOTOR_DISTANCE_TO_CENTER * wz_set;
}

