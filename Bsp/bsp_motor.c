#include "bsp_motor.h"
#include "tim.h"
#include "cmsis_os.h"
#include "math_calc.h"
#include "chassis_task.h"
extern osTimerId encoder_id;
extern chassis_move_t chassis_move;
motor_measure_t motor_rxbuf[5];
int8_t a=0;

/**
  * @brief          获取编码器数据
  * @retval         none
  */
static uint16_t Encoder_Counter_Get(TIM_HandleTypeDef *htim);
static void Encoder_Data_Get(void);
/**
  * @brief          电机正反转
	* @param[in]      motor:	电机
  * @param[in]      PinState: 正反转
  * @retval         none
  */
static void Motor_Direction(int8_t motor, GPIO_PinState PinState);
	
	
	
	
//开启编码器和PWM定时器
void Motor_init(void)
{
	HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start(&htim5, TIM_CHANNEL_ALL);

	//电机pwm
	HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim10, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_2);
	
	//舵机pwm
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_4);
	HAL_TIM_PWM_Start(&htim11, TIM_CHANNEL_1);
	
	HAL_TIM_Base_Start_IT(&htim6);
}



void TIM_Interrupt_Callback(TIM_HandleTypeDef *htim)
{
	
		static uint8_t tim6Cnt = 0;
		if(htim == &htim6)
		{
			//5ms(5ms * 1)一次读取编码器值
			if(++tim6Cnt != 1)
				return;
			tim6Cnt = 0;
			
			a++;
			Encoder_Data_Get();
		}
}


//底盘电机输出
void Motor_Control(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4)
{
	int16_t PWM[4] = {motor1, motor2, motor3, motor4};
	
	//高低电平控制正反转
	for(int8_t i=0; i<4; i++)
	{
		if(PWM[i] > 0)
		{
			Motor_Direction(i, GPIO_PIN_SET);
		}
		else
		{
			Motor_Direction(i, GPIO_PIN_RESET);
			PWM[i] = -PWM[i];
		}
	}

	__HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_1, PWM[0]);
	__HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_2, PWM[1]);
	__HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_1, PWM[2]);
	__HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_2, PWM[3]);
}

//上层电机输出
void Guide_Motor_Control(int16_t motor)
{
	if(motor > 0)
		{
			Motor_Direction(4, GPIO_PIN_SET);
		}
		else
		{
			Motor_Direction(4, GPIO_PIN_RESET);
			motor = -motor;
		}
		
	__HAL_TIM_SET_COMPARE(&htim10, TIM_CHANNEL_1, motor);
}


static void Encoder_Data_Get(void)
{
		//底盘电机
  	motor_rxbuf[0].last_ecd = motor_rxbuf[0].ecd;
  	motor_rxbuf[0].ecd = -ecd_calc(Encoder_Counter_Get(&htim2));
		motor_rxbuf[0].err_ecd = motor_rxbuf[0].ecd;

	
  	motor_rxbuf[1].last_ecd = motor_rxbuf[1].ecd;
  	motor_rxbuf[1].ecd = ecd_calc(Encoder_Counter_Get(&htim3));
		motor_rxbuf[1].err_ecd = motor_rxbuf[1].ecd;
	
		motor_rxbuf[2].last_ecd = motor_rxbuf[2].ecd;
		motor_rxbuf[2].ecd = -ecd_calc(Encoder_Counter_Get(&htim4));
		motor_rxbuf[2].err_ecd = motor_rxbuf[2].ecd;
	
  	motor_rxbuf[3].last_ecd = motor_rxbuf[3].ecd;
  	motor_rxbuf[3].ecd = ecd_calc(Encoder_Counter_Get(&htim5));
		motor_rxbuf[3].err_ecd = motor_rxbuf[3].ecd;
		
		//上层电机
	  motor_rxbuf[4].last_ecd = motor_rxbuf[4].ecd;
  	motor_rxbuf[4].ecd = ecd_calc(Encoder_Counter_Get(&htim1));
		motor_rxbuf[4].err_ecd = motor_rxbuf[4].ecd;
		chassis_move.guide_length += chassis_move.motor_guide.chassis_motor_measure->err_ecd * GUIDE_MOTOR_COUNT_TO_SPE;
		
		
}

static uint16_t Encoder_Counter_Get(TIM_HandleTypeDef *htim)
{
	uint32_t num;
	num =__HAL_TIM_GET_COUNTER(htim);
	__HAL_TIM_SET_COUNTER(htim, 0);
	return num;
}

static void Motor_Direction(int8_t motor, GPIO_PinState PinState)
{
	GPIO_PinState PinState_op = GPIO_PIN_RESET;
	if(PinState == PinState_op) PinState_op = GPIO_PIN_SET;
	
	//底盘电机
	if(motor == 0)
	{
	HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, PinState_op);
	HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, PinState);
	}
	else if(motor == 1)
	{
	HAL_GPIO_WritePin(BIN1_GPIO_Port, BIN1_Pin, PinState);
	HAL_GPIO_WritePin(BIN2_GPIO_Port, BIN2_Pin, PinState_op);
	}
	else if(motor == 2)
	{
	HAL_GPIO_WritePin(CIN1_GPIO_Port, CIN1_Pin, PinState_op);
	HAL_GPIO_WritePin(CIN2_GPIO_Port, CIN2_Pin, PinState);
	}
	else if(motor == 3)
	{
	HAL_GPIO_WritePin(DIN1_GPIO_Port, DIN1_Pin, PinState);
	HAL_GPIO_WritePin(DIN2_GPIO_Port, DIN2_Pin, PinState_op);
	}
	
	//上层电机
	else if(motor == 4)
	{
	HAL_GPIO_WritePin(EIN1_GPIO_Port, EIN1_Pin, PinState);
	HAL_GPIO_WritePin(EIN1_GPIO_Port, EIN2_Pin, PinState_op);
	}
}

int16_t PWM;
void Servo_Control(TIM_HandleTypeDef *htim, uint32_t channel, float angle)
{
//	int16_t PWM;
	PWM = (angle / 180.0f) * 2000.0f + 500.0f;
	__HAL_TIM_SET_COMPARE(htim, channel, PWM);
	
}

