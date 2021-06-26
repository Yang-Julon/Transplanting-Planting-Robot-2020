#include "bsp_echo.h"
#include "main.h"
#include "bsp_motor.h"
#define ECHO_htim htim8
trig_echo_t trig_echo[4];



void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{

	if(htim == &ECHO_htim)
	{
//		trig_echo_t *rx;
//		rx = trig_echo;
//				 if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1) rx=trig_echo;
//		else if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2) rx=trig_echo+1;
//		else if(htim->Channel==HAL_TIM_ACTIVE_CHANNEL_3) rx=trig_echo+2;
//		else if(htim->Channel==HAL_TIM_ACTIVE_CHANNEL_4) rx=trig_echo+3;
	
		for(int i=0; i<2; i++)
			if((trig_echo[i].CAPTURE_STA&0X80)==0 && trig_echo[i].CAPTURE_STA&0X40)
				trig_echo[i].CAPTURE_STA++;
	}
	
}


//定时器捕获中断
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	if(htim != &ECHO_htim)
	{
		return;
	}
	
	trig_echo_t *rx;
	uint32_t Channel_num;
	if(htim->Channel==HAL_TIM_ACTIVE_CHANNEL_1) rx=trig_echo,Channel_num=TIM_CHANNEL_1;
	else if(htim->Channel==HAL_TIM_ACTIVE_CHANNEL_2) rx=trig_echo+1,Channel_num=TIM_CHANNEL_2;
//	else if(htim->Channel==HAL_TIM_ACTIVE_CHANNEL_3) rx=trig_echo+2,Channel_num=TIM_CHANNEL_3;
//	else if(htim->Channel==HAL_TIM_ACTIVE_CHANNEL_4) rx=trig_echo+3,Channel_num=TIM_CHANNEL_4;
	
	if((rx->CAPTURE_STA&0X80)==0) //未捕获下降沿
	{
		if(rx->CAPTURE_STA&0X40)		 //已捕获上升沿
		{	  			
			rx->CAPTURE_STA|=0X80;    //标志下降沿
      rx->CAPTURE_VAL_END = HAL_TIM_ReadCapturedValue(&ECHO_htim,Channel_num);
			__HAL_TIM_SET_CAPTUREPOLARITY(&ECHO_htim,Channel_num,TIM_INPUTCHANNELPOLARITY_RISING);
			
		}else		//未捕获上升沿
		{
			rx->CAPTURE_STA|=0X40;    //标志上升沿
//			__HAL_TIM_SET_COUNTER(&ECHO_htim,0);
			rx->CAPTURE_VAL_START = HAL_TIM_ReadCapturedValue(&ECHO_htim,Channel_num);
			__HAL_TIM_SET_CAPTUREPOLARITY(&ECHO_htim,Channel_num,TIM_INPUTCHANNELPOLARITY_FALLING);

		}		    
	}
}

/**
  * @brief          发生两个超声波
  * @retval         none
  */
void trigProduce(void)
{
	HAL_GPIO_WritePin(TRIG1_GPIO_Port, TRIG1_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(TRIG2_GPIO_Port, TRIG2_Pin, GPIO_PIN_SET);
	delay_us(15);
	HAL_GPIO_WritePin(TRIG1_GPIO_Port, TRIG1_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(TRIG2_GPIO_Port, TRIG2_Pin, GPIO_PIN_RESET);
}

/**
  * @brief          超声波捕获数据处理
  * @retval         none
  */
void Echo_RX_Calc(void)
{
		for(int i=0; i<2; i++)
		{
			if(trig_echo[i].CAPTURE_STA&0X80)
			{
				trig_echo[i].time = (trig_echo[i].CAPTURE_STA&0X3F)*65536+(trig_echo[i].CAPTURE_VAL_END - trig_echo[i].CAPTURE_VAL_START); 
//				trig_echo[i].distance = trig_echo[i].time*0.17f;
//				trig_echo[i].CAPTURE_STA = 0;
//				trig_echo[i].CAPTURE_VAL_START = 0;
//				trig_echo[i].CAPTURE_VAL_END = 0;
				
				trig_echo[i].buf += trig_echo[i].time*0.17f/5.0f;
				trig_echo[i].CAPTURE_STA = 0;
				trig_echo[i].count ++;
				
				if(trig_echo[i].count == 5)
				{
					trig_echo[i].distance = trig_echo[i].buf;
					trig_echo[i].buf = 0;
					trig_echo[i].count = 0;
				}
			}
		}
}
