#include "bsp_vision.h"
#include "chassis_task.h"

#define VISION_huart huart3
extern UART_HandleTypeDef huart3;
extern DMA_HandleTypeDef hdma_usart3_rx;
static uint8_t usart3_rx_buf[USART_BUF_SIZE]={0};
float vision_buf[2];
extern flag_t chassis_flag;
extern bool work_flag;

static void VISION_RX_Calc(uint8_t *buf, uint32_t len);


//串口第一次DMA使能
void Vision_init(void)
{
		__HAL_UART_ENABLE_IT(&VISION_huart, UART_IT_RXNE);
		__HAL_UART_ENABLE_IT(&VISION_huart, UART_IT_IDLE);
		HAL_UART_Receive_IT(&VISION_huart, usart3_rx_buf, USART_BUF_SIZE);
		HAL_UART_Receive_DMA(&VISION_huart, usart3_rx_buf, USART_BUF_SIZE);
}


/**
  * @brief This function handles USART2 global interrupt.
  */
void USART3_IRQHandler(void)
{
	
  uint32_t tmp = 0;

  if((__HAL_UART_GET_FLAG(&VISION_huart,UART_FLAG_IDLE) != RESET))
  {
      __HAL_UART_CLEAR_IDLEFLAG(&VISION_huart);

      HAL_UART_DMAStop(&VISION_huart);//关闭DMA
      tmp =  USART_BUF_SIZE - hdma_usart3_rx.Instance->NDTR;//传输的字节数
		
			VISION_RX_Calc(usart3_rx_buf, tmp);

			if(work_flag == true)
				if(vision_buf[0] > 0 && vision_buf[1] > 0)
					chassis_flag.vision = true;
				
			if(vision_buf[0] == 0 && vision_buf[1] == 0)
				chassis_flag.vision = false;
					
			memset(usart3_rx_buf,0,tmp);//清零
      HAL_UART_Receive_DMA(&VISION_huart, usart3_rx_buf, USART_BUF_SIZE);//重新使能DMA
		
	}
		
//	if((__HAL_UART_GET_FLAG(&VISION_huart,UART_FLAG_RXNE) != RESET))
//	{
//		VISION_RX_Calc(usart3_rx_buf, tmp);
//		
//		if(work_flag == true)
//			chassis_flag.vision = true;
//		
//		memset(usart3_rx_buf,0,tmp);//清零
//		
//		HAL_UART_Receive_IT(&VISION_huart, usart3_rx_buf, USART_BUF_SIZE);
//	}
	
  HAL_UART_IRQHandler(&VISION_huart);
}

static void VISION_RX_Calc(uint8_t *buf, uint32_t len)
{
	  if(VISION_DATA_Len == len)
    {
        if(VISION_DATA_HEAD == buf[0])
        {
						vision_buf[0] = (float)(buf[1]<<8 | buf[2]);
						vision_buf[1] = (float)(buf[3]<<8 | buf[4]);
        }
    }
}

