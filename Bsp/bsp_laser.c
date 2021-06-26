#include "bsp_laser.h"

#define LASER_huart huart2
extern UART_HandleTypeDef huart2;
extern DMA_HandleTypeDef hdma_usart2_rx;
static uint8_t usart2_rx_buf[USART_BUF_SIZE]={0};
uint16_t laser_buf[2];
uint16_t laser_tof_buf[2];

float laser_data[3];
int count1=0,count2=0;
float buf1=0,buf2=0;
/**
  * @brief          激光数据处理
  * @param[in]      buf: 串口接收的数组
  * @param[in]      len: 串口接收数据的长度
  * @retval         none
  */
static void TOF_RX_Calc(uint8_t *buf, uint32_t len);





//串口第一次DMA使能
void TFmini_init(void)
{
		__HAL_UART_ENABLE_IT(&LASER_huart, UART_IT_RXNE);
		__HAL_UART_ENABLE_IT(&LASER_huart, UART_IT_IDLE);
		HAL_UART_Receive_IT(&LASER_huart, usart2_rx_buf, USART_BUF_SIZE);
		HAL_UART_Receive_DMA(&LASER_huart, usart2_rx_buf, USART_BUF_SIZE);
}


/**
  * @brief This function handles USART2 global interrupt.
  */
void USART2_IRQHandler(void)
{
	
  uint32_t tmp = 0;

  if((__HAL_UART_GET_FLAG(&LASER_huart,UART_FLAG_IDLE) != RESET))
  {
      __HAL_UART_CLEAR_IDLEFLAG(&LASER_huart);

      HAL_UART_DMAStop(&LASER_huart);//关闭DMA
      tmp =  USART_BUF_SIZE - hdma_usart2_rx.Instance->NDTR;//传输的字节数
		
			TOF_RX_Calc(usart2_rx_buf, tmp);
		
			memset(usart2_rx_buf,0,tmp);//清零
      HAL_UART_Receive_DMA(&LASER_huart, usart2_rx_buf, USART_BUF_SIZE);//重新使能DMA
		

		
	}
	
	  if((__HAL_UART_GET_FLAG(&LASER_huart,UART_FLAG_RXNE) != RESET))
    {

        TOF_RX_Calc(usart2_rx_buf, tmp);
			
				memset(usart2_rx_buf,0,tmp);//清零
			
				HAL_UART_Receive_IT(&LASER_huart, usart2_rx_buf, USART_BUF_SIZE);

    }
		
  HAL_UART_IRQHandler(&LASER_huart);
}


static void TOF_RX_Calc(uint8_t *buf, uint32_t len)
{
    uint32_t i = 0;
    uint8_t Checksum = 0;
	
    if(TFMINI_DATA_Len == len)
    {
        if((TFMINT_DATA_HEAD == buf[0])&&(TFMINT_DATA_HEAD == buf[1]))
        {
            for(i = 0; i < (TFMINI_DATA_Len - 1); i++)
            {
                Checksum += buf[i];
            }

            if(Checksum == buf[TFMINI_DATA_Len - 1])
            {
								laser_buf[Strength] = buf[4] | (buf[5] << 8);
								if( laser_buf[Strength] > 100 && laser_buf[Strength] != 65535)
									laser_buf[Dist] = buf[2] | (buf[3] << 8);
								else 
									laser_buf[Dist] = 0;
								
								laser_data[Forward] = (float)laser_buf[Dist];
            }
        }
    }
		
		if(TOF_DATA_LEN == len)
		{
			if(buf[0] == TOF_ID_00 && buf[1] == TOF_ORDER_NUMBER)
			{
				laser_tof_buf[0] = buf[3]<<8 | buf[4];	
				count1++;
				buf1 += (float)laser_tof_buf[0];
				if(count1 == 5)
				{
					laser_data[Left] = buf1/5.0f;
					buf1 = 0;
					count1 = 0;
				}
			}			
			if(buf[0] == TOF_ID_01 && buf[1] == TOF_ORDER_NUMBER)
			{
				laser_tof_buf[1] = buf[3]<<8 | buf[4];		
				
				count2++;
				buf2 += (float)laser_tof_buf[1];
				if(count2 == 5)
				{
					laser_data[Back] = buf2/5.0f;
					buf2 = 0;
					count2 = 0;
				}
			}
			
		}
}
