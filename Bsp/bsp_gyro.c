#include "bsp_gyro.h"

#define GYRO_HUART huart1
extern UART_HandleTypeDef huart1;
extern DMA_HandleTypeDef hdma_usart1_rx;
static uint8_t USART1_rx_buf[USART_BUF_SIZE]={0};
float gyro_buf[2] = {0}; //0:Angle 1:Palstance


/**
  * @brief          陀螺仪数据处理
  * @param[in]      buf: 串口接收的数组
  * @param[in]      len: 串口接收数据的长度
  * @retval         none
  */
static void WT_RX_Calc(uint8_t *buf, uint32_t len);





//串口第一次DMA使能
void WT101_init(void)
{
		__HAL_UART_ENABLE_IT(&GYRO_HUART, UART_IT_IDLE);
		HAL_UART_Receive_DMA(&GYRO_HUART, USART1_rx_buf, USART_BUF_SIZE);
}
	
//获取陀螺仪数据
void Gyro_Data_Get(sensor_t *rxbuf)
{
	rxbuf->gyro.Palstance = gyro_buf[Palstance];
	rxbuf->gyro.Angle = gyro_buf[Angle];
}

/**
  * @brief This function handles USART1 global interrupt.
  */
void USART1_IRQHandler(void)
{
	
  uint32_t tmp = 0;

  if((__HAL_UART_GET_FLAG(&GYRO_HUART,UART_FLAG_IDLE) != RESET))
  {
      __HAL_UART_CLEAR_IDLEFLAG(&GYRO_HUART);

      HAL_UART_DMAStop(&GYRO_HUART);//关闭DMA
      tmp =  USART_BUF_SIZE - hdma_usart1_rx.Instance->NDTR;//传输的字节数
		
			WT_RX_Calc(USART1_rx_buf, tmp);
		
			memset(USART1_rx_buf,0,tmp);//清零
      HAL_UART_Receive_DMA(&GYRO_HUART, USART1_rx_buf, USART_BUF_SIZE);//重新使能DMA
		
	}
  HAL_UART_IRQHandler(&GYRO_HUART);

}

static void WT_RX_Calc(uint8_t *buf, uint32_t len)
{
    uint32_t i = 0;
    uint8_t Checksum_ang = 0;
		uint8_t Checksum_pal = 0;
	
    if(WT101_DATA_Len*2 == len)
    {
        if( WT101_DATA_HEAD == buf[0] && WT101_DATA_HEAD == buf[11] )
        {
            for(i = 0; i < (WT101_DATA_Len - 1); i++)
            {
                Checksum_pal += buf[i];
								Checksum_ang += buf[i+11];
            }

            if( Checksum_pal == buf[10] && Checksum_ang == buf[21] )
            {
                gyro_buf[Palstance] = ((short)(buf[7] << 8) | buf[6]);
								gyro_buf[Palstance] = gyro_buf[Palstance]/32768.0f*2000.0f;
								gyro_buf[Angle] = ((short)(buf[18] << 8) | buf[17]);
								gyro_buf[Angle] = gyro_buf[Angle]/32768.0f*180.0f;
							
								if(gyro_buf[Angle] > 45)
									gyro_buf[Angle] = -360.0f + gyro_buf[Angle];
            }
        }
    }
}


