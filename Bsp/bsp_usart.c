#include "bsp_usart.h"
#include "cmsis_os.h"
#include "chassis_task.h"
#define GYRO_HUART huart1
#define LASER_huart huart2
#define LASER2_huart huart3
#define VISION_huart huart5
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart5;
extern DMA_HandleTypeDef hdma_usart1_rx;
extern DMA_HandleTypeDef hdma_usart2_rx;
extern DMA_HandleTypeDef hdma_usart3_rx;
extern DMA_HandleTypeDef hdma_uart5_rx;
static uint8_t USART1_rx_buf[USART_BUF_SIZE]={0};
static uint8_t usart2_rx_buf[USART_BUF_SIZE]={0};
static uint8_t usart3_rx_buf[USART_BUF_SIZE]={0};
static uint8_t usart5_rx_buf[USART_BUF_SIZE]={0};

float gyro_buf[2] = {0}; //0:Angle 1:Palstance
float vision_buf[2];
uint16_t laser_TF_buf[2];
float laser_buf[4];
uint16_t laser_tof_buf[4];

extern flag_t chassis_flag;
extern bool work_flag;
extern uint16_t route;

int count1=0,count2=0,count3=0,count4=0;
float buf1=0,buf2=0,buf4=0;
int buf3;
/**
  * @brief          激光数据处理
  * @param[in]      buf: 串口接收的数组
  * @param[in]      len: 串口接收数据的长度
  * @retval         none
  */
static void TOF_RX_Calc(uint8_t *buf, uint32_t len);
/**
  * @brief          陀螺仪数据处理
  * @param[in]      buf: 串口接收的数组
  * @param[in]      len: 串口接收数据的长度
  * @retval         none
  */
static void WT_RX_Calc(uint8_t *buf, uint32_t len);
/**
  * @brief          视觉数据处理
  * @param[in]      buf: 串口接收的数组
  * @param[in]      len: 串口接收数据的长度
  * @retval         none
  */
static void VISION_RX_Calc(uint8_t *buf, uint32_t len);
static void TOF_RX_Calc_Normal(uint8_t *buf);

//串口第一次DMA使能
void TFmini_init(void)
{
		__HAL_UART_ENABLE_IT(&LASER_huart, UART_IT_RXNE);
		__HAL_UART_ENABLE_IT(&LASER_huart, UART_IT_IDLE);
		HAL_UART_Receive_IT(&LASER_huart, usart2_rx_buf, USART_BUF_SIZE);
		HAL_UART_Receive_DMA(&LASER_huart, usart2_rx_buf, USART_BUF_SIZE);
	
		__HAL_UART_ENABLE_IT(&LASER2_huart, UART_IT_RXNE);
		__HAL_UART_ENABLE_IT(&LASER2_huart, UART_IT_IDLE);
		HAL_UART_Receive_IT(&LASER2_huart, usart3_rx_buf, USART_BUF_SIZE);
		HAL_UART_Receive_DMA(&LASER2_huart, usart3_rx_buf, USART_BUF_SIZE);
}
void WT101_init(void)
{
		__HAL_UART_ENABLE_IT(&GYRO_HUART, UART_IT_IDLE);
		HAL_UART_Receive_DMA(&GYRO_HUART, USART1_rx_buf, USART_BUF_SIZE);
}
void Vision_init(void)
{
		__HAL_UART_ENABLE_IT(&VISION_huart, UART_IT_RXNE);
		__HAL_UART_ENABLE_IT(&VISION_huart, UART_IT_IDLE);
		HAL_UART_Receive_IT(&VISION_huart, usart5_rx_buf, USART_BUF_SIZE);
		HAL_UART_Receive_DMA(&VISION_huart, usart5_rx_buf, USART_BUF_SIZE);
}





void USART_RX_Callback(UART_HandleTypeDef *huart)
{
	
  uint32_t tmp = 0;
	
	if(huart == &GYRO_HUART)
	{
			if((__HAL_UART_GET_FLAG(&GYRO_HUART,UART_FLAG_IDLE) != RESET))
			{
					__HAL_UART_CLEAR_IDLEFLAG(&GYRO_HUART);

					HAL_UART_DMAStop(&GYRO_HUART);//关闭DMA
					tmp =  USART_BUF_SIZE - hdma_usart1_rx.Instance->NDTR;//传输的字节数

					WT_RX_Calc(USART1_rx_buf, tmp);

					memset(USART1_rx_buf,0,tmp);//清零
					HAL_UART_Receive_DMA(&GYRO_HUART, USART1_rx_buf, USART_BUF_SIZE);//重新使能DMA
			}
	}
	
	
	if(huart == &LASER_huart)
	{
		if((__HAL_UART_GET_FLAG(&LASER_huart,UART_FLAG_IDLE) != RESET))
		{
				__HAL_UART_CLEAR_IDLEFLAG(&LASER_huart);

				HAL_UART_DMAStop(&LASER_huart);//关闭DMA
				tmp =  USART_BUF_SIZE - hdma_usart2_rx.Instance->NDTR;//传输的字节数
			
				TOF_RX_Calc(usart2_rx_buf, tmp);
			
				memset(usart2_rx_buf,0,tmp);//清零
				HAL_UART_Receive_DMA(&LASER_huart, usart2_rx_buf, USART_BUF_SIZE);//重新使能DMA	
		}
		
		
//	  if((__HAL_UART_GET_FLAG(&LASER_huart,UART_FLAG_RXNE) != RESET))
//    {
//				tmp =  USART_BUF_SIZE - hdma_usart2_rx.Instance->NDTR;//传输的字节数
//        TOF_RX_Calc(usart2_rx_buf, tmp);
//			
//				memset(usart2_rx_buf,0,tmp);//清零
//			
//				HAL_UART_Receive_IT(&LASER_huart, usart2_rx_buf, USART_BUF_SIZE);
//    }
	}
	
		if(huart == &LASER2_huart)
	{
		if((__HAL_UART_GET_FLAG(&LASER2_huart,UART_FLAG_IDLE) != RESET))
		{
				__HAL_UART_CLEAR_IDLEFLAG(&LASER2_huart);

				HAL_UART_DMAStop(&LASER2_huart);//关闭DMA
				tmp =  USART_BUF_SIZE - hdma_usart3_rx.Instance->NDTR;//传输的字节数
			
				TOF_RX_Calc_Normal(usart3_rx_buf);
				
				memset(usart3_rx_buf,0,tmp);//清零
				HAL_UART_Receive_DMA(&LASER2_huart, usart3_rx_buf, USART_BUF_SIZE);//重新使能DMA
			}
		
//		if((__HAL_UART_GET_FLAG(&VISION_huart,UART_FLAG_RXNE) != RESET))
//    {
//				tmp =  USART_BUF_SIZE - hdma_usart3_rx.Instance->NDTR;//传输的字节数
//        TOF_RX_Calc(usart3_rx_buf, tmp);
//			
//				memset(usart3_rx_buf,0,tmp);//清零
//			
//				HAL_UART_Receive_IT(&VISION_huart, usart3_rx_buf, USART_BUF_SIZE);
//    }
	}
	
	
	if(huart == &VISION_huart)
	{
		if((__HAL_UART_GET_FLAG(&VISION_huart,UART_FLAG_IDLE) != RESET))
		{
				__HAL_UART_CLEAR_IDLEFLAG(&VISION_huart);

				HAL_UART_DMAStop(&VISION_huart);//关闭DMA
				tmp =  USART_BUF_SIZE - hdma_uart5_rx.Instance->NDTR;//传输的字节数
			
				VISION_RX_Calc(usart5_rx_buf, tmp);
				
				memset(usart5_rx_buf,0,tmp);//清零
				HAL_UART_Receive_DMA(&VISION_huart, usart5_rx_buf, USART_BUF_SIZE);//重新使能DMA
			}
		
//		if((__HAL_UART_GET_FLAG(&VISION_huart,UART_FLAG_RXNE) != RESET))
//    {
//				tmp =  USART_BUF_SIZE - hdma_usart3_rx.Instance->NDTR;//传输的字节数
//        TOF_RX_Calc(usart3_rx_buf, tmp);
//			
//				memset(usart3_rx_buf,0,tmp);//清零
//			
//				HAL_UART_Receive_IT(&VISION_huart, usart3_rx_buf, USART_BUF_SIZE);
//    }
	}
	
	
	
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
								laser_TF_buf[Strength] = buf[4] | (buf[5] << 8);
								if( laser_TF_buf[Strength] > 100 && laser_TF_buf[Strength] != 65535)
									laser_TF_buf[Dist] = buf[2] | (buf[3] << 8);
								else 
									laser_TF_buf[Dist] = 0;
								
								laser_buf[Forward] = (float)laser_TF_buf[Dist];
            }
        }
    }
		
		if(TOF_DATA_LEN == len)
		{
			if(buf[0] == TOF_ID_00 && buf[1] == TOF_ORDER_NUMBER)
			{
				laser_tof_buf[0] = buf[3]<<8 | buf[4];	
				
				count1++;
				buf1 += (float)laser_tof_buf[0]/5.0f;
				if(count1 == 5)
				{
					laser_buf[Left] = buf1/10.0f;
					buf1 = 0;
					count1 = 0;
				}
			}			
			if(buf[0] == TOF_ID_01 && buf[1] == TOF_ORDER_NUMBER)
			{
				laser_tof_buf[1] = buf[3]<<8 | buf[4];		
				
				count2++;
				buf2 += (float)laser_tof_buf[1]/5.0f;
				if(count2 == 5)
				{
					laser_buf[Back] = buf2/10.0f;
					buf2 = 0;
					count2 = 0;
				}
			}
			if(buf[0] == TOF_ID_02 && buf[1] == TOF_ORDER_NUMBER)
			{
				laser_tof_buf[Right] = buf[3]<<8 | buf[4];		
				
				count4++;
				buf4 += (float)laser_tof_buf[Right]/5.0f;
				if(count4 == 5)
				{
					laser_buf[Right] = buf4/10.0f;
					buf4 = 0;
					count4 = 0;
				}

			}
		}
}

static void TOF_RX_Calc_Normal(uint8_t *buf)
{
	uint8_t *p;
	int Data=0;
	
	p = (uint8_t*)strstr((char*)buf,"d:");
	if(p == NULL) return;
	
		while(*p!='m')
		{
			if(*p>='0' && *p<='9')
				Data = Data*10 + (*p-'0');
			p++;		
		}
		
		count3++;
		buf3 += Data/10.0f;//均值
		if(count3 == 10)
		{
			laser_buf[Forward] = buf3/10.0f;//单位
			buf3 = 0;
			count3 = 0;
		}
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
            }
        }
    }
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
		
		if(vision_buf[0] > 0 && vision_buf[1] > 0)
				chassis_flag.vision = true;
			
		if(!chassis_flag.ready)
			chassis_flag.vision = false;
		
	if(chassis_flag.vision == true)
	{
		if(route == 3 || route == 9 || route == 15 || route == 6 || route == 12)
			chassis_flag.vision = true;
		else 
			chassis_flag.vision = false;
	}
}


