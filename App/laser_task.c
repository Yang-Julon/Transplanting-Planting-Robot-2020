#include "laser_task.h"
#include "bsp_usart.h"

#define LASER_TASK_INIT_TIME 50
#define LASER_CONTROL_TIME   10


extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
uint8_t CMD_0[8]={0x00,0x03,0x00,0x34,0x00,0x01,0xc4,0x15};
uint8_t CMD_1[8]={0x01,0x03,0x00,0x34,0x00,0x01,0xc5,0xc4};
uint8_t CMD_2[8]={0x02,0x03,0x00,0x34,0x00,0x01,0xc5,0xf7};
void LaserTask(void const * argument)
{
	vTaskDelay(LASER_TASK_INIT_TIME);
	
	while(1)
	{
		while(HAL_UART_Transmit(&huart2, CMD_0, 8, 100)!= HAL_OK);
		while(HAL_UART_Transmit(&huart2, CMD_1, 8, 100)!= HAL_OK);
		while(HAL_UART_Transmit(&huart2, CMD_2, 8, 100)!= HAL_OK);
		vTaskDelay(LASER_CONTROL_TIME);
	}
}

