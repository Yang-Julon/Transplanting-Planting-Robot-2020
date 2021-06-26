//Channel：选择通道（1-10）
//传入单精度浮点数
//在函数的最下方的DataWave函数修改发送的数据
//其他函数无需修改
#include "DataScope_DP.h"
#include "math.h"
#include "main.h"
#include "usart.h"
#include "pid.h"
#include "chassis_task.h"

DataTypedfef CK;	//传输数据用到的结构体


//函数说明：将单精度浮点数据转成4字节数据并存入指定地址 
//附加说明：用户无需直接操作此函数 
//target:目标单精度数据
//buf:待写入数组
//beg:指定从数组第几个元素开始写入
//函数无返回 
void Float2Byte(float *target,unsigned char *buf,unsigned char beg)
{
    unsigned char *point;
    point = (unsigned char*)target;	  //得到float的地址
    buf[beg]   = point[0];
    buf[beg+1] = point[1];
    buf[beg+2] = point[2];
    buf[beg+3] = point[3];
}
 
 
//函数说明：将待发送通道的单精度浮点数据写入发送缓冲区
//Data：通道数据
//Channel：选择通道（1-10）
//函数无返回 
void DataScope_Get_Channel_Data(float Data,unsigned char Channel)
{
	if ( (Channel > 10) || (Channel == 0) ) return;  //通道个数大于10或等于0，直接跳出，不执行函数
  else
  {
     switch (Channel)
		{
      case 1:  Float2Byte(&Data,CK.DataScope_OutPut_Buffer,1); break;
      case 2:  Float2Byte(&Data,CK.DataScope_OutPut_Buffer,5); break;
		  case 3:  Float2Byte(&Data,CK.DataScope_OutPut_Buffer,9); break;
		  case 4:  Float2Byte(&Data,CK.DataScope_OutPut_Buffer,13); break;
		  case 5:  Float2Byte(&Data,CK.DataScope_OutPut_Buffer,17); break;
		  case 6:  Float2Byte(&Data,CK.DataScope_OutPut_Buffer,21); break;
		  case 7:  Float2Byte(&Data,CK.DataScope_OutPut_Buffer,25); break;
		  case 8:  Float2Byte(&Data,CK.DataScope_OutPut_Buffer,29); break;
		  case 9:  Float2Byte(&Data,CK.DataScope_OutPut_Buffer,33); break;
		  case 10: Float2Byte(&Data,CK.DataScope_OutPut_Buffer,37); break;
		}
  }	 
}


//函数说明：生成 DataScopeV1.0 能正确识别的帧格式
//Channel_Number，需要发送的通道个数
//返回发送缓冲区数据个数
//返回0表示帧格式生成失败 
unsigned char DataScope_Data_Generate(unsigned char Channel_Number)
{
	if ( (Channel_Number > 10) || (Channel_Number == 0) ) { return 0; }  //通道个数大于10或等于0，直接跳出，不执行函数
  else
  {	
	 CK.DataScope_OutPut_Buffer[0] = '$';  //帧头
		
	 switch(Channel_Number)   
   { 
		 case 1:   CK.DataScope_OutPut_Buffer[5]  =  5; return  6;  
		 case 2:   CK.DataScope_OutPut_Buffer[9]  =  9; return 10;
		 case 3:   CK.DataScope_OutPut_Buffer[13] = 13; return 14; 
		 case 4:   CK.DataScope_OutPut_Buffer[17] = 17; return 18;
		 case 5:   CK.DataScope_OutPut_Buffer[21] = 21; return 22;  
		 case 6:   CK.DataScope_OutPut_Buffer[25] = 25; return 26;
		 case 7:   CK.DataScope_OutPut_Buffer[29] = 29; return 30; 
		 case 8:   CK.DataScope_OutPut_Buffer[33] = 33; return 34; 
		 case 9:   CK.DataScope_OutPut_Buffer[37] = 37; return 38;
     case 10:  CK.DataScope_OutPut_Buffer[41] = 41; return 42; 
   }	 
  }
	return 0;
}

//函数说明：MiniBalance上位机通过串口打印数据波形
//附加说明：直接在主函数中调用此函数（注意延时减少打印量）
//函数无返回 
void DataWave(UART_HandleTypeDef* huart)
{	
		//Channel：选择通道（1-10）
		//传入单精度浮点数
	

//	DataScope_Get_Channel_Data((float) gimbal.pid.pit_ecd_fdb, 1 );
//	DataScope_Get_Channel_Data((float) gimbal.pid.pit_ecd_ref, 2 );
//  DataScope_Get_Channel_Data((float) gimbal.sensor.yaw_angle, 1 );//gimbal.pid.yaw_angle_fdb
//	DataScope_Get_Channel_Data((float) pid_chassis_spd[0].MaxOutput, 1 );
//	DataScope_Get_Channel_Data((float) pid_chassis_spd[1].MaxOutput, 2 );
//	DataScope_Get_Channel_Data((float) pid_chassis_spd[2].MaxOutput, 3 );
//	DataScope_Get_Channel_Data((float) pid_chassis_spd[3].MaxOutput, 4 );
//	DataScope_Get_Channel_Data((float) gimbal.pid.yaw_angle_ref, 2 );
//	DataScope_Get_Channel_Data((float) gimbal.current[1], 2 );
//	DataScope_Get_Channel_Data((float)powercontrol.chassis_power, 1 );
//	DataScope_Get_Channel_Data((float)powercontrol.power_buffer, 2 );
	
		CK.Send_Count = DataScope_Data_Generate(2);//串口需要发送的数据个数
		if(huart == &huart2)
		{
			for( CK.DataCnt = 0 ; CK.DataCnt < CK.Send_Count; CK.DataCnt++) 
			{
			while((USART2->SR&0X40)==0);  
			USART2->DR = CK.DataScope_OutPut_Buffer[CK.DataCnt]; 
			}
		}
		else if(huart == &huart3)
		{
			for( CK.DataCnt = 0 ; CK.DataCnt < CK.Send_Count; CK.DataCnt++) 
			{
			while((USART3->SR&0X40)==0);  
			USART3->DR = CK.DataScope_OutPut_Buffer[CK.DataCnt]; 
			}
		}
		else if(huart == &huart6)
		{
			for( CK.DataCnt = 0 ; CK.DataCnt < CK.Send_Count; CK.DataCnt++) 
			{
			while((USART6->SR&0X40)==0);  
			USART6->DR = CK.DataScope_OutPut_Buffer[CK.DataCnt]; 
			}
		}
		else if(huart == &huart5)
		{
			for( CK.DataCnt = 0 ; CK.DataCnt < CK.Send_Count; CK.DataCnt++) 
			{
			while((UART5->SR&0X40)==0);  
			UART5->DR = CK.DataScope_OutPut_Buffer[CK.DataCnt]; 
			}
		}
}

