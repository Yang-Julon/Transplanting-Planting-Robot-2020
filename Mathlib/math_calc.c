#include "math_calc.h"

void abs_limit(float *a, float ABS_MAX,float offset)
{
    if(*a > ABS_MAX+offset)
        *a = ABS_MAX+offset;
    if(*a < -ABS_MAX+offset)
        *a = -ABS_MAX+offset;
}

float ecd_loop_calc(float New, float Last)
{
	float err = New - Last;
	
	if(err > 32768)
		return (65536 - err);
	
	else if(err < -32768)
		return (65536 + err);
	
	else
		return err;
}

float ecd_calc(float ecd)
{
	if(ecd > 32768)
		return (ecd - 65536);
	
	else
		return ecd;
}

/**
	*@func   float Circle_error(float set ,float get ,float circle_para)
	*@bref		环形数据计算偏差值
	*@param[in] set 设定值 get采样值 circle_para 一圈数值
	*@note	环形数据下，直接计算出PID中的偏差值
*/
float circle_error(float *set ,float *get ,float circle_para)
{
	float error;
	if(*set>=circle_para)
	{
		*set=*set-circle_para;
	}
	else if(*set<0)
	{
		*set=*set+circle_para;
	}
	if(*set > *get)
	{
		if(*set - *get> circle_para/2)
			error = *set - *get - circle_para;
		else
			error = *set - *get;
	}
	else if(*set < *get)
	{
		if(*set - *get<-1*circle_para/2)
			error = *set - *get +circle_para;
		else
			error = *set - *get;
	}
	else	error = 0;

	return error;
}

