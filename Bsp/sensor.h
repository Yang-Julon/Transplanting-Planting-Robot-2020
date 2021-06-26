#ifndef __SENSOR_H_
#define __SENSOR_H_

#include "main.h"

typedef struct
{
	float Palstance;
	float Angle;
	
}gyro_t;

typedef struct
{
	uint8_t Dist;
	uint8_t Strength;
	
}laser_t;

typedef struct
{
	laser_t laser;
	float echo[4];
	gyro_t gyro;
	
}sensor_t;

extern sensor_t sensor;

#endif

