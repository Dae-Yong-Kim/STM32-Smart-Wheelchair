#include "main.h"
#include <stdio.h>
#include <math.h>

I2C_HandleTypeDef* hi2c = NULL;
void i2c_Gyro_init(I2C_HandleTypeDef* p)
{
	hi2c = p;
}
uint8_t data;
#define Gyro_addr 0xD0
void Gyro_ModuleSet()
{
	// PWR_MGMT : 0x6B
	data = 0x80;	// Reset
	HAL_I2C_Mem_Write(hi2c, Gyro_addr, 0x6B, 1, &data, 1, 1000);
	osDelay(100);

	// PWR_MGMT : 0x6B
	data = 0x00;	// Operating
	HAL_I2C_Mem_Write(hi2c, Gyro_addr, 0x6B, 1, &data, 1, 1000);
	osDelay(100);

	// Signal Path Reset : 0x68
	data = 0x07;	// All Reset
	HAL_I2C_Mem_Write(hi2c, Gyro_addr, 0x68, 1, &data, 1, 1000);
	osDelay(100);

	// Sampling Rate set : 0x19
	data = 0x00;	// 1kHz Sampling Rate
	HAL_I2C_Mem_Write(hi2c, Gyro_addr, 0x19, 1, &data, 1, 1000);
	osDelay(100);

	// Gyroscope set : 0x1B
	data = 0x00;	// -250 ~ +250 Degree/sec
	HAL_I2C_Mem_Write(hi2c, Gyro_addr, 0x1B, 1, &data, 1, 1000);
	osDelay(100);
}
unsigned char gy_z[2];
double pre_gZ = 0, total_gZ = 0;
int before = 0;
double max_degree = 0;
void Read_Z_Angle(double* max)
{

	HAL_I2C_Mem_Read(hi2c, Gyro_addr, 0x47, 1, gy_z, 2, 1000);

	short gyro_z = (gy_z[0]<<8) + gy_z[1];
	double gZ = (double)gyro_z / 131.0;

	if((gZ > 0.3) && (gZ < 0.7))
	{
		gZ = 0;
		total_gZ = 0;
		before = HAL_GetTick();

		max_degree = 0;
		*max = max_degree;
	}
	else
	{
		if(!before)
		{
			before = HAL_GetTick();
		}
		else
		{
			total_gZ += (gZ + pre_gZ) * (HAL_GetTick() - before) / 2000;

			double gyroRate = gZ;
			double gyroAngle = total_gZ;

			// filter
			if (fabs(gyroRate) < 0.05) {
				// Steady state
				total_gZ = pre_gZ * 0.999 + gyroAngle * 0.001;
			} else {
				// Move State
				total_gZ = gyroAngle * 0.999 + pre_gZ * 0.001;
			}

			pre_gZ = gZ;
			before = HAL_GetTick();
		}

		if(fabs(max_degree) < fabs(total_gZ))	max_degree = total_gZ;

		*max = max_degree;
	}
}
