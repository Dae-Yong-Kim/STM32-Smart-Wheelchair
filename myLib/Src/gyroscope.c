#include "main.h"
#include <stdio.h>
#include <math.h>

I2C_HandleTypeDef* hi2c = NULL;

#define Gyro_addr 0xD0

void i2c_Gyro_init(I2C_HandleTypeDef* p)
{
	hi2c = p;
}
uint8_t data;

void Gyro_ModuleSet()
{
	// PWR_MGMT : 0x6B
	data = 0x80;	// Reset
	HAL_I2C_Mem_Write(hi2c, Gyro_addr, 0x6B, 1, &data, 1, 1000);
	HAL_Delay(100);

	// PWR_MGMT : 0x6B
	data = 0x00;	// Operating
	HAL_I2C_Mem_Write(hi2c, Gyro_addr, 0x6B, 1, &data, 1, 1000);
	HAL_Delay(100);

	// Signal Path Reset : 0x68
	data = 0x07;	// All Reset
	HAL_I2C_Mem_Write(hi2c, Gyro_addr, 0x68, 1, &data, 1, 1000);
	HAL_Delay(100);

	// Sampling Rate set : 0x19
	data = 0x00;	// 1kHz Sampling Rate
	HAL_I2C_Mem_Write(hi2c, Gyro_addr, 0x19, 1, &data, 1, 1000);
	HAL_Delay(100);

	// Gyroscope set : 0x1B
	data = 0x00;	// -250 ~ +250 Degree/sec
	HAL_I2C_Mem_Write(hi2c, Gyro_addr, 0x1B, 1, &data, 1, 1000);
	HAL_Delay(100);

	// Accelerometer set : 0x1C
	data = 0x18;	// -16 ~ +16 g
	HAL_I2C_Mem_Write(hi2c, Gyro_addr, 0x1C, 1, &data, 1, 1000);
	HAL_Delay(100);

	// Accelerometer LPF set : 0x1D
	data = 0x00;	// 460 Hz LPF
	HAL_I2C_Mem_Write(hi2c, Gyro_addr, 0x1D, 1, &data, 1, 1000);
	HAL_Delay(100);
}

unsigned char gyro[6];
void ReadGyro()
{
	HAL_I2C_Mem_Read(hi2c, Gyro_addr, 0x43, 1, gyro, 6, 1000);

	short gyro_x = (gyro[0] << 8) + gyro[1];
	short gyro_y = (gyro[2] << 8) + gyro[3];
	short gyro_z = (gyro[4] << 8) + gyro[5];

	double gX = (double)gyro_x / 131.0;	// degree per second Value
	double gY = (double)gyro_y / 131.0;
	double gZ = (double)gyro_z / 131.0;

	printf("gyroX : %2.1f, gyroY : %2.1f, gyroZ : %2.1f\r\n", gX, gY, gZ);
}

unsigned char acc[6];
void ReadAcc()
{

	// GYRO Data Read : 0x3B  XH-XL-YH-YL-ZH-ZL
	HAL_I2C_Mem_Read(hi2c, Gyro_addr, 0x3B, 1, acc, 6, 1000);

	short acc_x = (acc[0] << 8) + acc[1];
	short acc_y = (acc[2] << 8) + acc[3];
	short acc_z = (acc[4] << 8) + acc[5];

	// -16 ~ 16
	double aX = (double)acc_x / 2048.0;	// g Value
	double aY = (double)acc_y / 2048.0;
	double aZ = (double)acc_z / 2048.0;

	printf("aX : %2.1f, aY : %2.1f, aZ : %2.1f\r\n", aX, aY, aZ);
}

double radTodeg = 180 / (3.141592);
double angleX = 0.0;  // Roll
double angleY = 0.0;  // Pitch
//double angleZ = 0.0;

void ReadAcc_Angle()
{

	// GYRO Data Read : 0x3B  XH-XL-YH-YL-ZH-ZL
	HAL_I2C_Mem_Read(hi2c, Gyro_addr, 0x3B, 1, acc, 6, 1000);

	short acc_x = (acc[0] << 8) + acc[1];
	short acc_y = (acc[2] << 8) + acc[3];
	short acc_z = (acc[4] << 8) + acc[5];

	// -16 ~ 16
	double aX = (double)acc_x / 2048.0;	// g Value
	double aY = (double)acc_y / 2048.0;
	double aZ = (double)acc_z / 2048.0;

	// Roll
	angleX = atan2(aY, sqrt(pow(aX, 2) + pow(aZ, 2))) * radTodeg;
	// Pitch
	angleY = atan2(-aX, sqrt(pow(aY, 2) + pow(aZ, 2))) * radTodeg;

	printf("Roll (X): %3.1f, Pitch (Y): %3.1f\r\n", angleX, angleY);
}

double gyroZbias = 0.0; // Gyro Noise
// Gyro Standard Noise Detect
void CalibrateGyro()
{
    int sum = 0;
    int samples = 10;

    for(int i = 0; i < samples; i++)
    {
        HAL_I2C_Mem_Read(&hi2c1, Gyro_addr, 0x47, 1, gyro, 2, 1000);
        short gZ = (gyro[0]<<8) + gyro[1];
        //gZ = gZ / 131.0;
        if((gZ > 0.05) || (gZ < -150))	sum += 0;
        else							sum += gZ;
        HAL_Delay(10);
    }
    double devide = (double)samples * 131;
    gyroZbias = (double)sum / devide;
    printf("gyroZbias : %7.3f\r\n", gyroZbias);
    HAL_Delay(10);
}

unsigned char gy_z[2];
double pre_gZ = 0, total_gZ = 0;
int before = 0;
double max_degree = 0;
void Read_Z_Angle()
{
	CalibrateGyro();

	HAL_I2C_Mem_Read(&hi2c1, Gyro_addr, 0x47, 1, gy_z, 2, 1000);

	short gyro_z = (gy_z[0]<<8) + gy_z[1];
	double gZ = (double)gyro_z / 131.0;


	gZ -= gyroZbias;

	total_gZ += (gZ + pre_gZ) * (HAL_GetTick() - before) / 2000;

	double gyroRate = gZ;
	double gyroAngle = total_gZ;

	printf("gyrorate: %7.3f, gyroAngle: %7.3f\r\n", gyroRate, gyroAngle);

	// filter
	if (fabs(gyroRate) < 0.05) {
		// Steady state
		total_gZ = pre_gZ * 0.999 + gyroAngle * 0.001;
	} else {
		// Move State
		total_gZ = gyroAngle * 0.999 + pre_gZ * 0.001;
	}

	printf("gZ: %7.3f | total z degree: %7.3f\r\n", gZ, total_gZ);

	pre_gZ = gZ;
	before = HAL_GetTick();

	if(fabs(max_degree) < fabs(total_gZ))	max_degree = total_gZ;
	printf("max_degree : %7.3f\r\n", max_degree);
}
void Max_de(double* max)
{
	max = &max_degree;
}
