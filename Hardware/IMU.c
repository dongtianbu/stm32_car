#include "stm32f10x.h"                  // Device header
#include "MPU6050.h"
#include "IMU.h"
#include <math.h>
/*
获取变量，对加速度三轴数据归一化，通过理论计算理论三轴加速度
得出误差，对误差积分

*/

#define Kp				20.001f
#define Ki				0.001f
#define cycle_T			0.005f // 200Hz
#define half_T			0.0025f

param_imu imu_data;
param_Angle imu_angle;
float q[4] = {1.0, 0.0, 0.0, 0.0};
float exInt = 12.0f, eyInt = 0.0f, ezInt = 0.0f; //初始误差设置

//快速开平方根
float fast_sqrt(float x)
{
	float halfx = 0.5f * x;
	float y = x;
	long i = *(long *) &y;
	i = 0x5f3759df - (i >> 1);
	y = *(float *) &i;
	y = y * (1.5f - (halfx * y * y));
	return y;
}

void IMU_GetValues(void)
{
	//获取数据并进行转化
	MPU6050_GetData(&AX, &AY, &AZ, &GX, &GY, &GZ);
	imu_data.AX = ((float)AX) / 2048; // 65536 / 32
	imu_data.AY = ((float)AY) / 2048;
	imu_data.AZ = ((float)AZ) / 2048;
	imu_data.GX = ((float)GX) * 0.001065264; // 4000 / 65536 * 3.14 / 180 弧度制
	imu_data.GY = ((float)GY) * 0.001065264;
	imu_data.GZ = ((float)GZ) * 0.001065264;
}

void ARHSUpdate(param_imu* imu_temp)
{
	float ax, ay, az;
	float gx, gy, gz;
	float vx, vy, vz;
	float ex, ey, ez;
	
	float q0 = q[0];
	float q1 = q[1];
	float q2 = q[2];
	float q3 = q[3];
	
	ax = imu_temp->AX; 
	ay = imu_temp->AY; 
	az = imu_temp->AZ; 
	gx = imu_temp->GX; 
	gy = imu_temp->GY; 
	gz = imu_temp->GZ; 
	//对实际加速度归一化处理
	float norm = fast_sqrt(ax * ax + ay * ay + az * az);
    ax /= norm;
    ay /= norm;
    az /= norm;
	//估算加速度
	vx = 2  * (q1 * q3 - q0 * q2);
	//vy = 2  * (q2 * q3 + q0 * q3);
	vy = 2 * (q0 * q1 + q2 * q3); // 修正项
	vz = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;
	//叉乘的误差向量
	ex = (ay * vz - az * vy);
	ey = (az * vx - ax * vz);
	ez = (ax * vy - ay * vx);
	//
	exInt += Ki * ex * cycle_T;
	eyInt += Ki * ey * cycle_T;
	ezInt += Ki * ez * cycle_T;
	
	gx += Kp * ex + exInt;
	gy += Kp * ey + eyInt;
	gz += Kp * ez + ezInt;
	//更新四元数
	q0 = q0 + (-q1 * gx - q2 * gy - q3 * gz) * half_T;
	q1 = q1 + ( q0 * gx + q2 * gz - q3 * gy) * half_T;
	q2 = q2 + ( q0 * gy - q1 * gz + q3 * gx) * half_T;
	q3 = q3 + ( q0 * gz + q1 * gy - q2 * gx) * half_T;
	
	norm = fast_sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q[0] = q0 / norm;
    q[1] = q1 / norm;
    q[2] = q2 / norm;
    q[3] = q3 / norm;
}

void IMU_GetEulerAngle(void)
{
	IMU_GetValues();
	ARHSUpdate(&imu_data);
	imu_angle.Pitch = asinf(-2.0 * q[1] * q[3] + 2.0 * q[0] * q[2]) * 57.2957;
	imu_angle.Roll = atan2f(2 * q[2] * q[3] + 2 * q[0] * q[1], 1 - 2 * q[1] * q[1] - q[2] * q[2]) * 57.2957;
	imu_angle.Yaw = imu_data.GZ * cycle_T * 57.2957 * 4;
}

