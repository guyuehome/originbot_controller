/***********************************************************************
Copyright (c) 2022, www.guyuehome.com

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

? ? http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
***********************************************************************/

#include "imu_angle.h"

// 10ms计算一次
#define delta_T              0.01f  
#define new_weight           0.35f
#define old_weight           0.65f

#define M_PI (3.1415926f)

float I_ex, I_ey, I_ez;              // 误差积分
quaterInfo_t Q_info = {1, 0, 0, 0};  // 全局四元数
eulerianAngles_t eulerAngle;         //欧拉角


float param_Kp = 50.0;   // 加速度计的收敛速率比例增益50 
float param_Ki = 0.80;   // 陀螺仪收敛速率的积分增益 0.2
float gyro_scale = 16.4; // 131  65.5  32.8  16.4
float values[10];


static float invSqrt(float x) 
{
	float halfx = 0.5f * x;
	float y = x;
	long i = *(long*)&y;
	i = 0x5f3759df - (i>>1);
	y = *(float*)&i;
	y = y * (1.5f - (halfx * y * y));
	return y;
}

// 设置IMU gyro的dps缩放系数。dps=[250,500,1000,2000]
void IMU_Update_Gyro_Scale(uint16_t dps)
{
    switch (dps)
    {
    case 250:
        gyro_scale = 131;
        break;
    case 500:
        gyro_scale = 65.5;
        break;
    case 1000:
        gyro_scale = 32.8;
        break;
    case 2000:
        gyro_scale = 16.4;
        break;
    default:
        break;
    }
}


/*
	更新IMU原始数据
	对accel一阶低通滤波，对gyro转成弧度每秒
*/
void IMU_Update_Raw_Data(int16_t icm_accel[3], int16_t icm_gyro[3])
{
    static double lastaccel[3]= {0,0,0};
    int i;
    values[0] = ((float)icm_accel[0]) * new_weight + lastaccel[0] * old_weight;
    values[1] = ((float)icm_accel[1]) * new_weight + lastaccel[1] * old_weight;
    values[2] = ((float)icm_accel[2]) * new_weight + lastaccel[2] * old_weight;
    for(i=0; i<3; i++)
    {
        lastaccel[i] = values[i];
    }
 
    values[3] = ((float)icm_gyro[0]) * M_PI / 180.0 / gyro_scale;
    values[4] = ((float)icm_gyro[1]) * M_PI / 180.0 / gyro_scale;
    values[5] = ((float)icm_gyro[2]) * M_PI / 180.0 / gyro_scale;
}


/**
  * brief IMU_AHRS_Calculating  姿态解算融合，是Crazepony和核心算法
  * 使用的是互补滤波算法，没有使用Kalman滤波算法
  * param float gx, float gy, float gz, float ax, float ay, float az
  *
  * return None
  */
static void IMU_AHRS_Calculating(float gx, float gy, float gz, float ax, float ay, float az)
{
	float halfT = 0.5 * delta_T;
	float vx, vy, vz;    //当前的机体坐标系上的重力单位向量
	float ex, ey, ez;    //四元数计算值与加速度计测量值的误差
	float q0 = Q_info.q0;
	float q1 = Q_info.q1;
	float q2 = Q_info.q2;
	float q3 = Q_info.q3;
	float q0q0 = q0 * q0;
	float q0q1 = q0 * q1;
	float q0q2 = q0 * q2;
	// float q0q3 = q0 * q3;
	float q1q1 = q1 * q1;
	// float q1q2 = q1 * q2;
	float q1q3 = q1 * q3;
	float q2q2 = q2 * q2;
	float q2q3 = q2 * q3;
	float q3q3 = q3 * q3;
	float delta_2 = 0;
	
	//对加速度数据进行归一化 得到单位加速度
	float norm = invSqrt(ax*ax + ay*ay + az*az);       
	ax = ax * norm;
	ay = ay * norm;
	az = az * norm;
	vx = 2*(q1q3 - q0q2);
	vy = 2*(q0q1 + q2q3);
	vz = q0q0 - q1q1 - q2q2 + q3q3;

	ex = ay * vz - az * vy;
	ey = az * vx - ax * vz;
	ez = ax * vy - ay * vx;
	
	//用叉乘误差来做PI修正陀螺零偏，
	//通过调节 param_Kp，param_Ki 两个参数，
	//可以控制加速度计修正陀螺仪积分姿态的速度。
	I_ex += delta_T * ex;   // integral error scaled by Ki
	I_ey += delta_T * ey;
	I_ez += delta_T * ez;

	gx = gx+ param_Kp*ex + param_Ki*I_ex;
	gy = gy+ param_Kp*ey + param_Ki*I_ey;
	gz = gz+ param_Kp*ez + param_Ki*I_ez;
	
	
	//四元数微分方程，其中halfT为测量周期的1/2，gx gy gz为陀螺仪角速度。
    // 一阶龙格库塔法, 求解四元数微分方程
//	 q0 = q0 + (-q1*gx - q2*gy - q3*gz)*halfT;
//	 q1 = q1 + ( q0*gx + q2*gz - q3*gy)*halfT;
//	 q2 = q2 + ( q0*gy - q1*gz + q3*gx)*halfT;
//	 q3 = q3 + ( q0*gz + q1*gy - q2*gx)*halfT;

	// 二阶毕卡法, 求解四元数微分方程, 四元数更新算法
	delta_2=(2*halfT*gx)*(2*halfT*gx)+(2*halfT*gy)*(2*halfT*gy)+(2*halfT*gz)*(2*halfT*gz);
	q0 = (1-delta_2/8)*q0 + (-q1*gx - q2*gy - q3*gz)*halfT;
	q1 = (1-delta_2/8)*q1 + (q0*gx + q2*gz - q3*gy)*halfT;
	q2 = (1-delta_2/8)*q2 + (q0*gy - q1*gz + q3*gx)*halfT;
	q3 = (1-delta_2/8)*q3 + (q0*gz + q1*gy - q2*gx)*halfT;
	
    // normalise quaternion
	norm = invSqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
	Q_info.q0 = q0 * norm;
	Q_info.q1 = q1 * norm;
	Q_info.q2 = q2 * norm;
	Q_info.q3 = q3 * norm;
}



/*把四元数转换成欧拉角*/
void IMU_Update_Eulerian_Angles(void)
{
	IMU_AHRS_Calculating(values[3], values[4], values[5], values[0], values[1], values[2]);
    float q0 = Q_info.q0;
    float q1 = Q_info.q1;
    float q2 = Q_info.q2;
    float q3 = Q_info.q3;
    eulerAngle.pitch = asin(-2*q1*q3 + 2*q0*q2) * 180/M_PI; // pitch
    eulerAngle.roll = atan2(2*q2*q3 + 2*q0*q1, -2*q1*q1 - 2*q2*q2 + 1) * 180/M_PI; // roll
    eulerAngle.yaw = atan2(2*q1*q2 + 2*q0*q3, -2*q2*q2 - 2*q3*q3 + 1) * 180/M_PI; // yaw

 
    /* 可以不用作姿态限度的限制 */
    // if(eulerAngle.roll>90 || eulerAngle.roll<-90)
    // {
    //     if(eulerAngle.pitch > 0)
    //     {
    //         eulerAngle.pitch = 180-eulerAngle.pitch;
    //     }
    //     if(eulerAngle.pitch < 0)
    //     {
    //         eulerAngle.pitch = -(180+eulerAngle.pitch);
    //     }
    // }
    // if(eulerAngle.yaw > 180)
    // {
    //     eulerAngle.yaw -=360;
    // }
    // else if(eulerAngle.yaw <-180)
    // {
    //     eulerAngle.yaw +=360;
    // } 
}


void IMU_Get_Eulerian_Angles(eulerianAngles_t* angles)
{
    angles->pitch = eulerAngle.pitch;
    angles->roll = eulerAngle.roll;
    angles->yaw = eulerAngle.yaw;
}
