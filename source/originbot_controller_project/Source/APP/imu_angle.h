#ifndef __IMU_ANGLE_H_
#define __IMU_ANGLE_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "stdint.h"
#include <math.h>
#include "bsp_icm42670P.h"



typedef struct{
	float q0;
	float q1;
	float q2;
	float q3;
}quaterInfo_t;

typedef struct{
	float roll;
	float pitch;
	float yaw;
}eulerianAngles_t;


void IMU_Update_Raw_Data(int16_t icm_accel[3], int16_t icm_gyro[3]);
void IMU_Update_Eulerian_Angles(void);
void IMU_Update_Gyro_Scale(uint16_t dps);
void IMU_Get_Eulerian_Angles(eulerianAngles_t* angles);

#ifdef __cplusplus
}
#endif

#endif

