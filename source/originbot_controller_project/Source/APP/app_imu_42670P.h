#ifndef _APP_IMU_42670P_H_
#define _APP_IMU_42670P_H_

#include "math.h"
#include "bsp_icm42670P.h"

typedef struct _icm_data_t
{
    int16_t acc[3];
    int16_t gyro[3];


    int16_t Offset[6];
} icm_data_t;

void get_data_imu(void);

void deal_data_imu(int16_t icm_accel[3], int16_t icm_gyro[3]);

void get_raw_accel(void);
void get_raw_gyro(void);

void Acc_Send_Data_ICM42670P(void);
void Gyro_Send_Data_ICM42670P(void);
void Angle_Send_Data_ICM42670P(void);

void get_icm_attitude(void);
#endif



