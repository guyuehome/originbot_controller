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

#include "app_imu_42670P.h"
#include "imu_angle.h"

icm_data_t g_icm42670;
icm_data_t g_icm42670_old;
int16_t gyrosub[3] = {0};

inv_imu_sensor_event_t imu_event;
eulerianAngles_t myangles = {0};

int16_t accelmy[3] = {0}; //x,y,z
int16_t gyromy[3] = {0}; //x,y,z
int16_t Deviation_gyro[3] = {0};    //陀螺仪静差 和原始数据
int16_t Deviation_acc[3] = {0};

int16_t icm_check_time = 0;

u8 imu_frist = 0; //记录第一次赋值


// 在此时间前，陀螺仪数据用来检测。单位为：10毫秒。默认值：500
#define ICM_SKIP            20 //20*10

// 在上面的基础上，前几个数据不稳定直接抛弃掉。单位为：10毫秒。默认值：20
#define ICM_ABANDON         2  //2*10

int16_t imu_abs(int16_t x)
{
	if(x<0)
		return -x;
	
	return x;

}

void get_raw_data(void)
{
		u8 int_status;
		int_status = IIC_ReadReg(0x39);
//		printf("%d\r\n",int_status);
	if(int_status == 1) //数据准备好读出来
	{
		get_raw_accel();
		get_raw_gyro();
	}
}

void get_raw_accel(void)
{
	int8_t data_L,data_H;
	
	data_H = IIC_ReadReg(0x0b);
	data_L = IIC_ReadReg(0x0c);
	accelmy [0] =  data_H<<8 | data_L;
	
	
	data_H = IIC_ReadReg(0x0d);
	data_L = IIC_ReadReg(0x0e);
	accelmy [1] =  data_H<<8 | data_L;
	
	data_H = IIC_ReadReg(0x0f);
	data_L = IIC_ReadReg(0x10);
	accelmy [2] =  data_H<<8 | data_L;

}



void get_raw_gyro(void)
{
	int8_t data_L,data_H;
	
	data_H = IIC_ReadReg(0x11);
	data_L = IIC_ReadReg(0x12);
	gyromy [0] =  data_H<<8 | data_L;
	
	data_H = IIC_ReadReg(0x13);
	data_L = IIC_ReadReg(0x14);
	gyromy [1] =  data_H<<8 | data_L;
	
	data_H = IIC_ReadReg(0x15);
	data_L = IIC_ReadReg(0x16);
	gyromy [2] =  data_H<<8 | data_L;

}



void get_data_imu(void)
{

	getDataFromRegisters(&imu_event);

	memcpy(accelmy,imu_event.accel,sizeof(accelmy));
	memcpy(gyromy,imu_event.gyro,sizeof(gyromy));
}



void deal_data_imu(int16_t icm_accel[3], int16_t icm_gyro[3])
{
	static u8 printf_flag = 0;
	printf_flag++;
	
	IMU_Update_Raw_Data(icm_accel,icm_gyro);//更新原始数据
	IMU_Update_Eulerian_Angles();//四元数计算
	IMU_Get_Eulerian_Angles(&myangles);//更新融合角度
	
	
}


// 获取陀螺仪姿态角
void get_icm_attitude(void)
{

    if (icm_check_time < ICM_SKIP)
    {
        icm_check_time++;
				get_data_imu();
			
        if (icm_check_time <= ICM_ABANDON) return;
			
        Deviation_gyro[0] += gyromy[0];
        Deviation_gyro[1] += gyromy[1];
        Deviation_gyro[2] += gyromy[2];
			


        if (icm_check_time >= ICM_SKIP)
        {
				   // 求平均值
            Deviation_gyro[0] = Deviation_gyro[0] / (ICM_SKIP - ICM_ABANDON);
            Deviation_gyro[1] = Deviation_gyro[1] / (ICM_SKIP - ICM_ABANDON);
            Deviation_gyro[2] = Deviation_gyro[2] / (ICM_SKIP - ICM_ABANDON);

				}
    }
    else
    {
        get_data_imu();

        g_icm42670.acc[0] = accelmy[0] ;
        g_icm42670.acc[1] = accelmy[1] ;
        g_icm42670.acc[2] = accelmy[2];

        g_icm42670.gyro[0] = gyromy[0] - Deviation_gyro[0];
        g_icm42670.gyro[1] = gyromy[1] - Deviation_gyro[1];
        g_icm42670.gyro[2] = gyromy[2] - Deviation_gyro[2];
			
			if(imu_frist == 0)
			{
				imu_frist = 1;
				g_icm42670_old.gyro[0] = g_icm42670.gyro[0];
				g_icm42670_old.gyro[1] = g_icm42670.gyro[1];
				g_icm42670_old.gyro[2] = g_icm42670.gyro[2];
			}
			
			gyrosub[0] = imu_abs(g_icm42670.gyro[0] - g_icm42670_old.gyro[0]);
			gyrosub[1] = imu_abs(g_icm42670.gyro[1] - g_icm42670_old.gyro[1]);
			gyrosub[2] = imu_abs(g_icm42670.gyro[2] - g_icm42670_old.gyro[2]);
			
			//陀螺仪数据yaw角漂移过滤
			if((gyrosub[0]>16) || (gyrosub[1])>16 || (gyrosub[2])>16) 
			{
				deal_data_imu(g_icm42670.acc,g_icm42670.gyro);
			}
				
		
    }
}

void Acc_Send_Data_ICM42670P(void)
{
	#define AccLEN        7
	uint8_t data_buffer[AccLEN] = {0};
	uint8_t i, checknum = 0;
	
	//低字节在前
	data_buffer[0] = accelmy[0]&0xFF;
	data_buffer[1] = (accelmy[0]>>8)&0xFF;
	
	data_buffer[2] = accelmy[1]&0xFF;
	data_buffer[3] = (accelmy[1]>>8)&0xFF;
	data_buffer[4] = accelmy[2]&0xFF;
	data_buffer[5] = (accelmy[2]>>8)&0xFF;

	for (i = 0; i < AccLEN-1; i++)
	{
		checknum += data_buffer[i];
	}
	data_buffer[AccLEN-1] = checknum & 0xFF;
	UART1_Put_Char(0x55);
	UART1_Put_Char(0x03);
	UART1_Put_Char(0x06);
	
	UART1_Put_Char(data_buffer[0]);
	UART1_Put_Char(data_buffer[1]);
	UART1_Put_Char(data_buffer[2]);
	UART1_Put_Char(data_buffer[3]);
	UART1_Put_Char(data_buffer[4]);
	UART1_Put_Char(data_buffer[5]);
	UART1_Put_Char(data_buffer[6]);
	
	UART1_Put_Char(0xBB);
}

void Gyro_Send_Data_ICM42670P(void)
{
	#define GyroLEN        7
	uint8_t data_buffer[GyroLEN] = {0};
	uint8_t i, checknum = 0;
	
	//低字节在前
	data_buffer[0] = gyromy[0]&0xFF;
	data_buffer[1] = (gyromy[0]>>8)&0xFF;
	data_buffer[2] = gyromy[1]&0xFF;
	data_buffer[3] = (gyromy[1]>>8)&0xFF;
	data_buffer[4] = gyromy[2]&0xFF;
	data_buffer[5] = (gyromy[2]>>8)&0xFF;

	for (i = 0; i < GyroLEN-1; i++)
	{
		checknum += data_buffer[i];
	}
	data_buffer[GyroLEN-1] = checknum & 0xFF;
	UART1_Put_Char(0x55);
	UART1_Put_Char(0x04);
	UART1_Put_Char(0x06);
	
	UART1_Put_Char(data_buffer[0]);
	UART1_Put_Char(data_buffer[1]);
	UART1_Put_Char(data_buffer[2]);
	UART1_Put_Char(data_buffer[3]);
	UART1_Put_Char(data_buffer[4]);
	UART1_Put_Char(data_buffer[5]);
	UART1_Put_Char(data_buffer[6]);
	
	UART1_Put_Char(0xBB);
}
void Angle_Send_Data_ICM42670P(void)
{
	#define AngleLEN        7
	uint8_t data_buffer[AngleLEN] = {0};
	uint8_t i, checknum = 0;
	int16_t rolltemp,pitchtemp,yawtemp; //临时变量
	
//	printf("YAW:%f\t,ROLL:%f\t,PITCH:%f\r\n",myangles.yaw,myangles.roll,myangles.pitch);
	rolltemp = myangles.roll *100;
	pitchtemp = myangles.pitch *100;
	yawtemp	= myangles.yaw *100;
	
	//低字节在前
	data_buffer[0] = rolltemp&0xFF;
	data_buffer[1] = (rolltemp>>8)&0xFF;
	data_buffer[2] = pitchtemp&0xFF;
	data_buffer[3] = (pitchtemp>>8)&0xFF;
	data_buffer[4] = yawtemp&0xFF;
	data_buffer[5] = (yawtemp>>8)&0xFF;

	for (i = 0; i < AngleLEN-1; i++)
	{
		checknum += data_buffer[i];
	}
	data_buffer[AngleLEN-1] = checknum & 0xFF;
	UART1_Put_Char(0x55);
	UART1_Put_Char(0x05);
	UART1_Put_Char(0x06);
	
	UART1_Put_Char(data_buffer[0]);
	UART1_Put_Char(data_buffer[1]);
	UART1_Put_Char(data_buffer[2]);
	UART1_Put_Char(data_buffer[3]);
	UART1_Put_Char(data_buffer[4]);
	UART1_Put_Char(data_buffer[5]);
	UART1_Put_Char(data_buffer[6]);
	
	UART1_Put_Char(0xBB);
}



