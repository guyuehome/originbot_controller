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
#include <string.h>
#include <stdio.h>
#include "Main.h"
#include "UART1.h"
#include "UART3.h"
#include "delay.h"
#include "JY901.h"
#include "DIO.h"
#include "adc.h"
#include "bsp_icm42670P.h"
#include "app_imu_42670P.h"

int8_t imu_initflag = 1; //0:初始化成功
int8_t g_imu_dealflag = 0;

int main(void)
{
	int Call_10ms = 1;
  // 设置时钟频率
  SysTick_init(72, 10);

	// 初始化串口1，用于和主机通信
  UART1_Init(115200);

  // 注意delay_ms(ms)中，ms必须<=1864
  Delay_Ms(1000); Delay_Ms(1000);
  
  // 初始化ADC，用于读取电池电压
  Adc_Init();
  // 初始化GPIO，用于控制LED和蜂鸣器等
  GPIO_Config();
  // 初始化电机控制接口(PWM)
  MOTOR_GPIO_Init();
  // 设置不分频，PWM频率 72000000/3600=20khz
  Motor_PWM_Init(MOTOR_MAX_PULSE, MOTOR_FREQ_DIVIDE);
  // 初始化电机编码器接口
  Encoder_Init();
  // 初始化心跳包定时器
  TIM1_Init();
  // 初始化PID控制
  PID_Init();
  
	
	#if ENABLE_ICM42670P
	imu_initflag = icm42670_init();
	#else
  // 初始化串口3，用于读取陀螺仪数据
  UART3_Init(9600);

  // 等待陀螺仪初始化完成
	jy901_init();
	#endif
	
	Delay_Ms(1000);
  
  // printf("\n\nFirmware Version: V%d.%d\n", VERSION_MAJOR, VERSION_MINOR);h
  
  while (1) {
		
		if(g_imu_dealflag ==1) //imu数据处理
			{
				g_imu_dealflag = 0;
				get_icm_attitude();
			}
			
    // 接收、解析并响应主机发送的串口指令
    if (Is_Recv_New_Cmd()) {
      Parse_Cmd_Data(Get_RxBuffer(), Get_CMD_Length());
      Clear_CMD_Flag();
    }
    
    // 周期性上报数据给主机
    if (Timer_Get_Count(COUNT_BEAT_ID) == 0) {
      // 检测电池电压是否过低，电压过低，蜂鸣器报警
      if (!Bat_Update_Power_State())
        BUZZER_ON();  

      // 上报周期：40毫秒
      if ((Call_10ms % 4 + 1) == 1) {
        Motion_Send_Data(); // 电机速度
				
				#if ENABLE_ICM42670P 
						Acc_Send_Data_ICM42670P();// 陀螺仪 加速度
						Gyro_Send_Data_ICM42670P();// 陀螺仪 角速度
						Angle_Send_Data_ICM42670P();// 陀螺仪 角度
					#else
						Acc_Send_Data();    // 陀螺仪 加速度
						Gyro_Send_Data();   // 陀螺仪 角速度
						Angle_Send_Data();  // 陀螺仪 角度
					#endif
        
        Sensor_Send_Data(); // 电池电量
      }
      
      Call_10ms++;
      if (Call_10ms >= 8)
        Call_10ms = 0;

      Timer_Set_Count(COUNT_BEAT_ID, 1);
    }
  }
}
