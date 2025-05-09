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
#include "app_motion_control.h"
#include "Main.h"
#include "UART1.h"

#include <math.h>
#include <stdio.h> 

// 小车刹车超时时间，单位为10ms
#define MAX_STOP_COUNT       3

// 左右轮电机PWM变量
int motorLeft = 0;
int motorRight = 0;

// 左右轮速度
int leftSpeedNow = 0;
int rightSpeedNow = 0;

// 左右轮速度设置
int leftSpeedSet = 0;
int rightSpeedSet = 0;

// 编码器10ms前后数据
int leftWheelEncoderNow = 0;
int rightWheelEncoderNow = 0;
int leftWheelEncoderLast = 0;
int rightWheelEncoderLast = 0;


// 累计编码器计数和准备上报速度
int left_encoder_cnt = 0;
int right_encoder_cnt = 0;
double left_speed_mm_s = 0;
double right_speed_mm_s = 0;
// 上报速度数据时记录的编码器进入10ms次数
int record_time=0;


char s_pwm_1[8];
void pid_debug_uartsend(int data_1,int data_2,int data_3) // DEBUG pid调参，通过串口直接发送pid中的设定值、反馈和输出，使用过空格分隔。使用此方法可以注释掉main中的所有发送部分方便上位机分析。
{
  sprintf(s_pwm_1, "%8d", data_1);
  for (int i = 0; i < 8; i++)
  {
    UART1_Put_Char(s_pwm_1[i]);
  }
  // UART1_Put_Char(0x40);
  UART1_Put_Char(0x20);

  sprintf(s_pwm_1, "%8d", data_2);
  for (int i = 0; i < 8; i++)
  {
    UART1_Put_Char(s_pwm_1[i]);
  }

  UART1_Put_Char(0x20);

  sprintf(s_pwm_1, "%8d", data_3);
  for (int i = 0; i < 8; i++)
  {
    UART1_Put_Char(s_pwm_1[i]);
  }

  UART1_Put_Char(0x0d);
  UART1_Put_Char(0x0a);
}

u32 g_start_count = 0;//记录开机的时间 
extern int8_t imu_initflag;
extern int8_t g_imu_dealflag;
// TIM1每10ms产生一次中断
void TIM1_UP_IRQHandler(void)
{
  // 检查是否发生中断事件
  if (TIM_GetITStatus(TIM1, TIM_IT_Update) != RESET) {
		
		g_start_count++;//10ms加一次
		//IMU相关
		if(imu_initflag == 0)
		{
			g_imu_dealflag = 1;
//			get_icm_attitude();
		}
		
    if (Timer_Get_Count(COUNT_BEAT_ID))
      Timer_Count_Auto_Reduce(COUNT_BEAT_ID);

    #if ENABLE_MOTION_CONTROL
    Motion_Control_10ms();
    #endif

    
    // 清除中断标志位
    TIM_ClearITPendingBit(TIM1, TIM_IT_Update);  
  }
}

// 实时计算小车运行速度
void Motion_Control_10ms(void)
{
  // 获取左右轮当前实际速度(mm/s)
  Get_Motor_Speed(&leftSpeedNow, &rightSpeedNow);

  if (leftSpeedSet || rightSpeedSet) 
  {
    // 目标速度
    pid_Task_Left.speedSet = leftSpeedSet;
    pid_Task_Right.speedSet = rightSpeedSet;
    // 实际速度
    pid_Task_Left.speedNow = leftSpeedNow;
    pid_Task_Right.speedNow = rightSpeedNow;

    // 执行PID控制
    Pid_Ctrl(&motorLeft, &motorRight, g_attitude.yaw);
    
    // 设置PWM
    Motion_Set_PWM(motorLeft, motorRight);
		
  } 
  else 
  {
    motorLeft = 0;
    motorRight = 0;
    Motion_Set_PWM(motorLeft, motorRight);
    reset_Uk(&pid_Task_Left);
    reset_Uk(&pid_Task_Right);
  }

}


void Motion_Set_PWM(int motor_Left, int motor_Right)
{
  Motor_Set_Pwm(MOTOR_ID_1, motor_Left);
  Motor_Set_Pwm(MOTOR_ID_2, motor_Right);
}



void Get_Motor_Speed(int *leftSpeed, int *rightSpeed)
{
  Encoder_Update_Count(ENCODER_ID_A);
  leftWheelEncoderNow = Encoder_Get_Count_Now(ENCODER_ID_A);
  Encoder_Update_Count(ENCODER_ID_B);
  rightWheelEncoderNow = Encoder_Get_Count_Now(ENCODER_ID_B);
  
 
  *leftSpeed = (leftWheelEncoderNow - leftWheelEncoderLast) * ENCODER_CNT_10MS_2_SPD_MM_S;
  *rightSpeed =(rightWheelEncoderNow - rightWheelEncoderLast)* ENCODER_CNT_10MS_2_SPD_MM_S;
  left_encoder_cnt += leftWheelEncoderNow - leftWheelEncoderLast;
  right_encoder_cnt += rightWheelEncoderNow - rightWheelEncoderLast;
  record_time++;
  // 记录上一周期的编码器数据
  leftWheelEncoderLast = leftWheelEncoderNow;
  rightWheelEncoderLast = rightWheelEncoderNow;
}




// 上报电机速度
void Motion_Send_Data(void)
{
  //计算本次上报时应当上报的速度
  left_speed_mm_s = left_encoder_cnt * ENCODER_CNT_10MS_2_SPD_MM_S / record_time;
  right_speed_mm_s = right_encoder_cnt * ENCODER_CNT_10MS_2_SPD_MM_S / record_time;
  record_time = 0;
  left_encoder_cnt = 0;
  right_encoder_cnt = 0;

  #define MotionLEN        7
  uint8_t data_buffer[MotionLEN] = {0};
  uint8_t i, checknum = 0;
  
  if (left_speed_mm_s < 0) {
    data_buffer[0] = 0x00;
    uint16_t spd = (uint16_t)fabs(left_speed_mm_s);
    data_buffer[1] = spd&0xFF;
    data_buffer[2] = (spd>>8)&0xFF;
  } else {
    data_buffer[0] = 0xFF;
    uint16_t spd = (uint16_t)left_speed_mm_s;
    data_buffer[1] = spd&0xFF;
    data_buffer[2] = (spd>>8)&0xFF;
  }

  if (right_speed_mm_s < 0) {
    data_buffer[3] = 0x00;
    uint16_t spd = (uint16_t)fabs(right_speed_mm_s);
    data_buffer[4] = spd&0xFF;
    data_buffer[5] = (spd>>8)&0xFF;
  } else {
    data_buffer[3] = 0xFF;
    uint16_t spd = (uint16_t)right_speed_mm_s;
    data_buffer[4] = spd&0xFF;
    data_buffer[5] = (spd>>8)&0xFF;
  }

  // 校验位的计算使用数据位各个数据相加 & 0xFF
  for (i = 0; i < MotionLEN - 1; i++)
    checknum += data_buffer[i];

  data_buffer[MotionLEN - 1] = checknum & 0xFF;
  UART1_Put_Char(0x55); // 帧头
  UART1_Put_Char(0x02); // 标识位
  UART1_Put_Char(0x06); // 数据位长度(字节数)
  
  UART1_Put_Char(data_buffer[0]);
  UART1_Put_Char(data_buffer[1]);
  UART1_Put_Char(data_buffer[2]);
  UART1_Put_Char(data_buffer[3]);
  UART1_Put_Char(data_buffer[4]);
  UART1_Put_Char(data_buffer[5]);
  UART1_Put_Char(data_buffer[6]);
  
  UART1_Put_Char(0xBB); // 帧尾
}



// 设置目标速度 此处set是pwm值
void Motion_Test_SpeedSet(uint8_t index_l, int16_t left , 
                          uint8_t index_r, int16_t right)
{
  // l_speed_diff=left-right;
  if (left > SPD_MM_S_MAX) 
    left = SPD_MM_S_MAX;
  if (right > SPD_MM_S_MAX) 
    right = SPD_MM_S_MAX;

  if (index_l == 0)
    leftSpeedSet = -left;
  else
    leftSpeedSet = left;
    
  if (index_r == 0)
    rightSpeedSet = -right;
  else
    rightSpeedSet = right;
}
