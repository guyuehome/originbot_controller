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

// С��ɲ����ʱʱ�䣬��λΪ10ms
#define MAX_STOP_COUNT       3

// �����ֵ��PWM����
int motorLeft = 0;
int motorRight = 0;

// �������ٶ�
int leftSpeedNow = 0;
int rightSpeedNow = 0;

// �������ٶ�����
int leftSpeedSet = 0;
int rightSpeedSet = 0;

// ������10msǰ������
int leftWheelEncoderNow = 0;
int rightWheelEncoderNow = 0;
int leftWheelEncoderLast = 0;
int rightWheelEncoderLast = 0;


// �ۼƱ�����������׼���ϱ��ٶ�
int left_encoder_cnt = 0;
int right_encoder_cnt = 0;
double left_speed_mm_s = 0;
double right_speed_mm_s = 0;
// �ϱ��ٶ�����ʱ��¼�ı���������10ms����
int record_time=0;


char s_pwm_1[8];
void pid_debug_uartsend(int data_1,int data_2,int data_3) // DEBUG pid���Σ�ͨ������ֱ�ӷ���pid�е��趨ֵ�������������ʹ�ù��ո�ָ���ʹ�ô˷�������ע�͵�main�е����з��Ͳ��ַ�����λ��������
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

u32 g_start_count = 0;//��¼������ʱ�� 
extern int8_t imu_initflag;
extern int8_t g_imu_dealflag;
// TIM1ÿ10ms����һ���ж�
void TIM1_UP_IRQHandler(void)
{
  // ����Ƿ����ж��¼�
  if (TIM_GetITStatus(TIM1, TIM_IT_Update) != RESET) {
		
		g_start_count++;//10ms��һ��
		//IMU���
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

    
    // ����жϱ�־λ
    TIM_ClearITPendingBit(TIM1, TIM_IT_Update);  
  }
}

// ʵʱ����С�������ٶ�
void Motion_Control_10ms(void)
{
  // ��ȡ�����ֵ�ǰʵ���ٶ�(mm/s)
  Get_Motor_Speed(&leftSpeedNow, &rightSpeedNow);

  if (leftSpeedSet || rightSpeedSet) 
  {
    // Ŀ���ٶ�
    pid_Task_Left.speedSet = leftSpeedSet;
    pid_Task_Right.speedSet = rightSpeedSet;
    // ʵ���ٶ�
    pid_Task_Left.speedNow = leftSpeedNow;
    pid_Task_Right.speedNow = rightSpeedNow;

    // ִ��PID����
    Pid_Ctrl(&motorLeft, &motorRight, g_attitude.yaw);
    
    // ����PWM
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
  // ��¼��һ���ڵı���������
  leftWheelEncoderLast = leftWheelEncoderNow;
  rightWheelEncoderLast = rightWheelEncoderNow;
}




// �ϱ�����ٶ�
void Motion_Send_Data(void)
{
  //���㱾���ϱ�ʱӦ���ϱ����ٶ�
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

  // У��λ�ļ���ʹ������λ����������� & 0xFF
  for (i = 0; i < MotionLEN - 1; i++)
    checknum += data_buffer[i];

  data_buffer[MotionLEN - 1] = checknum & 0xFF;
  UART1_Put_Char(0x55); // ֡ͷ
  UART1_Put_Char(0x02); // ��ʶλ
  UART1_Put_Char(0x06); // ����λ����(�ֽ���)
  
  UART1_Put_Char(data_buffer[0]);
  UART1_Put_Char(data_buffer[1]);
  UART1_Put_Char(data_buffer[2]);
  UART1_Put_Char(data_buffer[3]);
  UART1_Put_Char(data_buffer[4]);
  UART1_Put_Char(data_buffer[5]);
  UART1_Put_Char(data_buffer[6]);
  
  UART1_Put_Char(0xBB); // ֡β
}



// ����Ŀ���ٶ� �˴�set��pwmֵ
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
