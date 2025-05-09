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
#ifndef __PID_H__
#define __PID_H__

#include "stm32f10x.h"

// PID默认值 ，可在此处修改PID默认值
#define PID_KP_DEF             (0.8)
#define PID_KI_DEF             (0.002)
#define PID_KD_DEF             (1.0)

struct pid_uint
{
  s32 U_kk;           //上一次的输出量
  s32 ekk;            //上一次的输入偏差
  s32 ekkk;           //前一次的输入偏差
  s32 Ur;             //限幅输出值,需初始化
  s32 Kp;             //比例
  s32 Ki;             //积分
	s32 Kd;             //微分
  
  u8  En;             //开关
  s16 Adjust;         //调节量
  s16 speedSet;       //速度设置
  s16 speedNow;       //当前速度
};

typedef struct
{
  float SetPoint;   // 设定目标Desired value
  float Proportion; // 比例常数Proportional Const
  float Integral;   // 积分常数Integral Const
  float Derivative; // 微分常数Derivative Const
  float LastError;  // Error[-1]
  float PrevError;  // Error[-2]
  float SumError;   // Sums of Errors
} PID;

// 姿态角
typedef struct _attitude_t
{
    float roll;
    float pitch;
    float yaw;
} attitude_t;

//一阶滤波
struct lowpass_filter {
  float y_k;      // 上一次的输出
  float alpha;    // 滤波器系数
};

extern struct pid_uint pid_Task_Left;
extern struct pid_uint pid_Task_Right;
extern attitude_t g_attitude;

void PID_Init(void);
void reset_Uk(struct pid_uint *p);
s32 PID_common(int set,int feedback,struct pid_uint *p,struct lowpass_filter *lpf);
void Pid_Ctrl(int *leftMotor, int *rightMotor, float yaw);
void reset_PID(struct pid_uint *p);
void Left_Pid_Update_Value(float kp, float ki, float kd);
void Right_Pid_Update_Value(float kp, float ki, float kd);

#endif //__PID_H__
