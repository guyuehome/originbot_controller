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

// ×óÂÖPIDÄ¬ÈÏÖµ
#define PID_KP_DEF_L             (0.1)
#define PID_KI_DEF_L             (0.0)
#define PID_KD_DEF_L             (4.0)

// ÓÒÂÖPIDÄ¬ÈÏÖµ
#define PID_KP_DEF_R             (0.1)
#define PID_KI_DEF_R             (0.0)
#define PID_KD_DEF_R             (4.0)

#define PI 3.1415926f

struct pid_uint
{
  s32 U_kk;          //ÉÏÒ»´ÎµÄÊä³öÁ¿
  s32 ekk;       //ÉÏÒ»´ÎµÄÊäÈëÆ«²î
  s32 ekkk;      //Ç°Ò»´ÎµÄÊäÈëÆ«²î
  s32 Ur;        //ÏÞ·ùÊä³öÖµ,Ðè³õÊ¼»¯
  s32 Kp;        //±ÈÀý
  s32 Ki;        //»ý·Ö
  s32 Kd;        //Î¢·Ö
  
  u8  En;             //¿ª¹Ø
  s16 Adjust;         //µ÷½ÚÁ¿
  s16 speedSet;       //ËÙ¶ÈÉèÖÃ
  s16 speedNow;       //µ±Ç°ËÙ¶È
};

typedef struct
{
  float SetPoint;   // Éè¶¨Ä¿±êDesired value
  float Proportion; // ±ÈÀý³£ÊýProportional Const
  float Integral;   // »ý·Ö³£ÊýIntegral Const
  float Derivative; // Î¢·Ö³£ÊýDerivative Const
  float LastError;  // Error[-1]
  float PrevError;  // Error[-2]
  float SumError;   // Sums of Errors
} PID;

// ×ËÌ¬½Ç
typedef struct _attitude_t
{
    float roll;
    float pitch;
    float yaw;
} attitude_t;

extern struct pid_uint pid_Task_Left;
extern struct pid_uint pid_Task_Right;
extern attitude_t g_attitude;

void PID_Init(void);
void reset_Uk(struct pid_uint *p);
s32 PID_common(int set,int jiance,struct pid_uint *p);
void Pid_Ctrl(int *leftMotor, int *rightMotor, float yaw);
void reset_PID(struct pid_uint *p);

void PID_Reset_Yaw(float yaw);
void Left_Pid_Update_Value(float kp, float ki, float kd);
void Right_Pid_Update_Value(float kp, float ki, float kd);
float PIDCal_car(float NextPoint);

void Pid_Send_param(void);

int PID_Get_Offset(void);

int Pid_Average_Speed_B(int target, int now);
int Pid_Average_Speed_D(int target, int now);

#endif //__PID_H__
