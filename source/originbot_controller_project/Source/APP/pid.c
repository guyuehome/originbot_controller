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
#include "pid.h"
#include "Main.h"

attitude_t g_attitude;

struct pid_uint pid_Task_Left;
struct pid_uint pid_Task_Right;


int s_offset = 0; //偏移量设置

// PID
float f_kp = PID_KP_DEF;
float f_ki = PID_KI_DEF;
float f_kd = PID_KD_DEF;

//初始化滤波器
struct lowpass_filter lpf = { 0, 1 };

void PID_Init(void)
{
  // 乘1024转换为整型运算，避免浮点数运算
  // 左轮速度pid
  pid_Task_Left.Kp = 1024 * PID_KP_DEF;
  pid_Task_Left.Ki = 1024 * PID_KI_DEF;
  pid_Task_Left.Kd = 1024 * PID_KD_DEF;
  pid_Task_Left.Ur = 1024 * MOTOR_MAX_PULSE;
  pid_Task_Left.Adjust = 0;
  pid_Task_Left.En = 1;
  pid_Task_Left.speedSet = 0;
  pid_Task_Left.speedNow = 0;
  reset_Uk(&pid_Task_Left);

  // 右轮速度pid
  pid_Task_Right.Kp = 1024 * PID_KP_DEF;
  pid_Task_Right.Ki = 1024 * PID_KI_DEF;
  pid_Task_Right.Kd = 1024 * PID_KD_DEF;
  pid_Task_Right.Ur = 1024 * MOTOR_MAX_PULSE;
  pid_Task_Right.Adjust = 0;
  pid_Task_Right.En = 1;
  pid_Task_Right.speedSet = 0;
  pid_Task_Right.speedNow = 0;
  reset_Uk(&pid_Task_Right);
}

// 在初始化时调用，改变PID参数时有可能需要调用
void reset_Uk(struct pid_uint *p)
{
  p->U_kk = 0;
  p->ekk = 0;
  p->ekkk = 0;
}
//复位PID值
void reset_PID(struct pid_uint *p)
{
  p->U_kk = 0;
  p->ekk = 0;
  p->ekkk = 0;
  p->Adjust = 0;
  p->speedNow = 0;
  p->speedSet = 0;
}

// PID计算，求任意单个PID的控制量
// 入口参数：期望值，实测值，PID单元结构体
// 返 回 值：PID控制量
s32 PID_common(int set, int feedback, struct pid_uint *p, struct lowpass_filter *lpf)
{
  int ek = 0, U_k = 0;

  ek = feedback - set;

  U_k = p->U_kk + p->Kp * (ek - p->ekk) + p->Ki * ek + p->Kd * (ek - 2 * p->ekk + p->ekkk);
	
	U_k = lpf->y_k + lpf->alpha * (U_k - lpf->y_k);
  lpf->y_k = U_k;

  p->U_kk = U_k;
  p->ekkk = p->ekk;
  p->ekk = ek;
	
  if (U_k > (p->Ur))
    U_k = p->Ur;
  if (U_k < -(p->Ur))
    U_k = -(p->Ur);

  return U_k >> 10;
}




// pid选择函数
void Pid_Which(struct pid_uint *pl, struct pid_uint *pr, float yaw)
{
  s_offset = 0;

  // 左轮速度pid
  if (pl->En == 1) {
    pl->Adjust = -PID_common(pl->speedSet, pl->speedNow, pl, &lpf) - s_offset;
  } else {
    pl->Adjust =  0;
    reset_Uk(pl);
    pl->En = 2;
  }

  // 右轮速度pid
  if (pr->En == 1) {
    pr->Adjust = -PID_common(pr->speedSet, pr->speedNow, pr, &lpf) + s_offset;
  } else {
    pr->Adjust = 0;
    reset_Uk(pr);
    pr->En = 2;
  }
}




// PID控制
void Pid_Ctrl(int *leftMotor, int *rightMotor, float yaw)
{
  int temp_l = *leftMotor;
  int temp_r = *rightMotor;
  Pid_Which(&pid_Task_Left, &pid_Task_Right, yaw);
  temp_l += pid_Task_Left.Adjust;
  temp_r += pid_Task_Right.Adjust;

  if (temp_l > MOTOR_MAX_PULSE)  temp_l = MOTOR_MAX_PULSE;
  if (temp_l < -MOTOR_MAX_PULSE) temp_l = -MOTOR_MAX_PULSE;
  if (temp_r > MOTOR_MAX_PULSE)  temp_r = MOTOR_MAX_PULSE;
  if (temp_r < -MOTOR_MAX_PULSE) temp_r = -MOTOR_MAX_PULSE;

  *leftMotor  = temp_l;
  *rightMotor = temp_r;
}

// 设置左轮PID参数
void Left_Pid_Update_Value(float kp, float ki, float kd)
{
  if (kp > 10 || ki > 10 || kd > 10) 
    return;

  f_kp = kp;
  f_ki = ki;
  f_kd = kd;

  // 乘1024转换为整型运算，避免浮点数运算
  // 左轮速度pid
  pid_Task_Left.Kp = 1024 * f_kp;
  pid_Task_Left.Ki = 1024 * f_ki;
  pid_Task_Left.Kd = 1024 * f_kd;
  reset_Uk(&pid_Task_Left);
}

// 设置右轮PID参数
void Right_Pid_Update_Value(float kp, float ki, float kd)
{
  if (kp > 10 || ki > 10 || kd > 10) 
    return;

  f_kp = kp;
  f_ki = ki;
  f_kd = kd;

  // 乘1024转换为整型运算，避免浮点数运算
  // 右轮速度pid
  pid_Task_Right.Kp = 1024 * f_kp;
  pid_Task_Right.Ki = 1024 * f_ki;
  pid_Task_Right.Kd = 1024 * f_kd;
  reset_Uk(&pid_Task_Right);
}
