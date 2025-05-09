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
#ifndef __APP_MOTION_CONTROL_H__
#define __APP_MOTION_CONTROL_H__   

#include "stdint.h"
#include "stm32f10x.h"


// 此处定义与电机/编码器硬件相关参数

// 金属输出轴编码器电机：
// 电机转一圈单相输出13个脉冲，1:45减速比，电机输出轴转一圈最大输出(45*13*4) 2340个计数
// AB相输出脉冲信号相位差为90°，可检测电机转动方向
#define ENCODER_CNT_PER_ROUND_BEFORE_REDUCTION       (13)        //编码器记录脉冲数
#define REDUCTION_RATIO                              (45)        //减速比
#define ENCODER_CNT_PER_ROUND                        (2340)      //编码器计数

#define WHEEL_CIRCUMFERENCE_CM                       (21.36283)  //轮子周长

// 限制最大速度约为 1360mm/s
#define SPD_MM_S_MAX                                 (1360.0)


// 此处计算一些常用的系数
#define ENCODER_CNT_10MS_2_SPD_MM_S (100.0 * WHEEL_CIRCUMFERENCE_CM * 10 / ENCODER_CNT_PER_ROUND)


void Get_Motor_Speed(int *leftSpeed, int *rightSpeed);

void Motion_Set_PWM(int motor_Left, int motor_Right);

void Motion_Send_Data(void);

void Motion_Control_10ms(void);

void Motion_Test_SpeedSet(uint8_t index_l, int16_t left, 
                          uint8_t index_r, int16_t right);

#endif
