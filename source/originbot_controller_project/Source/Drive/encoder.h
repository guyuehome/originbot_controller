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

#ifndef __ENCODER_H__
#define __ENCODER_H__

#include "main.h"

// 轮子转一整圈，编码器获得的脉冲数
#define ENCODER_CIRCLE           (2340)

// 轮子转一整圈，履带前进的距离(单位为：米)
#define DISTANCE_CIRCLE          (0.2136283)

// 轮子转一整圈的位移，单位为cm 
#define DISTANCE_CIRCLE_CM       (21.36283)

// 不可大于65535 因为F103的定时器是16位的
#define ENCODER_TIM_PERIOD (u16)(65535)

#define Hal_1A_PIN   GPIO_Pin_6
#define Hal_1A_PORT  GPIOA
#define Hal_1A_RCC   RCC_APB2Periph_GPIOA
#define Hal_1B_PIN   GPIO_Pin_7
#define Hal_1B_PORT  GPIOA
#define Hal_1B_RCC   RCC_APB2Periph_GPIOA

#define Hal_2A_PIN   GPIO_Pin_6
#define Hal_2A_PORT  GPIOB
#define Hal_2A_RCC   RCC_APB2Periph_GPIOB
#define Hal_2B_PIN   GPIO_Pin_7
#define Hal_2B_PORT  GPIOB
#define Hal_2B_RCC   RCC_APB2Periph_GPIOB

#define ENCODER_ID_A        2
#define ENCODER_ID_B        4

void Encoder_Init(void);

s16 getTIMx_DetaCnt(TIM_TypeDef * TIMx);
s16 Encoder_Read_CNT(u8 Encoder_id);

int Encoder_Get_Count_Now(u8 Encoder_id);
void Encoder_Update_Count(u8 Encoder_id);

#endif

