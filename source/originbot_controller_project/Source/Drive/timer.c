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
#include "timer.h"
#include "Main.h"

#include <stdio.h>
#include "DIO.h"

// 系统心跳包 
u16 Count_Beat = 0;
int temp = 0;

u16 Timer_Get_Count(u8 id)
{
  if (id == COUNT_BEAT_ID)
    return Count_Beat;
  return 65535;
}

void Timer_Set_Count(u8 id, u16 value)
{
  if (id == COUNT_BEAT_ID)
    Count_Beat = value;
}

void Timer_Count_Auto_Reduce(u8 id)
{
  if (id == COUNT_BEAT_ID)
    Count_Beat--;
}

// TIM1初始化，定时10毫秒
void TIM1_Init(void)
{   
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
  NVIC_InitTypeDef NVIC_InitStructure;

  RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE); // TIM1时钟使能

  TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
  // 设置在下一个更新事件装入活动的自动重装载寄存器周期的值，计数到5000为500ms
  TIM_TimeBaseStructure.TIM_Period = 99;
  // 设置用来作为TIMx时钟频率除数的预分频值，10Khz的计数频率  
  TIM_TimeBaseStructure.TIM_Prescaler =7199;
  // 设置时钟分割:TDTS = Tck_tim
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  // TIM向上计数模式
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  // 根据TIM_TimeBaseInitStruct中指定的参数初始化TIMx的时间基数单位
  TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);
  
  TIM_ITConfig(TIM1, TIM_IT_Update, ENABLE);
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2); 
  NVIC_InitStructure.NVIC_IRQChannel = TIM1_UP_IRQn;  // TIM1中断  
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2; // 先占优先级3级
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;  // 从优先级0级
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;     // IRQ通道被使能
  NVIC_Init(&NVIC_InitStructure);
  
  TIM_Cmd(TIM1, ENABLE); 
}
