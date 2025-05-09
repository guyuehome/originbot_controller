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
#include "delay.h"
#include "misc.h"   
static u8  fac_us=0; // us延时倍乘数
static u16 fac_ms=0; // ms延时倍乘数

// SYSTICK的时钟固定为HCLK时钟的1/8
// SYSCLK:系统时钟
// 初始化延迟系统，使延时程序进入可用状态
void SysTick_init(u8 SYSCLK,u16 nms)
{
  NVIC_InitTypeDef NVIC_InitStructure;
  SysTick->VAL =0x00;             // 清空计数器
  SysTick->LOAD = nms*SYSCLK*125; // 72MHz,最大1864ms
  SysTick->CTRL=3;                // bit2清空,选择外部时钟  HCLK/8
  fac_us=SYSCLK/8;        
  fac_ms=(u16)fac_us*1000;
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
  
  NVIC_InitStructure.NVIC_IRQChannel = (uint8_t)SysTick_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;  
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}

volatile unsigned char ucTimeFlag = 0,ucDelayFlag = 0;

void SysTick_Handler(void) 
{      
  ucTimeFlag=1;
  if (ucDelayFlag)
    ucDelayFlag--;
}

unsigned char CheckSystemTick(void)
{  
  if (ucTimeFlag) {
    ucTimeFlag = 0;
    return 1;
  }

  return 0;
}

// SysTick->LOAD为24位寄存器,所以,最大延时为:
// nms<=0xffffff*8*1000/SYSCLK
// SYSCLK单位为Hz,nms单位为ms
// 毫秒级延时  延时nms  nms<=1864
void Delay_Ms(u16 nms)
{
  u32 Start = SysTick->VAL;
  u32 Span = (u32)nms * fac_ms; // 时间加载(SysTick->LOAD为24bit)
  u32 End = 0;
  ucDelayFlag = Span / SysTick->LOAD;
  End = Span % SysTick->LOAD;
  if (Start > End) {
    End = Start - End;
    while (ucDelayFlag);
    if (End < 10)
      while (SysTick->VAL > 10);
    else      
      while (SysTick->VAL > End); 
  } else {
    ucDelayFlag++;
    End = (Start + SysTick->LOAD) - End;
    while (ucDelayFlag);
    if (End < 10)
      while (SysTick->VAL > 10);
    else      
      while (SysTick->VAL > End); 
  }            
}   

// 微秒级延时  延时nus  nms<=1864                       
void delay_us(u32 nus)
{    
  u32 Start = SysTick->VAL;
  u32 Span = (u32)nus * fac_us; // 时间加载(SysTick->LOAD为24bit)
  u32 End = 0;
  ucDelayFlag = Span / SysTick->LOAD;
  End = Span % SysTick->LOAD;
  if (Start > End) {
    End = Start-End;
    while (ucDelayFlag);
    if (End < 10)
      while (SysTick->VAL > 10);
    else
      while (SysTick->VAL > End); 
  } else {
    ucDelayFlag++;
    End = (Start + SysTick->LOAD) - End;
    while (ucDelayFlag);      
    if (End < 10)
      while (SysTick->VAL > 10);
    else
      while (SysTick->VAL > End); 
  }            
}
