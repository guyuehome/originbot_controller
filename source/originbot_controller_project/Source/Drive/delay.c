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
void delay_us(u32 nus)
{		
	u32 ticks;
	u32 told,tnow,tcnt=0;
	u32 reload=SysTick->LOAD;				//LOAD的值	    	 
	ticks=nus*fac_us; 						//需要的节拍数 
	told=SysTick->VAL;        				//刚进入时的计数器值
	while(1)
	{
		tnow=SysTick->VAL;	
		if(tnow!=told)
		{	    
			if(tnow<told)tcnt+=told-tnow;	//这里注意一下SYSTICK是一个递减的计数器就可以了.
			else tcnt+=reload-tnow+told;	    
			told=tnow;
			if(tcnt>=ticks)break;			//时间超过/等于要延迟的时间,则退出.
		}  
	};										    
}  

void Delay_Ms(u16 nms)
{	
	if(xTaskGetSchedulerState()!=taskSCHEDULER_NOT_STARTED)//系统已经运行
	{		
		if(nms>=fac_ms)
		{ 
   			vTaskDelay(nms/fac_ms);
		}
		nms%=fac_ms;  
	}
	delay_us((u16)(nms*1000));
}

unsigned char ucTimeFlag = 0,ucDelayFlag = 0;
extern void xPortSysTickHandler(void);
void SysTick_Handler(void) 
{      
    if(xTaskGetSchedulerState()!=taskSCHEDULER_NOT_STARTED)//系统已经运行
    {
        xPortSysTickHandler();	
    }
}
