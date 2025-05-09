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
static u8  fac_us=0; // us��ʱ������
static u16 fac_ms=0; // ms��ʱ������

// SYSTICK��ʱ�ӹ̶�ΪHCLKʱ�ӵ�1/8
// SYSCLK:ϵͳʱ��
// ��ʼ���ӳ�ϵͳ��ʹ��ʱ����������״̬
void SysTick_init(u8 SYSCLK,u16 nms)
{
  NVIC_InitTypeDef NVIC_InitStructure;
  SysTick->VAL =0x00;             // ��ռ�����
  SysTick->LOAD = nms*SYSCLK*125; // 72MHz,���1864ms
  SysTick->CTRL=3;                // bit2���,ѡ���ⲿʱ��  HCLK/8
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
	u32 reload=SysTick->LOAD;				//LOAD��ֵ	    	 
	ticks=nus*fac_us; 						//��Ҫ�Ľ����� 
	told=SysTick->VAL;        				//�ս���ʱ�ļ�����ֵ
	while(1)
	{
		tnow=SysTick->VAL;	
		if(tnow!=told)
		{	    
			if(tnow<told)tcnt+=told-tnow;	//����ע��һ��SYSTICK��һ���ݼ��ļ������Ϳ�����.
			else tcnt+=reload-tnow+told;	    
			told=tnow;
			if(tcnt>=ticks)break;			//ʱ�䳬��/����Ҫ�ӳٵ�ʱ��,���˳�.
		}  
	};										    
}  

void Delay_Ms(u16 nms)
{	
	if(xTaskGetSchedulerState()!=taskSCHEDULER_NOT_STARTED)//ϵͳ�Ѿ�����
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
    if(xTaskGetSchedulerState()!=taskSCHEDULER_NOT_STARTED)//ϵͳ�Ѿ�����
    {
        xPortSysTickHandler();	
    }
}
