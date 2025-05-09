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

#include "DIO.h"

// IO初始化
void GPIO_Config(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC| RCC_APB2Periph_AFIO, ENABLE);
#ifdef NEW_BOARD_20220808
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB| RCC_APB2Periph_AFIO, ENABLE);
#else
  GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable , ENABLE); // 改变指定管脚的映射
#endif // NEW_BOARD_20220808
  GPIO_InitStructure.GPIO_Pin = LED_PIN | BUZZER_PIN;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; // 设置引脚模式为通用推挽输出
  GPIO_Init(GPIOC, &GPIO_InitStructure);
#ifdef NEW_BOARD_20220808
  GPIO_Init(GPIOB, &GPIO_InitStructure);
#endif // NEW_BOARD_20220808
}

// LED开
void LED_ON(void)
{
  GPIO_ResetBits(LED_PORT, LED_PIN);
}

// LED关
void LED_OFF(void)
{
  GPIO_SetBits(LED_PORT, LED_PIN);
}

// 蜂鸣器开
void BUZZER_ON(void)
{
  GPIO_SetBits(BUZZER_PORT, BUZZER_PIN);
}

// 蜂鸣器关
void BUZZER_OFF(void)
{
  GPIO_ResetBits(BUZZER_PORT, BUZZER_PIN);
}
