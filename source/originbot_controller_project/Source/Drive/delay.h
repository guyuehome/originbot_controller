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
#ifndef __DELAY_H
#define __DELAY_H          
#include "stm32f10x.h"

void SysTick_init(u8 SYSCLK,u16 nms);
void delay_init(u8 SYSCLK);
void Delay_Ms(u16 nms);
void delay_us(u32 nus);

#endif
