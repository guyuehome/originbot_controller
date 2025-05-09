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
#ifndef __TIMER_H__
#define __TIMER_H__

#include "stm32f10x.h"

#define COUNT_BEAT_ID        1

void TIM1_Init(void);

u16 Timer_Get_Count(u8 id);
void Timer_Set_Count(u8 id, u16 value);
void Timer_Count_Auto_Reduce(u8 id);

#endif
