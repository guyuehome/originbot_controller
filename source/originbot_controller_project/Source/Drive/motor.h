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

#ifndef __MOTOR_H
#define __MOTOR_H

#include "stm32f10x.h"

#define MOTOR_ID_1     1
#define MOTOR_ID_2     2

#define MOTOR_IGNORE_PULSE    0
// PWM的最大输出值
#define MOTOR_MAX_PULSE       3600
#define MOTOR_FREQ_DIVIDE     0

static int myabs(int a);
void MOTOR_GPIO_Init(void);
void Motor_Close_Brake(void);
void Motor_PWM_Init(u16 arr, u16 psc);
void Motor_m1_pwm(int speed);
void Motor_m2_pwm(int speed);
void Motor_Set_Pwm(u8 id, int speed);

#endif
