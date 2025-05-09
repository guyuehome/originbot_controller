/***********************************************************************
Copyright (c) 2022, www.guyuehome.com

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

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

void Get_Motor_Speed(int *leftSpeed, int *rightSpeed);

void Motion_Set_PWM(int motor_Left, int motor_Right);

void Motion_Send_Data(void);

void Motion_Control_10ms(void);

void Motion_Test_SpeedSet(uint8_t index_l, int16_t left, 
                          uint8_t index_r, int16_t right);

#endif
