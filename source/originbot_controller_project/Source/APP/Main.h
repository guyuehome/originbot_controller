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

#ifndef _MAIN_H_
#define _MAIN_H_

#include "stm32f10x_exti.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_tim.h"
#include "misc.h"
#include "stm32f10x.h"
#include "stm32f10x_adc.h"
#include "stm32f10x_usart.h"

#include "timer.h"
#include "motor.h"
#include "encoder.h"
#include "app_motion_control.h"
#include "pid.h"
#include "protocol.h"

#define ENABLE_MOTION_CONTROL        1
#define ENABLE_CHECKSUM              1
#define ENABLE_CLEAR_RXBUF           1
#define VERSION_MAJOR                1
#define VERSION_MINOR                0


#ifndef uint8
#define uint8	unsigned char 
#endif

#ifndef uint16
#define uint16	unsigned short 
#endif

#ifndef int32
#define int32	int
#endif

#ifndef uint32
#define uint32	unsigned int
#endif

#endif

