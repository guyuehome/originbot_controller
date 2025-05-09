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

#ifndef __DIO_H
#define __DIO_H 

#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"

#define NEW_BOARD_20220808

#define LED_PORT        GPIOC
#define LED_PIN        GPIO_Pin_13

#ifdef NEW_BOARD_20220808
#define BUZZER_PORT        GPIOB
#define BUZZER_PIN        GPIO_Pin_13
#else
#define BUZZER_PORT        GPIOC
#define BUZZER_PIN        GPIO_Pin_14
#endif // NEW_BOARD_20220808

void LED_ON(void);
void LED_OFF(void);

void BUZZER_ON(void);
void BUZZER_OFF(void);

void GPIO_Config(void);
#endif



