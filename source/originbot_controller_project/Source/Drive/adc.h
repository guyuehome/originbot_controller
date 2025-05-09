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

#ifndef __ADC_H
#define __ADC_H  
#include "stm32f10x.h"

void Adc_Init(void);
u16  Get_Adc(u8 ch); 
u16  Adc_Get_Average(u8 ch, u8 times);
float Adc_Get_Measure_Volotage(void);
float Adc_Get_Battery_Volotage(void);
u8 Bat_Update_Power_State(void);
u8 Bat_Is_Low_Power(void);
void Sensor_Send_Data(void);
 
#endif 
