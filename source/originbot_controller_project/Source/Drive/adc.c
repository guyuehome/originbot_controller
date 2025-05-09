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
#include "adc.h"
#include "delay.h"
#include "stm32f10x_adc.h"
#include "Main.h"
#include "UART1.h"

// 电池低电压检测计数阈值，乘50毫秒就是延迟时间，单位为毫秒
// 例如：10*100=1000，即1秒
#define BAT_CHECK_COUNT         (100)
#define BAT_LOW_POWER_THRESHOLD (4.4f) // 低于9.6V则认为电池电量过低

u8 g_bat_state = 1;          // 电池低电压状态。检测到低电压后为0。只能通过复位恢复1
int Voltage_Z100 = 0;        // 电池电压值
int Voltage_Low_Count = 0;   // 低电压计数

// 初始化ADC，仅以规则通道为例，默认将开启通道0~3                                     
void Adc_Init(void)
{   
  ADC_InitTypeDef ADC_InitStructure; 
  GPIO_InitTypeDef GPIO_InitStructure;

  // 使能ADC1通道时钟
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_ADC1, ENABLE);
 
  // 设置ADC分频因子6 72M/6=12,ADC最大时间不能超过14M
  RCC_ADCCLKConfig(RCC_PCLK2_Div6);

  // PA4 作为模拟通道输入引脚       
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN; // 模拟输入引脚
  GPIO_Init(GPIOA, &GPIO_InitStructure);  

  ADC_DeInit(ADC1);  // 复位ADC1 

  // ADC工作模式: ADC1和ADC2工作在独立模式
  ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
  // 模数转换工作在单通道模式
  ADC_InitStructure.ADC_ScanConvMode = DISABLE;
  // 模数转换工作在单次转换模式
  ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
  // 转换由软件而不是外部触发启动
  ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
  // ADC数据右对齐
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
  // 顺序进行规则转换的ADC通道的数目
  ADC_InitStructure.ADC_NbrOfChannel = 1;
  // 根据ADC_InitStruct中指定的参数初始化外设ADCx的寄存器
  ADC_Init(ADC1, &ADC_InitStructure);

  // 使能指定的ADC1
  ADC_Cmd(ADC1, ENABLE);
  
  // 使能复位校准
  ADC_ResetCalibration(ADC1);
   
  // 等待复位校准结束
  while (ADC_GetResetCalibrationStatus(ADC1));
  
  // 开启AD校准
  ADC_StartCalibration(ADC1);
 
  // 等待校准结束
  while (ADC_GetCalibrationStatus(ADC1));
}

// 获得ADC值，ch:通道值 0~3
u16 Get_Adc(u8 ch)   
{
  // 设置指定ADC的规则组通道，一个序列，采样时间
  // ADC1,ADC通道,采样时间为239.5周期
  ADC_RegularChannelConfig(ADC1, ch, 1, ADC_SampleTime_239Cycles5 );              
  // 使能指定的ADC1的软件转换启动功能
  ADC_SoftwareStartConvCmd(ADC1, ENABLE); 
  // 等待转换结束
  while (!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC ));
  // 返回最近一次ADC1规则组的转换结果
  return ADC_GetConversionValue(ADC1);
}

// 获得 ADC 多次测量平均值, ch:通道值 ; times:测量次数
u16 Adc_Get_Average(u8 ch, u8 times)
{
  u32 temp_val = 0;
  u8 t;
  for (t = 0; t < times; t++)
    temp_val += Get_Adc(ch);
  return temp_val / times;
}

// 获得测得原始电压值
float Adc_Get_Measure_Volotage(void)
{
  u16 adcx;
  float temp;
  // ADC Channel 4
  adcx = Adc_Get_Average(4, 5);
  temp = (float)adcx * (3.30f / 4096);
  return temp;
}

// 获得实际电池分压前电压
float Adc_Get_Battery_Volotage(void)
{
  float temp;
  temp = Adc_Get_Measure_Volotage();
  // 实际测量值比计算值低一点点
  temp = temp * 5.00f;
  return temp;
}

// 查询电池电压状态，连续几秒读到低于9.6V返回0，高于9.6V返回1
u8 Bat_Update_Power_State(void)
{
  if (g_bat_state) {
    Voltage_Z100 = (int)(Adc_Get_Battery_Volotage() * 100);
    if (Voltage_Z100 < (BAT_LOW_POWER_THRESHOLD * 100)) {
      Voltage_Low_Count++;
      if (Voltage_Low_Count > BAT_CHECK_COUNT)
			{
				g_bat_state = 0;
			}
        
    } else {
      Voltage_Low_Count = 0;
    }
  }
  return g_bat_state;
}

// 驱动板供电是否正常，正常返回1，电压过低返回0
u8 Bat_Is_Low_Power(void)
{
  return (g_bat_state == 0);
}

// 上报电池电量
void Sensor_Send_Data(void)
{
  #define SensorLEN        7
  uint8_t data_buffer[SensorLEN] = {0};
  uint8_t i, checknum = 0;
  int bat = 0;
  
  bat = (int)(Adc_Get_Battery_Volotage() * 100); // V
  data_buffer[0] = (int) (bat / 100);
  data_buffer[1] = (int) (bat % 100);
  data_buffer[2] = 0;
  data_buffer[3] = 0;
  data_buffer[4] = 0;
  data_buffer[5] = 0;

  // 校验位的计算使用数据位各个数据相加 & 0xFF
  for (i = 0; i < SensorLEN - 1; i++)
    checknum += data_buffer[i];
  data_buffer[SensorLEN - 1] = checknum & 0xFF;

  UART1_Put_Char(0x55); // 帧头
  UART1_Put_Char(0x06); // 标识位
  UART1_Put_Char(0x06); // 数据位长度(字节数)
  
  UART1_Put_Char(data_buffer[0]);
  UART1_Put_Char(data_buffer[1]);
  UART1_Put_Char(data_buffer[2]);
  UART1_Put_Char(data_buffer[3]);
  UART1_Put_Char(data_buffer[4]);
  UART1_Put_Char(data_buffer[5]);
  UART1_Put_Char(data_buffer[6]);
  
  UART1_Put_Char(0xBB); // 帧尾
}
