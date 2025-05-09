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

#include "motor.h"
#include "Main.h"

#define PWM1 TIM2->CCR1
#define PWM2 TIM2->CCR2
#define PWM3 TIM2->CCR3
#define PWM4 TIM2->CCR4

// 自定义绝对值函数
static int myabs(int a)
{
  int temp;
  if (a < 0)
    temp = -a;
  else
    temp = a;
  return temp;
}

// 初始化电机引脚
void MOTOR_GPIO_Init(void) 
{
  GPIO_InitTypeDef GPIO_InitStructure;

  // 开启外设时钟
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE); 

  // 选择要控制的GPIO引脚
  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;        // 推挽输出
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

  GPIO_Init(GPIOA, &GPIO_InitStructure);
}

// 关闭小车的刹车功能，让车轮不再保持阻力
void Motor_Close_Brake(void)
{
  PWM1 = MOTOR_MAX_PULSE;
  PWM2 = MOTOR_MAX_PULSE;
  PWM3 = MOTOR_MAX_PULSE;
  PWM4 = MOTOR_MAX_PULSE;
}

// 电机PWM口初始化, arr：自动重装值  psc：时钟预分频数
void Motor_PWM_Init(u16 arr, u16 psc)
{
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
  TIM_OCInitTypeDef  TIM_OCInitStructure;
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE); 
  // 重新将Timer设置为缺省值
  TIM_DeInit(TIM2);
  
  // 设置计数溢出大小，每计xxx个数就产生一个更新事件 
  TIM_TimeBaseStructure.TIM_Period = arr - 1 ;
  // 预分频系数为0，即不进行预分频，此时TIMER的频率为72MHzre.TIM_Prescaler =0;
  TIM_TimeBaseStructure.TIM_Prescaler = psc;
  TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1 ;    // 设置时钟分频系数：不分频
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; // 向上计数模式

  TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
  
  // 设置缺省值
  TIM_OCStructInit(&TIM_OCInitStructure);
  
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;              // 配置为PWM模式1
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;  //比较输出使能
  TIM_OCInitStructure.TIM_Pulse = 0;       // 设置跳变值，当计数器计数到这个值时，电平发生跳变
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;       // 当定时器计数值小于跳变值时为低电平
  TIM_OC1Init(TIM2, &TIM_OCInitStructure); //使能通道1
  TIM_OC1PreloadConfig(TIM2, TIM_OCPreload_Enable);

  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;             // 配置为PWM模式1
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; // 比较输出使能
  TIM_OCInitStructure.TIM_Pulse = 0;       // 设置跳变值，当计数器计数到这个值时，电平发生跳变
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;      // 当定时器计数值小于跳变值时为低电平
  TIM_OC2Init(TIM2, &TIM_OCInitStructure); // 使能通道2
  TIM_OC2PreloadConfig(TIM2, TIM_OCPreload_Enable);
  
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;             // 配置为PWM模式1
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; // 比较输出使能
  TIM_OCInitStructure.TIM_Pulse = 0;       //设置跳变值，当计数器计数到这个值时，电平发生跳变
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;      // 当定时器计数值小于跳变值时为低电平
  TIM_OC3Init(TIM2, &TIM_OCInitStructure); //使能通道3
  TIM_OC3PreloadConfig(TIM2, TIM_OCPreload_Enable);

  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;             // 配置为PWM模式1
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; // 比较输出使能
  TIM_OCInitStructure.TIM_Pulse = 0;       // 设置跳变值，当计数器计数到这个值时，电平发生跳变
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;      // 当定时器计数值小于跳变值时为低电平
  TIM_OC4Init(TIM2, &TIM_OCInitStructure); // 使能通道4
  TIM_OC4PreloadConfig(TIM2, TIM_OCPreload_Enable);
  
  TIM_ARRPreloadConfig(TIM2, ENABLE);      // 使能TIM3重载寄存器ARR

  TIM_Cmd(TIM2, ENABLE);                   //使能定时器2  
}

void Motor_m1_pwm(int speed)
{
  if (speed >= 0) {
    PWM1 = 0;
    PWM2 = speed;
  } else {
    PWM1 = myabs(speed);
    PWM2 = 0;
  }
}

void Motor_m2_pwm(int speed)
{
  if (speed >= 0) {
    PWM3 = speed;
    PWM4 = 0;
  } else {
    PWM3 = 0;
    PWM4 = myabs(speed);
  }
}

// 设置电机速度，speed:±3600, 0为停止
void Motor_Set_Pwm(u8 id, int speed)
{
  // 限制输入
  if (speed > MOTOR_MAX_PULSE) speed = MOTOR_MAX_PULSE;
  if (speed < -MOTOR_MAX_PULSE) speed = -MOTOR_MAX_PULSE;

  switch (id) {
  case MOTOR_ID_1:
  {
    Motor_m1_pwm(speed);
    break;
  }
  
  case MOTOR_ID_2:
  {
    Motor_m2_pwm(speed);
    break;
  }
  
  default:
    break;
  }
}


