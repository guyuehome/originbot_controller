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
#include <stdio.h>
#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_usart.h"
#include "stm32f10x_rcc.h"
#include "misc.h"
#include "UART1.h"
#include "protocol.h"

static unsigned char TxBuffer[256];
static unsigned char TxCounter=0;
static unsigned char count=0;

void UART1_Init(unsigned long baudrate)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  USART_InitTypeDef USART_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure; 
	//GPIOA时钟使能
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 | RCC_APB2Periph_GPIOA, ENABLE);
	//选择引脚
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	//设置推挽输出模式
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	//输出速度50KHZ
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	//初始化GPIOA
  GPIO_Init(GPIOA, &GPIO_InitStructure);    
  //选择引脚
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	//设置为浮空输入模式
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
	
  //设置波特率为baudrate，主函数当中调用为115200
  USART_InitStructure.USART_BaudRate = baudrate;
	//设置数据位长度为8位
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	//设置停止位为1
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
	//设置为无校验位
  USART_InitStructure.USART_Parity = USART_Parity_No ;
	//设置为无硬件控制流
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	//设置模式为接收模式或发送模式
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	//初始化串口USART1
  USART_Init(USART1, &USART_InitStructure); 
	//关闭此中断
  USART_ITConfig(USART1, USART_IT_TXE, DISABLE);  
	//开启此中断
  USART_ITConfig(USART1, USART_IT_RXNE, ENABLE); 
	//清除中断标志位
  USART_ClearFlag(USART1,USART_FLAG_TC);
	
	//开启中断
  USART_Cmd(USART1, ENABLE);
	//设置两位抢占优先和两位响应优先
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2); 
	//UsartNVIC配置
  NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	//抢占优先级设为1
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	//响应优先级设置为3
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
	//中断通道启用
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	//初始化中断
  NVIC_Init(&NVIC_InitStructure);
}


//发送数据
void UART1_Put_Char(unsigned char DataToSend)
{
  TxBuffer[count++] = DataToSend;               //缓存将要发送到数据
  USART_ITConfig(USART1, USART_IT_TXE, ENABLE); //开启中断 
}


//发送字符串
void UART1_Put_String(unsigned char *Str)
{
  while (*Str) {
    if (*Str=='\r')
      UART1_Put_Char(0x0d);
    else if (*Str=='\n')
      UART1_Put_Char(0x0a);
    else 
      UART1_Put_Char(*Str);

    Str++;
  }
}

// 重定义fputc函数 
int fputc(int ch, FILE *f)
{
  // 循环发送,直到发送完毕
  while ((USART1->SR & 0X40) == 0);

  USART1->DR = (u8)ch;      
  return ch;
}


//中断服务函数
void USART1_IRQHandler(void)
{
  u8 Rx1_Temp = 0;

  if (USART_GetITStatus(USART1, USART_IT_TXE) != RESET) {   
    USART_SendData(USART1, TxBuffer[TxCounter++]);

    if (TxCounter == count)
      USART_ITConfig(USART1, USART_IT_TXE, DISABLE);
      
    USART_ClearITPendingBit(USART1, USART_IT_TXE); 
  }

  if (USART_GetITStatus(USART1, USART_IT_RXNE) != RESET) {   
    Rx1_Temp = USART_ReceiveData(USART1);
    Upper_Data_Receive(Rx1_Temp);
  }
}
