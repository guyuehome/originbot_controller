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
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 | RCC_APB2Periph_GPIOA, ENABLE);
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);    

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
    
  USART_InitStructure.USART_BaudRate = baudrate;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No ;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
  USART_Init(USART1, &USART_InitStructure); 
  USART_ITConfig(USART1, USART_IT_TXE, DISABLE);  
  USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);        
  USART_ClearFlag(USART1,USART_FLAG_TC);
  USART_Cmd(USART1, ENABLE);
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2); 
  NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}

void UART1_Put_Char(unsigned char DataToSend)
{
  TxBuffer[count++] = DataToSend;  
  USART_ITConfig(USART1, USART_IT_TXE, ENABLE);  
}

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
