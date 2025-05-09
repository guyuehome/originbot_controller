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
#include "UART3.h"
#include "Main.h"

static unsigned char TxBuffer[256];
static unsigned char TxCounter=0;
static unsigned char count=0;
extern void CopeSerial3Data(unsigned char ucData);

void UART3_Init(unsigned long baudrate)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  USART_InitTypeDef USART_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure; 
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
	
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
   
  USART_InitStructure.USART_BaudRate = baudrate;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No ;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
  USART_Init(USART3, &USART_InitStructure); 
  USART_ITConfig(USART3, USART_IT_TXE, DISABLE);    
  USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);
  
  USART_ClearFlag(USART3,USART_FLAG_TC);  
  USART_Cmd(USART3, ENABLE);
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2); 
  NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 4;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}

void USART3_IRQHandler(void)
{
  if (USART_GetITStatus(USART3, USART_IT_TXE) != RESET) {   
    USART_SendData(USART3, TxBuffer[TxCounter++]); 
    USART_ClearITPendingBit(USART3, USART_IT_TXE);

    if (TxCounter == count) 
      USART_ITConfig(USART3, USART_IT_TXE, DISABLE);
  } else if (USART_GetITStatus(USART3, USART_IT_RXNE) != RESET) {
    CopeSerial3Data((unsigned char)USART3->DR);
    USART_ClearITPendingBit(USART3, USART_IT_RXNE);
  }
  
  USART_ClearITPendingBit(USART3,USART_IT_ORE);
}

void UART3_Put_Char(unsigned char DataToSend)
{
  TxBuffer[count++] = DataToSend;  
  USART_ITConfig(USART3, USART_IT_TXE, ENABLE);  
}

void UART3_Put_String(unsigned char *Str)
{
  while (*Str) {
    if (*Str=='\r')
      UART3_Put_Char(0x0d);
    else if (*Str=='\n')
      UART3_Put_Char(0x0a);
    else 
      UART3_Put_Char(*Str);

    Str++;
  }
}
