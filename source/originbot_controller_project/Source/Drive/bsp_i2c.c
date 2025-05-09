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

#include "bsp_i2c.h"


void icm42670_GPIO_INT(void)
{
		GPIO_InitTypeDef GPIO_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
    EXTI_InitTypeDef EXTI_InitStructure;
		RCC_APB2PeriphClockCmd(INT_RCC, ENABLE);
    
    
    GPIO_InitStructure.GPIO_Mode=GPIO_Mode_IPU;
    GPIO_InitStructure.GPIO_Pin=INT_PIN;
    GPIO_Init(INT_PORT,&GPIO_InitStructure);
	
    
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource15);
    EXTI_ClearITPendingBit(EXTI_Line15);
    
    NVIC_InitStructure.NVIC_IRQChannel=EXTI15_10_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=2;//抢占式优先级，按自己需求配置
    NVIC_InitStructure.NVIC_IRQChannelSubPriority=3;//响应式优先级
    NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
    NVIC_Init(&NVIC_InitStructure);

	//外部中断
		EXTI_InitStructure.EXTI_Line=EXTI_Line15;
    EXTI_InitStructure.EXTI_Mode=EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger=EXTI_Trigger_Rising_Falling;//
    EXTI_InitStructure.EXTI_LineCmd=ENABLE;
		EXTI_Init(&EXTI_InitStructure);  

}




void IIC_icm42607_init(void)
{
	I2C_InitTypeDef  I2C_InitStructure;
	GPIO_InitTypeDef  GPIO_InitStructure; 
 
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C2,ENABLE);  //使能时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
 
	
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_10 | GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;//
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	I2C_DeInit(I2CX);
	I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
	I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
	I2C_InitStructure.I2C_OwnAddress1 = 0x09;
	I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
	I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
	I2C_InitStructure.I2C_ClockSpeed = I2C_SPEED;//400K
	
	I2C_Init(I2CX, &I2C_InitStructure);
	I2C_Cmd(I2CX, ENABLE);
	
}

//延时函数
void IIC_WaitEvent(I2C_TypeDef* I2Cx, uint32_t I2C_EVENT)
{
	uint32_t Timeout;
	Timeout = 1000;
	while (I2C_CheckEvent(I2Cx, I2C_EVENT) != SUCCESS)
	{
		Timeout --;
		if (Timeout == 0)
		{
			break;
		}
	}
}



//参数1：要写入的数据的对应寄存器的地址。参数2：要写入的数据。
//功能：配置指定寄存器的数据，进而配置模块的功能
void IIC_WriteReg(uint8_t RegAddress, uint8_t Data)
{
	I2C_GenerateSTART(I2CX, ENABLE);
	IIC_WaitEvent(I2CX, I2C_EVENT_MASTER_MODE_SELECT);
	
	I2C_Send7bitAddress(I2CX, I2C_icm42670_ADDRESS7, I2C_Direction_Transmitter);
	IIC_WaitEvent(I2CX, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED);
	
	I2C_SendData(I2CX, RegAddress);
	IIC_WaitEvent(I2CX, I2C_EVENT_MASTER_BYTE_TRANSMITTING);
	
	I2C_SendData(I2CX, Data);
	IIC_WaitEvent(I2CX, I2C_EVENT_MASTER_BYTE_TRANSMITTED);
	
	I2C_GenerateSTOP(I2CX, ENABLE);
}
 
//参数1：要读取的寄存器的地址.   返回值：返回值为读取的寄存器上存储的数据。
//功能：读取指定寄存器上的数据。
uint8_t IIC_ReadReg(uint8_t RegAddress)
{
	uint8_t Data;
	
	I2C_GenerateSTART(I2CX, ENABLE);
	IIC_WaitEvent(I2CX, I2C_EVENT_MASTER_MODE_SELECT);
	
	I2C_Send7bitAddress(I2CX, I2C_icm42670_ADDRESS7, I2C_Direction_Transmitter);
	IIC_WaitEvent(I2CX, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED);
	
	I2C_SendData(I2CX, RegAddress);
	IIC_WaitEvent(I2CX, I2C_EVENT_MASTER_BYTE_TRANSMITTED);
	
	I2C_GenerateSTART(I2CX, ENABLE);
	IIC_WaitEvent(I2CX, I2C_EVENT_MASTER_MODE_SELECT);
	
	I2C_Send7bitAddress(I2CX, I2C_icm42670_ADDRESS7, I2C_Direction_Receiver);
	IIC_WaitEvent(I2CX, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED);
	
	I2C_AcknowledgeConfig(I2CX, DISABLE);
	I2C_GenerateSTOP(I2CX, ENABLE);
	
	IIC_WaitEvent(I2CX, I2C_EVENT_MASTER_BYTE_RECEIVED);
	Data = I2C_ReceiveData(I2CX);
	
	I2C_AcknowledgeConfig(I2CX, ENABLE);
	
	return Data;
}

