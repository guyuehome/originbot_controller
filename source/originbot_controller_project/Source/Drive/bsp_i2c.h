#ifndef __BSP_I2C_H
#define __BSP_I2C_H

#include <string.h>
#include <stdio.h>

#include "Main.h"


#define I2C_SPEED               	400000
#define I2C_icm42670_ADDRESS7     0xD0 //这个值已经是0x68<<1


#define I2CX                    I2C2
#define I2C_SCL_PORT            GPIOB
#define I2C_SDA_PORT            GPIOB
#define I2C_SCL_PIN             GPIO_Pin_10
#define I2C_SDA_PIN             GPIO_Pin_11


#define INT_RCC   						RCC_APB2Periph_GPIOA
#define INT_PORT 							GPIOA
#define INT_PIN 							GPIO_Pin_10


void IIC_icm42607_init(void);
void IIC_WaitEvent(I2C_TypeDef* I2Cx, uint32_t I2C_EVENT);
void IIC_WriteReg(uint8_t RegAddress, uint8_t Data);
uint8_t IIC_ReadReg(uint8_t RegAddress);


void icm42670_GPIO_INT(void);




#endif



