
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

#include "stm32f10x_i2c.h"

#include "delay.h"
#include "timer.h"
#include "motor.h"
#include "encoder.h"
#include "app_motion_control.h"
#include "pid.h"
#include "protocol.h"
#include "UART1.h"
#include "adc.h"
#include "bsp_i2c.h"
#include "DIO.h"
#include "inv_time.h"

#include "app_imu_42670P.h"

#define ENABLE_MOTION_CONTROL        1
#define ENABLE_CHECKSUM              1
#define ENABLE_CLEAR_RXBUF           1

#define ENABLE_ICM42670P						 1  //使用的是icm42670P的imu

#define VERSION_MAJOR								 2
#define VERSION_MINOR								 0



#endif

