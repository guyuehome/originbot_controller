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

#include <string.h>
#include <stdio.h>
#include "JY901.h"
#include "UART3.h"
#include "UART1.h"
#include "Main.h"
#include "delay.h"

struct STime    stcTime;
struct SAcc     stcAcc;
struct SGyro    stcGyro;
struct SAngle   stcAngle;
struct SMag     stcMag;
struct SDStatus stcDStatus;
struct SPress   stcPress;
struct SLonLat  stcLonLat;
struct SGPSV    stcGPSV;
struct SQ       stcQ;

char JY_CMD_UNLOCK[5] = {0xFF,0xAA,0x69,0x88,0xB5};        	 // 陆芒脣酶
char JY_CMD_ALGORITHM[5] = {0xFF,0xAA,0x24,0x01,0x00};       // 脡猫脰脙脣茫路篓脦陋脕霉脰谩
char JY_CMD_ACC_CALIBRATION[5] = {0XFF,0XAA,0X01,0X01,0X00}; // 执行加速度校准模式
char JY_CMD_YAW_ZERO[5] = {0xFF,0xAA,0x01,0x04,0x00};        // 脰麓脨脨潞陆脧貌陆脟脰脙脕茫
char JY_CMD_SAVE_CFG[5] = {0XFF,0XAA,0X00,0X00,0X00};        // 保存当前配置

// 通过串口3给陀螺仪发送指令
void sendcmd(char cmd[])
{
  for (char i = 0; i < 5; i++)
    UART3_Put_Char(cmd[i]);
}

void jy901_init(void)
{
	sendcmd(JY_CMD_UNLOCK);
  // 陆芒脣酶
  Delay_Ms(100);
	
  sendcmd(JY_CMD_ALGORITHM);
  // 碌脠麓媒脡猫脰脙脣茫路篓
  Delay_Ms(100);
	
	sendcmd(JY_CMD_SAVE_CFG);
  Delay_Ms(100);
	//卤拢麓忙碌卤脟掳脜盲脰脙
}

void jy901_calibration(void)
{
  sendcmd(JY_CMD_UNLOCK);
  // 陆芒脣酶
  Delay_Ms(100);

  sendcmd(JY_CMD_ACC_CALIBRATION);
  // 等待模块校准完成
  Delay_Ms(100);

  sendcmd(JY_CMD_YAW_ZERO);
  // 碌脠麓媒潞陆脧貌陆脟脰脙脕茫
  Delay_Ms(100);

  sendcmd(JY_CMD_SAVE_CFG);
  Delay_Ms(100);
}

// 串口3数据处理函数，串口每收到一个数据，调用一次
void CopeSerial3Data(unsigned char ucData)
{
  static unsigned char ucRxBuffer[250];
  static unsigned char ucRxCnt = 0;  
  
  // 将收到的数据存入缓冲区中
  ucRxBuffer[ucRxCnt++] = ucData;

  // 数据头不对，则重新开始寻找0x55数据头
  if (ucRxBuffer[0] != 0x55) {
    ucRxCnt=0;
    return;
  }

  // 数据不满11个，则返回
  if (ucRxCnt < 11)
    return;

  // 判断陀螺仪数据类型
  // 部分数据需要通过维特官方提供的上位机配置，设置输出后，陀螺仪才会发送该类数据包
  // 本项目仅关注 加速度、角速度、角度 三类数据
  switch(ucRxBuffer[1]) {
    case 0x50:  memcpy(&stcTime, &ucRxBuffer[2], 8);   break;
    case 0x51:  memcpy(&stcAcc, &ucRxBuffer[2], 8);    break; // 加速度
    case 0x52:  memcpy(&stcGyro, &ucRxBuffer[2], 8);   break; // 角速度
    case 0x53:  memcpy(&stcAngle, &ucRxBuffer[2], 8);  break; // 角度
    case 0x54:  memcpy(&stcMag, &ucRxBuffer[2], 8);    break;
    case 0x55:  memcpy(&stcDStatus, &ucRxBuffer[2], 8);break;
    case 0x56:  memcpy(&stcPress, &ucRxBuffer[2], 8);  break;
    case 0x57:  memcpy(&stcLonLat, &ucRxBuffer[2], 8); break;
    case 0x58:  memcpy(&stcGPSV, &ucRxBuffer[2], 8);   break;
    case 0x59:  memcpy(&stcQ, &ucRxBuffer[2], 8);      break;
  }

  ucRxCnt = 0; // 清空缓存区
}

// 上报陀螺仪加速度
void Acc_Send_Data(void)
{
  #define AccLEN        7
  uint8_t data_buffer[AccLEN] = {0};
  uint8_t i, checknum = 0;
  
  // 低字节在前，高字节在后
  // X轴加速度
  data_buffer[0] = stcAcc.a[0]&0xFF;
  data_buffer[1] = (stcAcc.a[0]>>8)&0xFF;
  // Y轴加速度
  data_buffer[2] = stcAcc.a[1]&0xFF;
  data_buffer[3] = (stcAcc.a[1]>>8)&0xFF;
  // Z轴加速度
  data_buffer[4] = stcAcc.a[2]&0xFF;
  data_buffer[5] = (stcAcc.a[2]>>8)&0xFF;

  // 校验位的计算使用数据位各个数据相加 & 0xFF
  for (i = 0; i < AccLEN-1; i++)
    checknum += data_buffer[i];

  data_buffer[AccLEN-1] = checknum & 0xFF;
  UART1_Put_Char(0x55); // 帧头
  UART1_Put_Char(0x03); // 标识位
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

// 上报陀螺仪角速度
void Gyro_Send_Data(void)
{
  #define GyroLEN        7
  uint8_t data_buffer[GyroLEN] = {0};
  uint8_t i, checknum = 0;
  
  // 低字节在前，高字节在后
  // X轴角速度
  data_buffer[0] = stcGyro.w[0]&0xFF;
  data_buffer[1] = (stcGyro.w[0]>>8)&0xFF;
  // Y轴角速度
  data_buffer[2] = stcGyro.w[1]&0xFF;
  data_buffer[3] = (stcGyro.w[1]>>8)&0xFF;
  // Z轴角速度
  data_buffer[4] = stcGyro.w[2]&0xFF;
  data_buffer[5] = (stcGyro.w[2]>>8)&0xFF;

  // 校验位的计算使用数据位各个数据相加 & 0xFF
  for (i = 0; i < GyroLEN-1; i++)
    checknum += data_buffer[i];

  data_buffer[GyroLEN-1] = checknum & 0xFF;
  UART1_Put_Char(0x55); // 帧头
  UART1_Put_Char(0x04); // 标识位
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

// 上报陀螺仪欧拉角
void Angle_Send_Data(void)
{
  #define AngleLEN        7
  uint8_t data_buffer[AngleLEN] = {0};
  uint8_t i, checknum = 0;
  
  // 低字节在前，高字节在后
  // Roll
  data_buffer[0] = stcAngle.Angle[0]&0xFF;
  data_buffer[1] = (stcAngle.Angle[0]>>8)&0xFF;
  // Pitch
  data_buffer[2] = stcAngle.Angle[1]&0xFF;
  data_buffer[3] = (stcAngle.Angle[1]>>8)&0xFF;
  // Yaw
  data_buffer[4] = stcAngle.Angle[2]&0xFF;
  data_buffer[5] = (stcAngle.Angle[2]>>8)&0xFF;

  // 校验位的计算使用数据位各个数据相加 & 0xFF
  for (i = 0; i < AngleLEN-1; i++)
    checknum += data_buffer[i];

  data_buffer[AngleLEN-1] = checknum & 0xFF;
  UART1_Put_Char(0x55); // 帧头
  UART1_Put_Char(0x05); // 标识位
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
