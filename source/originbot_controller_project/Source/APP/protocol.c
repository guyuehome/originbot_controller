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
#include "protocol.h"
#include "Main.h"
#include "UART1.h"
#include "DIO.h"
#include "JY901.h"

// 命令接收缓存
u8 RxBuffer[PTO_MAX_BUF_LEN];
// 接收数据下标
u8 RxIndex = 0;
// 接收状态机
u8 RxFlag = 0;
// 新命令接收标志
u8 New_CMD_flag;
// 新命令数据长度
u8 New_CMD_length;

// 获取接收的数据
u8* Get_RxBuffer(void)
{
  return (u8*)RxBuffer;
}

// 获取命令长度
u8 Get_CMD_Length(void)
{
  return New_CMD_length;
}

// 获取命令标志
u8 Is_Recv_New_Cmd(void)
{
  return New_CMD_flag;
}

// 清除命令数据和相关标志
void Clear_CMD_Flag(void)
{
  #if ENABLE_CLEAR_RXBUF
  for (u8 i = 0; i < New_CMD_length; i++)
    RxBuffer[i] = 0;
  #endif
  New_CMD_length = 0;
  New_CMD_flag = 0;
}

// RxBuffer置0
void Clear_RxBuffer(void)
{
  for (u8 i = 0; i < PTO_MAX_BUF_LEN; i++)
    RxBuffer[i] = 0;
}

// 指令解析，传入接收到的完整指令，及其长度
void Parse_Cmd_Data(u8 *data_buf, u8 num)
{
  #if ENABLE_CHECKSUM
  // 计算校验
  int sum = 0;
  for (u8 i = 3; i < (num - 2); i++)
    sum += *(data_buf + i);
  sum = sum & 0xFF;

  u8 recvSum = *(data_buf + num - 2);
  if (!(sum == recvSum))
    return;
  #endif

  // 判断帧头
  if (!(*(data_buf) == 0x55))
    return;

  u8 func_id = *(data_buf + 1);
  switch (func_id) {
  // 判断功能字：速度控制
  case FUNC_MOTION:
  {
    u8 index_l = *(data_buf + 3);
    u16 left = *(data_buf + 5);
    left = (left << 8) | (*(data_buf + 4));

    u8 index_r = *(data_buf + 6);
    u16 right = *(data_buf + 8);
    right = (right << 8) | (*(data_buf + 7));

    Motion_Test_SpeedSet(index_l, left, index_r, right);
    break;
  }
  
  // 判断功能字：LED、蜂鸣器状态、imu校准
  case FUNC_BEEP_LED:
  {
    u8 led_ctrl_en = *(data_buf + 3); // 使能控制字段
    u8 led = *(data_buf + 4);         // 状态字段
    if (led_ctrl_en) {
      if (led)
        LED_ON();
      else
        LED_OFF();
    }

    u8 buzzer_ctrl_en = *(data_buf + 5); // 使能控制字段
    u8 buzzer = *(data_buf + 6);         // 状态字段
    if (buzzer_ctrl_en) {
      if (buzzer)
        BUZZER_ON();
      else
        BUZZER_OFF();
    }

    u8 calibration_ctrl_en = *(data_buf + 7); // 使能控制字段
    u8 calibration = *(data_buf + 8);         // 状态字段
    if (calibration_ctrl_en && calibration)
		{
			#if ENABLE_ICM42670P
			
			#else
			jy901_calibration();
			#endif
		}

    break;
  }
  
  // 判断功能字：左轮PID参数
  case FUNC_SET_LEFT_PID:
  {
    u16 kp_recv = *(data_buf + 4);
    kp_recv = (kp_recv << 8) | *(data_buf + 3);

    u16 ki_recv = *(data_buf + 6);
    ki_recv = (ki_recv << 8) | *(data_buf + 5);

    u16 kd_recv = *(data_buf + 8);
    kd_recv = (kd_recv << 8) | *(data_buf + 7);

    float kp = (float)kp_recv / 1000.0;
    float ki = (float)ki_recv / 1000.0;
    float kd = (float)kd_recv / 1000.0;
    
    Left_Pid_Update_Value(kp, ki, kd);
    break;
  }
	
	// 判断功能字：右轮PID参数
  case FUNC_SET_RIGHT_PID:
  {
    u16 kp_recv = *(data_buf + 4);
    kp_recv = (kp_recv << 8) | *(data_buf + 3);

    u16 ki_recv = *(data_buf + 6);
    ki_recv = (ki_recv << 8) | *(data_buf + 5);

    u16 kd_recv = *(data_buf + 8);
    kd_recv = (kd_recv << 8) | *(data_buf + 7);

    float kp = (float)kp_recv / 1000.0;
    float ki = (float)ki_recv / 1000.0;
    float kd = (float)kd_recv / 1000.0;
    
    Right_Pid_Update_Value(kp, ki, kd);
    break;
  }

  default:
    break;
  }
}

// 接收串口单字节数据接收并保存
void Upper_Data_Receive(u8 Rx_Temp)
{
  switch (RxFlag) {
  // 帧头
  case 0:
  {
    if (Rx_Temp == 0x55) {
      RxBuffer[0] = 0x55;
      RxFlag = 1;
    } else {
      RxFlag = 0;
      RxBuffer[0] = 0x0;
    }
    break;
  }

  // 标识位
  case 1:
  {
    if (Rx_Temp == 0x01 || Rx_Temp == 0x07 || Rx_Temp == 0x08) {
      RxBuffer[1] = Rx_Temp;
      RxFlag = 2;
      RxIndex = 2;
    } else {
      RxFlag = 0;
      RxBuffer[0] = 0;
      RxBuffer[1] = 0;
    }
    break;
  }

  // 数据位长度
  case 2:
  {
    // New_CMD_length为数据帧总字节数 = 帧头+标识位+长度+校验位+帧尾(5 bytes)+数据位
    New_CMD_length = Rx_Temp+5;
    if (New_CMD_length >= PTO_MAX_BUF_LEN) {
      RxIndex = 0;
      RxFlag = 0;
      RxBuffer[0] = 0;
      RxBuffer[1] = 0;
      New_CMD_length = 0;
      break;
    }
    RxBuffer[RxIndex] = Rx_Temp;
    RxIndex++;
    RxFlag = 3;
    break;
  }

  // 读取完剩余的所有字段
  case 3:
  {
    RxBuffer[RxIndex] = Rx_Temp;
    RxIndex++;
    if (RxIndex >= New_CMD_length && RxBuffer[New_CMD_length-1] == 0xBB) {
      New_CMD_flag = 1;
      RxIndex = 0;
      RxFlag = 0;
    }
		else if(RxIndex >= New_CMD_length)
		{
			memset(RxBuffer,0,sizeof(RxBuffer));//防止键盘短按控制死机
			RxIndex = 0;
      RxFlag = 0;
		}
    break;
  }

  default:
    break;
  }
}

