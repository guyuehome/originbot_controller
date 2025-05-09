/***********************************************************************
Copyright (c) 2022, www.guyuehome.com

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
***********************************************************************/

#ifndef __PROTOCOL_H__
#define __PROTOCOL_H__

#include "stm32f10x.h"

#define FUNC_MOTION               0x01
#define FUNC_BEEP_LED             0x07
#define FUNC_SET_LEFT_PID         0x08
#define FUNC_SET_RIGHT_PID        0x09

#define PTO_MAX_BUF_LEN           15

void Upper_Data_Receive(u8 data);
void Parse_Cmd_Data(u8 *data_buf, u8 num);

void Clear_CMD_Flag(void);
void Clear_RxBuffer(void);
u8* Get_RxBuffer(void);
u8 Get_CMD_Length(void);
u8 Is_Recv_New_Cmd(void);

#endif
