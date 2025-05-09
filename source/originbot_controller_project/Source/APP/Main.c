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
#include "Main.h"
#include "UART1.h"
#include "UART3.h"
#include "delay.h"
#include "JY901.h"
#include "DIO.h"
#include "adc.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "misc.h"

void _system_init(void);
//Task priority    //�������ȼ�
#define START_TASK_PRIO	4
#define LED_TASK_PRIO 4
#define BUZZER_TASK_PRIO1 3
#define RECEIVE_HANDLE_TASK_PRIO 6

//Task stack size //�����ջ��С	
#define START_STK_SIZE 	128  
#define LED_STK_SIZE   64
#define BUZZER_STK_SIZE   64

//Task handle     //������
TaskHandle_t StartTask_Handler;

//Task function   //������
void start_task(void *pvParameters);
void led_task(void *pvParameters);
void buzzer_task(void *pvParameters);
void uart_task(void *pvParameters);
void receive_handle_task(void *pvParameters);

//Main function //������
int main(void)
{
	_system_init(); //Hardware initialization //Ӳ����ʼ��
//	Delay_Ms(1000);
	//Create the start task //������ʼ����
	printf("0");
	xTaskCreate((TaskFunction_t )start_task,            //Task function   //������
							(const char*    )"start_task",          //Task name       //��������
							(uint16_t       )START_STK_SIZE,        //Task stack size //�����ջ��С
							(void*          )NULL,                  //Arguments passed to the task function //���ݸ��������Ĳ���
							(UBaseType_t    )START_TASK_PRIO,       //Task priority   //�������ȼ�
							(TaskHandle_t*  )&StartTask_Handler);   //Task handle     //������   					
	vTaskStartScheduler();  //Enables task scheduling //�����������	
	printf("9");
}

//Start task task function //��ʼ����������
void start_task(void *pvParameters)
{
    taskENTER_CRITICAL(); //Enter the critical area //�����ٽ���
    //Create the task //��������
		xTaskCreate(led_task, "led_task", 1000, NULL, 2, NULL);
		xTaskCreate(buzzer_task, "buzzer_task", 1000, NULL, 3, NULL);
		xTaskCreate(uart_task, "uart_task2", 1000, NULL, 4, NULL);
    vTaskDelete(StartTask_Handler); //Delete the start task //ɾ����ʼ����
    taskEXIT_CRITICAL();            //Exit the critical section//�˳��ٽ���
}

void led_task(void *pvParameters)
{
	for(;;)
	{
		LED_ON();
		vTaskDelay(1000/portTICK_RATE_MS);
		LED_OFF();
		vTaskDelay(1000/portTICK_RATE_MS);
	}
}

void buzzer_task(void *pvParameters)
{	
	for(;;)
	{
    Bat_Update_Power_State();
    if (Bat_Is_Low_Power()) {
			BUZZER_ON();
		}
		vTaskDelay(100/portTICK_RATE_MS);
	}
}

static u32 sysTickCnt=0;
u32 getSysTickCnt(void)
{
	if(xTaskGetSchedulerState()!=taskSCHEDULER_NOT_STARTED)	//The system is running //ϵͳ�Ѿ�����
		return xTaskGetTickCount();
	else
		return sysTickCnt;
}
void uart_task(void *pvParameters)
{
	unsigned int lastWakeTime = getSysTickCnt();
	for(;;)
  {	
		if (Is_Recv_New_Cmd()) {
      Parse_Cmd_Data(Get_RxBuffer(), Get_CMD_Length());
      Clear_CMD_Flag();
    }
	//	vTaskDelayUntil(&lastWakeTime, 50/portTICK_RATE_MS);
		Motion_Send_Data(); 
		Acc_Send_Data();   
		Gyro_Send_Data();
		Angle_Send_Data();
		Sensor_Send_Data();
		vTaskDelay(100/portTICK_RATE_MS);
	}
}

void _system_init(void)
{
	SysTick_init(72, 10);
  UART3_Init(9600);
  UART1_Init(115200);
  jy901_init();
 // Delay_Ms(1000); Delay_Ms(1000);
  Adc_Init();
  GPIO_Config();
  MOTOR_GPIO_Init();
  Motor_PWM_Init(MOTOR_MAX_PULSE, MOTOR_FREQ_DIVIDE);
  Encoder_Init();
  TIM1_Init();
  PID_Init();
//	Delay_Ms(1000); 
}
