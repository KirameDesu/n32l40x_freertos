/*****************************************************************************
 * Copyright (c) 2022, Nations Technologies Inc.
 *
 * All rights reserved.
 * ****************************************************************************
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * - Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the disclaimer below.
 *
 * Nations' name may not be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * DISCLAIMER: THIS SOFTWARE IS PROVIDED BY NATIONS "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
 * DISCLAIMED. IN NO EVENT SHALL NATIONS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * ****************************************************************************/

/**
 * @file main.c
 * @author Nations
 * @version V1.2.2
 *
 * @copyright Copyright (c) 2022, Nations Technologies Inc. All rights reserved.
 */
#include "main.h"

#include <string.h>
#include <stdio.h>

#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"

#include "led.h"
#include "uart.h"

TaskHandle_t start_led_task_handle;
TaskHandle_t led1_task_handle = NULL;
TaskHandle_t led2_task_handle = NULL;

TaskHandle_t start_uart_task_handle;
TaskHandle_t start_uart1_task_handle = NULL;
TaskHandle_t start_uart2_task_handle = NULL;

TaskHandle_t cpu_monitor_task_handle = NULL;

//======================================================================
//                          LED Task
//======================================================================
void led1_task(void *pv)
{
	while (1)
	{
        LedBlink(PORT_GROUP2, LED2_PIN);
		vTaskDelay(pdMS_TO_TICKS(1000));
	}
}

void led2_task(void *pv)
{
	while (1)
	{
        LedBlink(PORT_GROUP2, LED3_PIN);
		vTaskDelay(pdMS_TO_TICKS(500));
	}
}

void start_led_task(void *pv)
{
    /* Initialize Led1~Led5 as output pushpull mode*/
    LedInit(PORT_GROUP1, LED1_PIN);
    LedInit(PORT_GROUP2, LED2_PIN | LED3_PIN);
    /*Turn on Led1*/
    LedOff(PORT_GROUP1, LED1_PIN);

	xTaskCreate(led1_task,"led1",50,NULL,tskIDLE_PRIORITY + 2,&led1_task_handle);
	xTaskCreate(led2_task,"led2",50,NULL,tskIDLE_PRIORITY + 2,&led2_task_handle);

	vTaskDelete(start_led_task_handle);
}


//======================================================================
//                          Uart Task
//======================================================================
void uart1_task(void *pv)
{
    Uart_WaitRecevComplete(USARTz);
}

void uart2_task(void *pv)
{
	while (1)
	{
        //Uart_Send(USARTz, "This is USARTz task.\r\n", 22);
		vTaskDelay(pdMS_TO_TICKS(2000));
	}
}

void start_uart_task(void *pv)
{
    /* Initialize Led1~Led5 as output pushpull mode*/
    Uart_init();
    
    xTaskCreate(uart1_task,"uart1",50,NULL,tskIDLE_PRIORITY + 2,&led1_task_handle);
	xTaskCreate(uart2_task,"uart2",50,NULL,tskIDLE_PRIORITY + 2,&led2_task_handle);
    // 接收任务
    xTaskCreate(vUsart2RxProcessTask, "uart2_rx", 512, NULL, tskIDLE_PRIORITY + 3, NULL);

	vTaskDelete(start_uart_task_handle);
}


//======================================================================
//                          CPU Monitor Task
//======================================================================
void cpu_monitor_task(void *pv)
{
    char taskStats[512];
    size_t offset = 0;
    const size_t bufSize = sizeof(taskStats);

    while (1)
    {
        offset = snprintf(taskStats, bufSize, "Task Monitor:\n");
        // 判断是否还有空间写入任务统计信息
        if (offset < bufSize)
        {
            vTaskGetRunTimeStats(taskStats + offset);

            // 可选：再次检查长度，如果超出，截断末尾添加提醒
            if (strlen(taskStats) >= bufSize - 1)
            {
                strcpy(taskStats + bufSize - 20, "\n[Truncated...]\n");
            }
        }
        else
        {
            // 缓冲区太小，直接输出错误信息
            strcpy(taskStats, "Buffer overflow: header too large.\n");
        }

        Uart_Send(USARTy, taskStats, strlen(taskStats));

        vTaskDelay(pdMS_TO_TICKS(5000)); // 每 5 秒打印一次
    }
}

void start_cpu_monitor_task(void *pvParameters)
{
    xTaskCreate(cpu_monitor_task, "cpu_monitor", 512, NULL, tskIDLE_PRIORITY + 1, &cpu_monitor_task_handle);

    vTaskDelete(NULL);
}

/**
 * @brief  Main program.
 */
int main(void)
{
    BaseType_t xReturn[3];
    int i;
    
    // 配置优先级分组
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);

    // LED TASK
    xReturn[0] = xTaskCreate(start_led_task,
                        "startLed",
                        64,
                        NULL,
                        tskIDLE_PRIORITY + 1,
                        &start_led_task_handle);
    // UART TASK
    xReturn[1] = xTaskCreate(start_uart_task,
                        "startUart",
                        64,
                        NULL,
                        tskIDLE_PRIORITY + 1,
                        &start_uart_task_handle);

#if ENABLE_CPU_MONITOR
    xReturn[2] = xTaskCreate(start_cpu_monitor_task,
                        "startCPUMonitor",
                        64,
                        NULL,
                        tskIDLE_PRIORITY + 1,
                        NULL);
#endif

    for(i=0; i<3; i++)
    {
        if(pdPASS != xReturn[i])
            return -1;
    }
    vTaskStartScheduler();   /* 启动任务，开启调度 */

    while(1);   /* 正常不会执行到这里 */
}


// 堆空间不足时会自动调用此函数
void vApplicationMallocFailedHook(void)
{
    volatile int i;
    // 可以打断点调试，或者重启系统等
    taskDISABLE_INTERRUPTS();
    while (1)
    {
        LedBlink(PORT_GROUP1, LED1_PIN);
		for (i = 0; i < 500000; ++i);  // 粗略延时
    }
}

// 栈空间不足时会自动调用此函数
void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
    volatile int i;
    // 栈溢出时调用，可以放断点调试
    taskDISABLE_INTERRUPTS();
    while (1)
    {
        LedBlink(PORT_GROUP1, LED1_PIN);
		for (i = 0; i < 500000; ++i);  // 粗略延时
    }
}
