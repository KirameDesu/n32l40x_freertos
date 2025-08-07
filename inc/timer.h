/*
 * @Author: Kirame
 * @Date: 2025-08-07 17:51:46
 * @FilePath: \RTOS\inc\timer.h
 * @Descripttion: 
 */
#ifndef __TIMER_H__
#define __TIMER_H__

#include <stdint.h>

// 提供给 FreeRTOS 使用的接口
void vConfigureTimerForRunTimeStats(void);
uint32_t ulGetRunTimeCounterValue(void);

// 外部引用的计数变量
extern volatile uint32_t ulHighFrequencyTimerTicks;

#endif
