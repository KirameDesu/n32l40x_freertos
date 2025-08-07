#include "timer.h"

#include "n32l40x.h"

volatile uint32_t ulHighFrequencyTimerTicks = 0;

void vConfigureTimerForRunTimeStats(void)
{
    // 以 TIM2 为例，配置为 10kHz 定时
    RCC_EnableAPB1PeriphClk(RCC_APB1_PERIPH_TIM2, ENABLE);

    TIM_TimeBaseInitType TIM_TimeBaseStructure;
    TIM_TimeBaseStructure.Period = 999;             // 每 100us 中断一次（10KHz）
    TIM_TimeBaseStructure.Prescaler = (SystemCoreClock / 1000000) - 1;  // 1MHz 基准
    TIM_TimeBaseStructure.ClkDiv = 0;
    TIM_TimeBaseStructure.CntMode = TIM_CNT_MODE_UP;

    TIM_InitTimeBase(TIM2, &TIM_TimeBaseStructure);

    // 开启中断
    TIM_ConfigInt(TIM2, TIM_INT_UPDATE, ENABLE);

    NVIC_InitType NVIC_InitStructure;
    NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 5;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    TIM_Enable(TIM2, ENABLE);
}

// 运行时间计数器读取函数
uint32_t ulGetRunTimeCounterValue(void)
{
    return ulHighFrequencyTimerTicks;
}

// TIM2 中断处理函数
void TIM2_IRQHandler(void)
{
    if (TIM_GetIntStatus(TIM2, TIM_INT_UPDATE) != RESET)
    {
        ulHighFrequencyTimerTicks++;
        TIM_ClrIntPendingBit(TIM2, TIM_INT_UPDATE);
    }
}
