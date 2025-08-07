#include "uart.h"

#include <string.h>
#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"
#include "queue.h"

void RCC_Configuration(void);
void NVIC_Configuration(void);
void GPIO_Configuration(void);
void DMA_Configuration(void);

#define RINGBUFFER_ENABLE 0

// This file contains the implementation of UART initialization and DMA configuration
uint8_t TxBuffer1[DMABufferSize1];
uint8_t TxBuffer2[DMABufferSize2];
#if !RINGBUFFER_ENABLE
uint8_t RxBuffer1[DMABufferSize1];
uint8_t RxBuffer2[DMABufferSize2];
uint16_t RxCounter1 = 0;
uint16_t RxCounter2 = 0;
#else
// 环形缓冲区
RingBuffer_t RxRingBuffer1;
RingBuffer_t RxRingBuffer2;
#endif

// 串口接收信号量
SemaphoreHandle_t xUsart2IdleSemaphore;
// 消息队列
QueueHandle_t xUartRxQueue;

void Uart_init(void)
{
    USART_InitType USART_InitStructure;

    /* System Clocks Configuration */
    RCC_Configuration();

    /* NVIC configuration */
    NVIC_Configuration();

    /* Configure the GPIO ports */
    GPIO_Configuration();

    /* Configure the DMA */
    DMA_Configuration();

    /* USARTy and USARTz configuration ------------------------------------------------------*/
    USART_StructInit(&USART_InitStructure);
    USART_InitStructure.BaudRate            = 115200;
    USART_InitStructure.WordLength          = USART_WL_8B;
    USART_InitStructure.StopBits            = USART_STPB_1;
    USART_InitStructure.Parity              = USART_PE_NO;
    USART_InitStructure.HardwareFlowControl = USART_HFCTRL_NONE;
    USART_InitStructure.Mode                = USART_MODE_RX | USART_MODE_TX;

    /* Configure USARTy and USARTz */
    USART_Init(USARTy, &USART_InitStructure);
    USART_Init(USARTz, &USART_InitStructure);

    // 配置空闲中断
    USART_ConfigInt(USARTy, USART_INT_IDLEF, ENABLE);
    USART_ConfigInt(USARTz, USART_INT_IDLEF, ENABLE);

    /* Enable USARTy DMA Rx and TX request */
    USART_EnableDMA(USARTy, USART_DMAREQ_RX | USART_DMAREQ_TX, ENABLE);
    /* Enable USARTz DMA Rx and TX request */
    USART_EnableDMA(USARTz, USART_DMAREQ_RX | USART_DMAREQ_TX, ENABLE);

    /* Enable USARTy TX DMA Channel */
    //DMA_EnableChannel(USARTy_Tx_DMA_Channel, ENABLE);
    /* Enable USARTy RX DMA Channel */
    DMA_EnableChannel(USARTy_Rx_DMA_Channel, ENABLE);

    /* Enable USARTz TX DMA Channel */
    //DMA_EnableChannel(USARTz_Tx_DMA_Channel, ENABLE);
    /* Enable USARTz RX DMA Channel */
    DMA_EnableChannel(USARTz_Rx_DMA_Channel, ENABLE);

    /* Enable the USARTy and USARTz */
    USART_Enable(USARTy, ENABLE);
    USART_Enable(USARTz, ENABLE);

    // 初始化环形缓冲区
    #if RINGBUFFER_ENABLE
    RingBuffer_Init(&RxRingBuffer1);
    RingBuffer_Init(&RxRingBuffer2);
    #endif

    // 初始化信号量
    xUsart2IdleSemaphore = xSemaphoreCreateBinary();
    // 初始化消息队列
    xUartRxQueue = xQueueCreate(UART_QUEUE_LENGTH, sizeof(UartMsg_t));
    configASSERT(xUartRxQueue != NULL);  // 创建成功检查

}

/**
 * @brief  Configures the different system clocks.
 */
void RCC_Configuration(void)
{
    /* DMA clock enable */
    RCC_EnableAHBPeriphClk(RCC_AHB_PERIPH_DMA, ENABLE);
    /* Enable GPIO clock */
    RCC_EnableAPB2PeriphClk(USARTy_GPIO_CLK | USARTz_GPIO_CLK, ENABLE);
    /* Enable USARTy and USARTz Clock */
    USARTy_APBxClkCmd(USARTy_CLK, ENABLE);
    USARTz_APBxClkCmd(USARTz_CLK, ENABLE);
}

/**
 * @brief  Configures the nested vectored interrupt controller.
 */
void NVIC_Configuration(void)
{
    NVIC_InitType NVIC_InitStructure;

    /* Enable the USARTy Interrupt */
    NVIC_InitStructure.NVIC_IRQChannel            = USARTy_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 5;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd         = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    /* Enable the USARTz Interrupt */
    NVIC_InitStructure.NVIC_IRQChannel            = USARTz_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 5;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd         = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

/**
 * @brief  Configures the different GPIO ports.
 */
void GPIO_Configuration(void)
{
    GPIO_InitType GPIO_InitStructure;

    /* Initialize GPIO_InitStructure */
    GPIO_InitStruct(&GPIO_InitStructure);

    /* Configure USARTy Tx as alternate function push-pull */
    GPIO_InitStructure.Pin            = USARTy_TxPin;  
    GPIO_InitStructure.GPIO_Pull      = GPIO_Pull_Up;	
    GPIO_InitStructure.GPIO_Mode      = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Alternate = USARTy_Tx_GPIO_AF;
    GPIO_InitPeripheral(USARTy_GPIO, &GPIO_InitStructure);

    /* Configure USARTz Tx as alternate function push-pull */
    GPIO_InitStructure.Pin            = USARTz_TxPin;
    GPIO_InitStructure.GPIO_Alternate = USARTz_Tx_GPIO_AF;
    GPIO_InitPeripheral(USARTz_GPIO, &GPIO_InitStructure);

    /* Configure USARTy Rx as alternate function push-pull and pull-up */
    GPIO_InitStructure.Pin            = USARTy_RxPin;
    GPIO_InitStructure.GPIO_Pull      = GPIO_Pull_Up;
    GPIO_InitStructure.GPIO_Alternate = USARTy_Rx_GPIO_AF;
    GPIO_InitPeripheral(USARTy_GPIO, &GPIO_InitStructure);    

    /* Configure USARTz Rx as alternate function push-pull and pull-up */
    GPIO_InitStructure.Pin            = USARTz_RxPin;
    GPIO_InitStructure.GPIO_Alternate = USARTz_Rx_GPIO_AF;
    GPIO_InitPeripheral(USARTz_GPIO, &GPIO_InitStructure);      
}

/**
 * @brief  Configures the DMA.
 */
void DMA_Configuration(void)
{
    DMA_InitType DMA_InitStructure;

    /* USARTy TX DMA1 Channel (triggered by USARTy Tx event) Config */
    DMA_DeInit(USARTy_Tx_DMA_Channel);
    DMA_StructInit(&DMA_InitStructure);
    DMA_InitStructure.PeriphAddr     = USARTy_DAT_Base;
    DMA_InitStructure.MemAddr        = (uint32_t)TxBuffer1;
    DMA_InitStructure.Direction      = DMA_DIR_PERIPH_DST;
    DMA_InitStructure.BufSize        = DMABufferSize1;
    DMA_InitStructure.PeriphInc      = DMA_PERIPH_INC_DISABLE;
    DMA_InitStructure.DMA_MemoryInc  = DMA_MEM_INC_ENABLE;
    DMA_InitStructure.PeriphDataSize = DMA_PERIPH_DATA_SIZE_BYTE;
    DMA_InitStructure.MemDataSize    = DMA_MemoryDataSize_Byte;
    DMA_InitStructure.CircularMode   = DMA_MODE_NORMAL;
    DMA_InitStructure.Priority       = DMA_PRIORITY_VERY_HIGH;
    DMA_InitStructure.Mem2Mem        = DMA_M2M_DISABLE;
    DMA_Init(USARTy_Tx_DMA_Channel, &DMA_InitStructure);
    DMA_RequestRemap(USARTy_Tx_DMA_REMAP, DMA, USARTy_Tx_DMA_Channel, ENABLE);

    /* USARTy RX DMA1 Channel (triggered by USARTy Rx event) Config */
    DMA_DeInit(USARTy_Rx_DMA_Channel);
    DMA_InitStructure.PeriphAddr = USARTy_DAT_Base;
    #if RINGBUFFER_ENABLE
    DMA_InitStructure.MemAddr    = (uint32_t)RxRingBuffer1.buffer; // 使用环形缓冲区
    #else
    DMA_InitStructure.MemAddr    = (uint32_t)RxBuffer1;
    #endif
    DMA_InitStructure.Direction  = DMA_DIR_PERIPH_SRC;
    DMA_InitStructure.BufSize    = DMABufferSize2;
    DMA_Init(USARTy_Rx_DMA_Channel, &DMA_InitStructure);
    DMA_RequestRemap(USARTy_Rx_DMA_REMAP, DMA, USARTy_Rx_DMA_Channel, ENABLE);

    /* USARTz TX DMA1 Channel (triggered by USARTz Tx event) Config */
    DMA_DeInit(USARTz_Tx_DMA_Channel);
    DMA_InitStructure.PeriphAddr = USARTz_DAT_Base;
    DMA_InitStructure.MemAddr    = (uint32_t)TxBuffer2;
    DMA_InitStructure.Direction  = DMA_DIR_PERIPH_DST;
    DMA_InitStructure.BufSize    = DMABufferSize2;
    DMA_Init(USARTz_Tx_DMA_Channel, &DMA_InitStructure);
    DMA_RequestRemap(USARTz_Tx_DMA_REMAP, DMA, USARTz_Tx_DMA_Channel, ENABLE);

    /* USARTz RX DMA1 Channel (triggered by USARTz Rx event) Config */
    DMA_DeInit(USARTz_Rx_DMA_Channel);
    DMA_InitStructure.PeriphAddr = USARTz_DAT_Base;
    #if RINGBUFFER_ENABLE
    DMA_InitStructure.MemAddr    = (uint32_t)RxRingBuffer2.buffer;
    #else
    DMA_InitStructure.MemAddr    = (uint32_t)RxBuffer2;
    #endif
    DMA_InitStructure.Direction  = DMA_DIR_PERIPH_SRC;
    DMA_InitStructure.BufSize    = DMABufferSize1;
    DMA_Init(USARTz_Rx_DMA_Channel, &DMA_InitStructure);
    DMA_RequestRemap(USARTz_Rx_DMA_REMAP, DMA, USARTz_Rx_DMA_Channel, ENABLE);
}

void Uart_Send(USART_Module* USARTx, const char* pData, uint16_t size)
{
    DMA_ChannelType*    DMA_CH_TX = NULL;
    uint32_t            DMA_FLAG_TX_TC = 0;
    uint8_t*            TxBuffer = NULL;
    //uint16_t            TxBuffLen;

    if (USARTx == NULL || pData == NULL || size == 0)
        return;

    if (USARTx == USARTy)
    {
        DMA_CH_TX = USARTy_Tx_DMA_Channel;
        DMA_FLAG_TX_TC = USARTy_Tx_DMA_FLAG;
        TxBuffer = TxBuffer1;
        //TxBuffLen = DMABufferSize1;
    }
    else if (USARTx == USARTz)
    {
        DMA_CH_TX = USARTz_Tx_DMA_Channel;
        DMA_FLAG_TX_TC = USARTz_Tx_DMA_FLAG;
        TxBuffer = TxBuffer2;
        //TxBuffLen = DMABufferSize2;
    }
    else
    {
        return; // 无效的 USARTx
    }

    // 停止之前的 DMA 发送（如果有）
    DMA_EnableChannel(DMA_CH_TX, DISABLE);  // DMA_CH_TX 是你配置的 TX 通道
    DMA_ClearFlag(DMA_FLAG_TX_TC, DMA);         // 清除传输完成标志
    memcpy(TxBuffer, pData, size); // 将数据复制到发送缓冲区
    DMA_SetCurrDataCounter(DMA_CH_TX, size);
    // 启动 DMA 传输
    DMA_EnableChannel(DMA_CH_TX, ENABLE);
}

void Uart_WaitRecevComplete(USART_Module* USARTx)
{
    UartMsg_t* xReceivedData = NULL;
    while (1)
    {
        // 从消息队列中取数据
        if (xQueueReceive(xUartRxQueue, &xReceivedData, portMAX_DELAY) == pdTRUE)
        {
            // 处理接收到的数据
            // 这里可以根据实际需求进行数据处理
            // 例如打印接收到的数据
            if (xReceivedData)
            {
                Uart_Send(USARTy, (const char*)xReceivedData->data, xReceivedData->length);
            }
        }
    }
}

static UartMsg_t msg;
void vUsart2RxProcessTask(void *pv)
{
    UartMsg_t* pmsg = &msg;
    for (;;)
    {
        // 等待串口空闲中断通知
        if (xSemaphoreTake(xUsart2IdleSemaphore, portMAX_DELAY) == pdTRUE)
        {
            // 计算接收长度
            RxCounter2 = DMABufferSize2 - DMA_GetCurrDataCounter(USARTz_Rx_DMA_Channel);

            // 发送队列消息
            if (RxCounter2 > 0)
            {
                if (RxCounter2 > 20)
                    RxCounter2 = 20;
                memcpy(pmsg->data, RxBuffer2, RxCounter2);
                pmsg->length = RxCounter2;
                
                xQueueSend(xUartRxQueue, &pmsg, portMAX_DELAY);
            }

            // 启动DMA发送
            //DMA_CTL_Send(pUsartUserPara->p_dma_tx_ch, pUsartUserPara->rx_buff_pos);

            // 重启DMA接收
            DMA_SetCurrDataCounter(USARTz_Rx_DMA_Channel, DMABufferSize2);
            DMA_EnableChannel(USARTz_Rx_DMA_Channel, ENABLE);
        }
    }
}

void USART1_IRQHandler(void)
{
    //BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    if (USART_GetIntStatus(USART1, USART_INT_IDLEF) != RESET)
    {
        volatile uint32_t tmp;
        tmp = USART1->STS;    // 读SR
        tmp = USART1->DAT;    // 读DR，清除IDLE标志

        //DMA_EnableChannel(USARTz_Rx_DMA_Channel, DISABLE);

        // 通知处理任务（这里用信号量）
        //xSemaphoreGiveFromISR(xUsart2IdleSemaphore, &xHigherPriorityTaskWoken);

        // 如果有高优先级任务被唤醒，立即切换
        //portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

void USART2_IRQHandler(void)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    if (USART_GetIntStatus(USART2, USART_INT_IDLEF) != RESET)
    {
        volatile uint32_t tmp;
        tmp = USART2->STS;    // 读SR
        tmp = USART2->DAT;    // 读DR，清除IDLE标志

        DMA_EnableChannel(USARTz_Rx_DMA_Channel, DISABLE);

        // 通知处理任务（这里用信号量）暂时屏蔽
        //xSemaphoreGiveFromISR(xUsart2IdleSemaphore, &xHigherPriorityTaskWoken);

        // 如果有高优先级任务被唤醒，立即切换
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}
