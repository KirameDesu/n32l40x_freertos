#ifndef UART_H
#define UART_H

#include "n32l40x.h"


#define DMABufferSize1      1024
#define DMABufferSize2      1024


#define _USART1_USART2_
// #define _USART3_UART4_
// #define _UART4_UART5_

#ifdef _USART1_USART2_
#define USARTy                USART1
#define USARTy_GPIO           GPIOB
#define USARTy_CLK            RCC_APB2_PERIPH_USART1
#define USARTy_GPIO_CLK       RCC_APB2_PERIPH_GPIOB
#define USARTy_RxPin          GPIO_PIN_7
#define USARTy_TxPin          GPIO_PIN_6
#define USARTy_Rx_GPIO_AF     GPIO_AF0_USART1
#define USARTy_Tx_GPIO_AF     GPIO_AF0_USART1
#define USARTy_APBxClkCmd     RCC_EnableAPB2PeriphClk
#define USARTy_DAT_Base       (USART1_BASE + 0x04)
#define USARTy_Tx_DMA_Channel DMA_CH4
#define USARTy_Tx_DMA_FLAG    DMA_FLAG_TC4
#define USARTy_Rx_DMA_Channel DMA_CH5
#define USARTy_Rx_DMA_FLAG    DMA_FLAG_TC5
#define USARTy_Tx_DMA_REMAP   DMA_REMAP_USART1_TX
#define USARTy_Rx_DMA_REMAP   DMA_REMAP_USART1_RX
#define USARTy_IRQn           USART1_IRQn

#define USARTz                USART2
#define USARTz_GPIO           GPIOA
#define USARTz_CLK            RCC_APB1_PERIPH_USART2
#define USARTz_GPIO_CLK       RCC_APB2_PERIPH_GPIOA
#define USARTz_RxPin          GPIO_PIN_3
#define USARTz_TxPin          GPIO_PIN_2
#define USARTz_Rx_GPIO_AF     GPIO_AF4_USART2
#define USARTz_Tx_GPIO_AF     GPIO_AF4_USART2
#define USARTz_APBxClkCmd     RCC_EnableAPB1PeriphClk
#define USARTz_DAT_Base       (USART2_BASE + 0x04)
#define USARTz_Tx_DMA_Channel DMA_CH7
#define USARTz_Tx_DMA_FLAG    DMA_FLAG_TC7
#define USARTz_Rx_DMA_Channel DMA_CH6
#define USARTz_Rx_DMA_FLAG    DMA_FLAG_TC6
#define USARTz_Tx_DMA_REMAP   DMA_REMAP_USART2_TX
#define USARTz_Rx_DMA_REMAP   DMA_REMAP_USART2_RX
#define USARTz_IRQn           USART2_IRQn
#endif

#ifdef _USART3_UART4_
#define USARTy                USART3
#define USARTy_GPIO           GPIOB
#define USARTy_CLK            RCC_APB1_PERIPH_USART3
#define USARTy_GPIO_CLK       RCC_APB2_PERIPH_GPIOB
#define USARTy_RxPin          GPIO_PIN_11
#define USARTy_TxPin          GPIO_PIN_10
#define USARTy_Rx_GPIO_AF     GPIO_AF5_USART3
#define USARTy_Tx_GPIO_AF     GPIO_AF0_USART3
#define USARTy_APBxClkCmd     RCC_EnableAPB1PeriphClk
#define USARTy_DAT_Base       (USART3_BASE + 0x04)
#define USARTy_Tx_DMA_Channel DMA_CH1
#define USARTy_Tx_DMA_FLAG    DMA_FLAG_TC1
#define USARTy_Rx_DMA_Channel DMA_CH2
#define USARTy_Rx_DMA_FLAG    DMA_FLAG_TC2
#define USARTy_Tx_DMA_REMAP   DMA_REMAP_USART3_TX
#define USARTy_Rx_DMA_REMAP   DMA_REMAP_USART3_RX

#define USARTz                UART4
#define USARTz_GPIO           GPIOB
#define USARTz_CLK            RCC_APB2_PERIPH_UART4
#define USARTz_GPIO_CLK       RCC_APB2_PERIPH_GPIOB
#define USARTz_RxPin          GPIO_PIN_1
#define USARTz_TxPin          GPIO_PIN_0
#define USARTz_Rx_GPIO_AF     GPIO_AF6_UART4
#define USARTz_Tx_GPIO_AF     GPIO_AF6_UART4
#define USARTz_APBxClkCmd     RCC_EnableAPB2PeriphClk
#define USARTz_DAT_Base       (UART4_BASE + 0x04)
#define USARTz_Tx_DMA_Channel DMA_CH5
#define USARTz_Tx_DMA_FLAG    DMA_FLAG_TC5
#define USARTz_Rx_DMA_Channel DMA_CH4
#define USARTz_Rx_DMA_FLAG    DMA_FLAG_TC4
#define USARTz_Tx_DMA_REMAP   DMA_REMAP_UART4_TX
#define USARTz_Rx_DMA_REMAP   DMA_REMAP_UART4_RX
#endif

#ifdef _UART4_UART5_
#define USARTy                UART4
#define USARTy_GPIO           GPIOC
#define USARTy_CLK            RCC_APB2_PERIPH_UART4
#define USARTy_GPIO_CLK       RCC_APB2_PERIPH_GPIOC
#define USARTy_RxPin          GPIO_PIN_11
#define USARTy_TxPin          GPIO_PIN_10
#define USARTy_Rx_GPIO_AF     GPIO_AF6_UART4
#define USARTy_Tx_GPIO_AF     GPIO_AF6_UART4
#define USARTy_APBxClkCmd     RCC_EnableAPB2PeriphClk
#define USARTy_DAT_Base       (UART4_BASE + 0x04)
#define USARTy_Tx_DMA_Channel DMA_CH2
#define USARTy_Tx_DMA_FLAG    DMA_FLAG_TC2
#define USARTy_Rx_DMA_Channel DMA_CH1
#define USARTy_Rx_DMA_FLAG    DMA_FLAG_TC1
#define USARTy_Tx_DMA_REMAP   DMA_REMAP_UART4_TX
#define USARTy_Rx_DMA_REMAP   DMA_REMAP_UART4_RX

#define USARTz                UART5
#define USARTz_GPIO           GPIOB
#define USARTz_CLK            RCC_APB2_PERIPH_UART5
#define USARTz_GPIO_CLK       RCC_APB2_PERIPH_GPIOB
#define USARTz_RxPin          GPIO_PIN_5
#define USARTz_TxPin          GPIO_PIN_4
#define USARTz_Rx_GPIO_AF     GPIO_AF7_UART5
#define USARTz_Tx_GPIO_AF     GPIO_AF6_UART5
#define USARTz_APBxClkCmd     RCC_EnableAPB2PeriphClk
#define USARTz_DAT_Base       (UART5_BASE + 0x04)
#define USARTz_Tx_DMA_Channel DMA_CH7
#define USARTz_Tx_DMA_FLAG    DMA_FLAG_TC7
#define USARTz_Rx_DMA_Channel DMA_CH8
#define USARTz_Rx_DMA_FLAG    DMA_FLAG_TC8
#define USARTz_Tx_DMA_REMAP   DMA_REMAP_UART5_TX
#define USARTz_Rx_DMA_REMAP   DMA_REMAP_UART5_RX
#endif

// 消息队列
#define UART_MSG_MAX_LEN   DMABufferSize2
#define UART_QUEUE_LENGTH  3
typedef struct
{
    uint8_t data[20];
    uint16_t length;
} UartMsg_t;


void Uart_init(void);
void Uart_Send(USART_Module* USARTx, const char* pData, uint16_t size);
void vUsart2RxProcessTask(void *pv);
void Uart_WaitRecevComplete(USART_Module* USARTx);

#endif // UART_H
