#ifndef __STM32F1xx_VTD_UART_H
#define __STM32F1xx_VTD_UART_H

#include "main.h"

#define DMA_UART2_TX            DMA1_Channel7
#define DMA_UART2_RX            DMA1_Channel6
#define DMA_UART1_RX            DMA1_Channel5
#define DMA_UART1_TX            DMA1_Channel4

#define BRate_9600   (52 << 4)|(1 << 0)
#define BRate_115200 (4 << 4)|(5 << 0)

void UART_Init(volatile USART_TypeDef *USART, uint32_t Baudrate);
void UART_DMA_Init(volatile USART_TypeDef *USART, uint32_t Baudrate);
void DMA_Transfer(volatile DMA_Channel_TypeDef *DMAChannel, volatile void* MemPointer, volatile void* PeriPointer, unsigned long Length);



#endif