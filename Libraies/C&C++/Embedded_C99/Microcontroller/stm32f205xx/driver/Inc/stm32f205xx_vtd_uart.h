#ifndef __STM32F1xx_VTD_UART_H
#define __STM32F1xx_VTD_UART_H

#include "main.h"

#define DMA_USART1_TX            DMA2_Stream7
#define DMA_USART1_RX            DMA2_Stream5
#define DMA_USART2_RX            DMA1_Stream5
#define DMA_USART2_TX            DMA1_Stream6
#define DMA_USART3_TX            DMA1_Stream3
#define DMA_USART3_RX            DMA1_Stream1
#define DMA_UART4_RX             DMA1_Stream2
#define DMA_UART4_TX             DMA1_Stream4
#define DMA_UART5_TX             DMA1_Stream7
#define DMA_UART5_RX             DMA1_Stream0
#define DMA_USART6_RX            DMA2_Stream1
#define DMA_USART6_TX            DMA2_Stream6



#define BRate_9600_Clk8M    (52 << 4)|(1 << 0)
#define BRate_115200_Clk8M  (4 << 4)|(5 << 0)
#define BRate_9600_Clk16M   (104 << 4)|(3 << 0)
#define BRate_115200_Clk16M (8 << 4)|(11 << 0)


void USART1_Init(uint32_t Baudrate);
void USART2_Init(uint32_t Baudrate);
void USART3_Init(uint32_t Baudrate);
void UART4_Init(uint32_t Baudrate);
void UART5_Init(uint32_t Baudrate);
void USART6_Init(uint32_t Baudrate);

void UART_DMA_Init(volatile USART_TypeDef *USART, uint32_t Baudrate);
void DMA_Transfer(volatile DMA_Stream_TypeDef *DMAChannel, volatile void* MemPointer, volatile void* PeriPointer, unsigned long Length);
void DMA_Transfer1(volatile DMA_Stream_TypeDef *DMA_Stream, volatile void* MemPointer, volatile void* PeriPointer, unsigned long Length);
void UART_Transmit(volatile USART_TypeDef *USART,  unsigned char* Data, unsigned long Length);


#endif