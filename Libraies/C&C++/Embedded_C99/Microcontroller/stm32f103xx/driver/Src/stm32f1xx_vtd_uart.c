#include "stm32f1xx_vtd_uart.h"


//USART_HandleTypeDef huart1;
//USART_HandleTypeDef huart2;
//DMA_HandleTypeDef hdma_uart_tx;
//DMA_HandleTypeDef hdma_uart_rx;


void UART_Init(volatile USART_TypeDef *USART, uint32_t Baudrate){
  GPIO_InitTypeDef GPIO_InitStruct = {0};
	
	if(USART == USART1){
		RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
		RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;
		
		GPIO_InitStruct.Pin = GPIO_PIN_9 | GPIO_PIN_10;
		GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;  
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
		HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	}
	else if(USART == USART2){
		RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
		RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;		
		
		GPIO_InitStruct.Pin = GPIO_PIN_2 | GPIO_PIN_3;
		GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;  
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
		HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	}
	USART->CR1  |= USART_CR1_UE;
	USART->BRR 	 = Baudrate;	
	USART->CR1 	|= USART_CR1_TE 
							|  USART_CR1_RE; 
} 

void UART_DMA_Init(volatile USART_TypeDef *USART, uint32_t Baudrate) {
	UART_Init(USART,Baudrate);	
	RCC->AHBENR |= RCC_AHBENR_DMA1EN;
	USART->CR3 	|= USART_CR3_DMAR
				|  USART_CR3_DMAT;
	if(USART == USART1){ 
		DMA_UART1_RX->CCR |= DMA_CCR_MINC       //7 Tu dong tang dia chi bo nho sau moi lan truyen                     
											|  DMA_CCR_CIRC;      //5 Hoat dong o che do vong lap (du lieu tu lap lai)
		DMA_UART1_TX->CCR |= DMA_CCR_MINC       // Tu dong tang dia chi bo nho sau moi lan truyen                     
											|  DMA_CCR_DIR;       //4 Huong truyen: tu bo nho den ngoai vi
	}
	else if(USART == USART2){
		DMA_UART2_RX->CCR |= DMA_CCR_MINC       //7 Tu dong tang dia chi bo nho sau moi lan truyen                     
											|  DMA_CCR_CIRC;      //5 Hoat dong o che do vong lap (du lieu tu lap lai)
		DMA_UART2_TX->CCR |= DMA_CCR_MINC       // Tu dong tang dia chi bo nho sau moi lan truyen                     
											|  DMA_CCR_DIR;       //4 Huong truyen: tu bo nho den ngoai vi	
	}

//	DMA_UART1_TX->CCR |= DMA_CCR_MEM2MEM    //14 Cho phep che do truyen tu bo nho sang bo nho
//                    |  DMA_CCR_PL_0       //12:13 Dat muc uu tien trung binh (01)
//                    |  DMA_CCR_MSIZE_0    //10:11 Kich thuoc du lieu bo nho: 16-bit
//                    |  DMA_CCR_PSIZE_0    //8:9 Kich thuoc du lieu ngoai vi: 16-bit
//                    |  DMA_CCR_MINC       //7 Tu dong tang dia chi bo nho sau moi lan truyen
//                    |  DMA_CCR_PINC       //6 Tu dong tang dia chi ngoai vi sau moi lan truyen
//                    |  DMA_CCR_CIRC       //5 Hoat dong o che do vong lap (du lieu tu lap lai)
//                    |  DMA_CCR_DIR        //4 Huong truyen: tu bo nho den ngoai vi
//                    |  DMA_CCR_TEIE       //3 Cho phep ngat khi xay ra loi truyen
//                    |  DMA_CCR_HTIE       //2 Cho phep ngat khi truyen duoc nua buffer
//                    |  DMA_CCR_TCIE       //1 Cho phep ngat khi truyen xong
//                    |  DMA_CCR_EN;        //0 Kich hoat DMA (bat dau hoat dong)


}

void DMA_Transfer(volatile DMA_Channel_TypeDef *DMAChannel, volatile void* MemPointer, volatile void* PeriPointer, unsigned long Length){
  DMAChannel->CCR &= ~(1<<0);
  DMAChannel->CNDTR = Length;
  DMAChannel->CMAR = (unsigned long)MemPointer;
  DMAChannel->CPAR = (unsigned long)PeriPointer;
  DMAChannel->CCR |= (1<<0);
}









