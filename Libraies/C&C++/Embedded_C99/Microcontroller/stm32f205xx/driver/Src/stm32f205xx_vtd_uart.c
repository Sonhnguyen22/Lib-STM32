#include "stm32f205xx_vtd_uart.h"


//USART_HandleTypeDef huart1;
//USART_HandleTypeDef huart2;
//DMA_HandleTypeDef hdma_uart_tx;
//DMA_HandleTypeDef hdma_uart_rx;



void USART1_Init(uint32_t Baudrate){
	// 1. Enable clocks
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;    									// GPIOA clock
	RCC->APB2ENR |= RCC_APB2ENR_USART1EN;											// USART1 clock
    									
	// 2. Configure PA9 (TX) and PA10 (RX) as AF7
	GPIOA->MODER &= ~((3 << (9 * 2)) | (3 << (10 * 2))); 			// Clear MODER0/1 
	GPIOA->MODER |=   (2 << (9 * 2)) | (2 << (10 * 2));   		// Alternate Function 
	GPIOA->OSPEEDR |= (3 << (9 * 2)) | (3 << (10 * 2));  			// High speed
	GPIOA->AFR[1] &= ~((0xF << (1 * 4)) | (0xF << (2 * 4))); 	// Clear AF
	GPIOA->AFR[1] |=  (7 << (1 * 4)) | (7 << (2 * 4));       	// AF7
	// 3. Enable UART: UE
	USART1->CR1 |= USART_CR1_UE;
	// 3. Calculate BRR 
	USART1->BRR |= Baudrate;
	// 4. Enable UART: TE (transmit), RE (receive)
	USART1->CR1 |= USART_CR1_TE | USART_CR1_RE;
} 
void USART2_Init(uint32_t Baudrate){
	// 1. Enable clocks
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;    									// GPIOA clock
	RCC->APB1ENR |= RCC_APB1ENR_USART2EN;											// USART1 clock
    									
	// 2. Configure PA2 (TX) and PA3 (RX) as AF7
	GPIOA->MODER &= ~((3 << (2 * 2)) | (3 << (3 * 2))); 			// Clear MODER0/1 
	GPIOA->MODER |=   (2 << (2 * 2)) | (2 << (3 * 2));   		  // Alternate Function 
	GPIOA->OSPEEDR |= (3 << (2 * 2)) | (3 << (3 * 2));  			// High speed
	GPIOA->AFR[0] &= ~((0xF << (2 * 4)) | (0xF << (3 * 4))); 	// Clear AF
	GPIOA->AFR[0] |=  (7 << (2 * 4)) | (7 << (3 * 4));       	// AF7
	// 3. Enable UART: UE
	USART2->CR1 |= USART_CR1_UE;
	// 3. Calculate BRR 
	USART2->BRR |= Baudrate;
	// 4. Enable UART: TE (transmit), RE (receive)
	USART2->CR1 |= USART_CR1_TE | USART_CR1_RE;
} 
void USART3_Init(uint32_t Baudrate){
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;    									// GPIOA clock
	RCC->APB1ENR |= RCC_APB1ENR_USART3EN;											// USART3 clock
    									
	// 2. Configure PB10 (TX) and PB11 (RX) USART3 as AF7
	GPIOB->MODER &= ~((3 << (10 * 2)) | (3 << (11 * 2))); 			// Clear MODER0/1 
	GPIOB->MODER |=   (2 << (10 * 2)) | (2 << (11 * 2));   		  // Alternate Function 
	GPIOB->OSPEEDR |= (3 << (10 * 2)) | (3 << (11 * 2));  			// High speed
	GPIOB->AFR[1] &= ~((0xF << (2 * 4)) | (0xF << (3 * 4))); 	// Clear AF
	GPIOB->AFR[1] |=  (7 << (2 * 4)) | (7 << (3 * 4));       	// AF7
	// 3. Enable UART: UE
	USART3->CR1 |= USART_CR1_UE;
	// 3. Calculate BRR 
	USART3->BRR |= Baudrate;
	// 4. Enable UART: TE (transmit), RE (receive)
	USART3->CR1 |= USART_CR1_TE | USART_CR1_RE;
} 
void UART4_Init(uint32_t Baudrate) {
	// 1. Enable clocks
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;    									// GPIOA clock
	RCC->APB1ENR |= RCC_APB1ENR_UART4EN;    									// UART4 clock
	// 2. Configure PA0 (TX) and PA1 (RX) as AF8
	GPIOA->MODER &= ~((3 << (0 * 2)) | (3 << (1 * 2))); 			// Clear MODER0/1 
	GPIOA->MODER |=   (2 << (0 * 2)) | (2 << (1 * 2));   			// Alternate Function 
	GPIOA->OSPEEDR |= (3 << (0 * 2)) | (3 << (1 * 2));  			// High speed
	GPIOA->AFR[0] &= ~((0xF << (0 * 4)) | (0xF << (1 * 4))); 	// Clear AF
	GPIOA->AFR[0] |=  (8 << (0 * 4)) | (8 << (1 * 4));       	// AF8
	// 3. Enable UART: UE
	UART4->CR1 |= USART_CR1_UE;
	// 3. Calculate BRR 
	UART4->BRR |= Baudrate;
	// 4. Enable UART: TE (transmit), RE (receive)
	UART4->CR1 |= USART_CR1_TE | USART_CR1_RE;
} 

 

void UART_DMA_Init(volatile USART_TypeDef *USART, uint32_t Baudrate) {
	RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN;
	
	USART->CR3 	 |= USART_CR3_DMAR
							 |  USART_CR3_DMAT;
	if(USART == USART1){ 
		USART1_Init(Baudrate);
		DMA_USART1_RX->CR |= DMA_SxCR_MINC       // Tu dong tang dia chi bo nho sau moi lan truyen                     
											|  DMA_SxCR_CIRC       // Hoat dong o che do vong lap (du lieu tu lap lai)
											| (4 << DMA_SxCR_CHSEL_Pos);
		DMA_USART1_TX->CR |= DMA_SxCR_MINC       // Tu dong tang dia chi bo nho sau moi lan truyen                     
											|  DMA_SxCR_DIR        // Huong truyen: tu bo nho den ngoai vi
											| (4 << DMA_SxCR_CHSEL_Pos);
	}                                          
	else if(USART == USART2){
		USART2_Init(Baudrate);
		USART->CR3 	 |= USART_CR3_DMAR
							   |  USART_CR3_DMAT;
		DMA_USART2_RX->CR |= DMA_SxCR_MINC       // Tu dong tang dia chi bo nho sau moi lan truyen                     
											|  DMA_SxCR_CIRC       // Hoat dong o che do vong lap (du lieu tu lap lai)
											| (4 << DMA_SxCR_CHSEL_Pos);
		DMA_USART2_TX->CR |= DMA_SxCR_MINC       // Tu dong tang dia chi bo nho sau moi lan truyen                     	
//											|  DMA_SxCR_CIRC
											|  DMA_SxCR_DIR        // Huong truyen: tu bo nho den ngoai vi
											| (4 << DMA_SxCR_CHSEL_Pos);
	}
	else if(USART == USART3){ 
		USART3_Init(Baudrate);
		DMA_USART3_RX->CR |= DMA_SxCR_MINC       // Tu dong tang dia chi bo nho sau moi lan truyen                     
											|  DMA_SxCR_CIRC       // Hoat dong o che do vong lap (du lieu tu lap lai)
											| (4 << DMA_SxCR_CHSEL_Pos);
		DMA_USART3_TX->CR |= DMA_SxCR_MINC       // Tu dong tang dia chi bo nho sau moi lan truyen                     
											|  DMA_SxCR_DIR        // Huong truyen: tu bo nho den ngoai vi
											| (4 << DMA_SxCR_CHSEL_Pos);
	}                                          
	else if(USART == UART4){ 
		UART4_Init(Baudrate);		
		DMA_UART4_RX->CR  |= DMA_SxCR_MINC       // Tu dong tang dia chi bo nho sau moi lan truyen                     
											|  DMA_SxCR_CIRC       // Hoat dong o che do vong lap (du lieu tu lap lai)
											| (4 << DMA_SxCR_CHSEL_Pos);
		DMA_UART4_TX->CR  |= DMA_SxCR_MINC       // Tu dong tang dia chi bo nho sau moi lan truyen                     	
											|  DMA_SxCR_DIR        // Huong truyen: tu bo nho den ngoai vi
											| (4 << DMA_SxCR_CHSEL_Pos);
	}

//	DMA_USART1_TX->CR |= DMA_SxCR_CHSEL      // Chon Channel 
//										|  DMA_SxCR_MBURST     // 
//										|  DMA_SxCR_PBURST		 //
//                    |  DMA_SxCR_PL_0       // Dat muc uu tien trung binh (01)
//                    |  DMA_SxCR_MSIZE_0    // Kich thuoc du lieu bo nho: 16-bit
//                    |  DMA_SxCR_PSIZE_0    // Kich thuoc du lieu ngoai vi: 16-bit
//                    |  DMA_SxCR_MINC       // Tu dong tang dia chi bo nho sau moi lan truyen
//                    |  DMA_SxCR_PINC       // Tu dong tang dia chi ngoai vi sau moi lan truyen
//                    |  DMA_SxCR_CIRC       // Hoat dong o che do vong lap (du lieu tu lap lai)
//                    |  DMA_SxCR_DIR        // Huong truyen: tu bo nho den ngoai vi
//                    |  DMA_SxCR_TEIE       // Cho phep ngat khi xay ra loi truyen
//                    |  DMA_SxCR_HTIE       // Cho phep ngat khi truyen duoc nua buffer
//                    |  DMA_SxCR_TCIE       // Cho phep ngat khi truyen xong
//                    |  DMA_SxCR_EN;        // Kich hoat DMA (bat dau hoat dong)
//	
}

void DMA_Transfer(volatile DMA_Stream_TypeDef *DMA_Stream, volatile void* MemPointer, volatile void* PeriPointer, unsigned long Length)
{
  DMA_Stream->CR  &= ~DMA_SxCR_EN;
  DMA_Stream->NDTR = Length;
  DMA_Stream->M0AR = (unsigned long)MemPointer;
  DMA_Stream->PAR  = (unsigned long)PeriPointer;
  DMA_Stream->CR  |= DMA_SxCR_EN;
}


void DMA_ClearFlags(DMA_Stream_TypeDef *stream) {
    if (stream == DMA1_Stream6) DMA1->HIFCR |= DMA_HIFCR_CTCIF6 | DMA_HIFCR_CTEIF6;
    else if (stream == DMA1_Stream7) DMA1->HIFCR |= DMA_HIFCR_CTCIF7 | DMA_HIFCR_CTEIF7;
    else if (stream == DMA1_Stream5) DMA1->HIFCR |= DMA_HIFCR_CTCIF5 | DMA_HIFCR_CTEIF5;
    else if (stream == DMA1_Stream2) DMA1->LIFCR |= DMA_LIFCR_CTCIF2 | DMA_LIFCR_CTEIF2;
    // Add other streams if needed
}

void UART_Transmit(volatile USART_TypeDef *USART, unsigned char* Data, unsigned long Length){
	for(uint8_t i = 0; i < Length; i++){
		USART->DR = (uint8_t)Data[i];
	while (!(USART->SR & USART_SR_TC));
	}
}

void DMA_Transfer1(volatile DMA_Stream_TypeDef *DMA_Stream, volatile void* MemPointer, volatile void* PeriPointer, unsigned long Length)
{
  DMA_Stream->CR &= ~DMA_SxCR_EN;
  while (DMA_Stream->CR & DMA_SxCR_EN);
	DMA_ClearFlags(DMA1_Stream6);
  // Clear all interrupt flags
  if (DMA_Stream == DMA1_Stream6)
    DMA1->HIFCR |= DMA_HIFCR_CTCIF6 | DMA_HIFCR_CTEIF6 | DMA_HIFCR_CDMEIF6 | DMA_HIFCR_CFEIF6;

  DMA_Stream->PAR  = (uint32_t)PeriPointer;
  DMA_Stream->M0AR = (uint32_t)MemPointer;
  DMA_Stream->NDTR = Length;

  // Assume CR, CHSEL, DIR, MINC... already set

  DMA_Stream->FCR = 0; // disable FIFO

  DMA_Stream->CR |= DMA_SxCR_EN;
}







