#include "main.h"


void Config_Timer_PWM_WITH_DMA(){
    //	================== PWM - done ======================
	RCC->APB2ENR |= (1<<2)|(1<<3);
	RCC->APB2ENR |= (1<<11);
	GPIOA->CRH |= (0b1011<<0); // PA8 CH1
	GPIOB->CRH |=(0b1011<<20); // PB13 CH1N
	TIM1->CCMR1 = (0b110<<4)|(1<<3);	// PWM mode 1, bit 3 Y/N ok
	TIM1->PSC = 0;
	TIM1->ARR = 7999;
	TIM1->CCR1 = 4000;
	TIM1->CCER |= (1<<0)|(1<<2); //EN PWM Thuan, EN PWM Nghich;
	TIM1->BDTR |= (1<<11)|(1<<10); // OSSi, OSSR
	TIM1->BDTR |= (1<<4); // dead time
	TIM1->EGR |= (1<<0); // bit UG	
	TIM1->BDTR |= (1<<15); // bit MOE
	TIM1->CR1 |= (1<<8)|(1<<7)|(1<<2)|(1<<0);  // set T_clock of dead time, Reload ARR, xay ra su kien ngat gi, EN counter tim1 
	
    //========== RAM TO PWM WITH DMA - done ==============
	RCC->APB2ENR |= (1<<2)|(1<<3);
	RCC->APB2ENR |= (1<<11);
	GPIOA->CRH |= (0b1011<<0); // PA8 CH1
	GPIOB->CRH |=(0b1011<<20); // PB13 CH1N
	TIM1->CCMR1 = (0b110<<4)|(1<<3);	// PWM mode 1, bit 3 Y/N ok
	TIM1->PSC = 0;
	TIM1->ARR = 7999;
	TIM1->CCR1 = 1000;
	TIM1->CCER |= (1<<0)|(1<<2);
	TIM1->BDTR |= (1<<11)|(1<<10)|(1<<4); // OSSi, OSSR
	TIM1->EGR |= (1<<0); // bit UG	
	TIM1->BDTR |= (1<<15); // bit MOE
	TIM1->DIER |= (1<<8);
	TIM1->CR1 |= (1<<8)|(1<<7)|(1<<2);
	TIM1->CR1 |= (1<<0);
	RCC->AHBENR |= (1<<0);                            // EN clock DMA1
	DMA1_Channel5->CPAR = (unsigned long)&TIM1->CCR1;	
	DMA1_Channel5->CMAR = (unsigned long)data3;				
	DMA1_Channel5->CNDTR = 7;												// so lan chuyen 1 byte data
	DMA1_Channel5->CCR |= (1<<10)|(1<<8)|(1<<7)|(1<<5)|(1<<4);			// bit 12:8->truyen & nhan 16bit, EN auto truyen vong, EN mo rong dia chi R, UEN mo rong dia chi P, chieu du lieu tu R->P
	DMA1_Channel5->CCR |= (1<<0);	
	
}

// ================== PWM - done ======================
RCC->APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_IOPBEN | RCC_APB2ENR_TIM1EN;

// PA8 = TIM1_CH1, PB13 = TIM1_CH1N
GPIOA->CRH &= ~(0xF << 0);
GPIOA->CRH |=  (GPIO_CNF_AFPP << 2) | (GPIO_MODE_OUTPUT2MHz << 0);
GPIOB->CRH &= ~(0xF << 20);
GPIOB->CRH |=  (GPIO_CNF_AFPP << 22) | (GPIO_MODE_OUTPUT2MHz << 20);

TIM1->PSC = 0;
TIM1->ARR = 7999;
TIM1->CCR1 = 4000;

TIM1->CCMR1 = (TIM_CCMR1_OC1M_PWM1 << TIM_CCMR1_OC1M_Pos) | TIM_CCMR1_OC1PE;
TIM1->CCER |= TIM_CCER_CC1E | TIM_CCER_CC1NE;

TIM1->BDTR |= TIM_BDTR_OSSI | TIM_BDTR_OSSR | (1 << TIM_BDTR_DTG_Pos); // dead-time if needed
TIM1->EGR |= TIM_EGR_UG;
TIM1->BDTR |= TIM_BDTR_MOE;

TIM1->CR1 |= TIM_CR1_CKD_1 | TIM_CR1_ARPE | TIM_CR1_URS | TIM_CR1_CEN;

// ========== RAM TO PWM WITH DMA - done ==============
RCC->APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_IOPBEN | RCC_APB2ENR_TIM1EN;
GPIOA->CRH &= ~(0xF << 0);
GPIOA->CRH |=  (GPIO_CNF_AFPP << 2) | (GPIO_MODE_OUTPUT2MHz << 0);
GPIOB->CRH &= ~(0xF << 20);
GPIOB->CRH |=  (GPIO_CNF_AFPP << 22) | (GPIO_MODE_OUTPUT2MHz << 20);

TIM1->PSC = 0;
TIM1->ARR = 7999;
TIM1->CCR1 = 1000;

TIM1->CCMR1 = (TIM_CCMR1_OC1M_PWM1 << TIM_CCMR1_OC1M_Pos) | TIM_CCMR1_OC1PE;
TIM1->CCER |= TIM_CCER_CC1E | TIM_CCER_CC1NE;
TIM1->BDTR |= TIM_BDTR_OSSI | TIM_BDTR_OSSR | (1 << TIM_BDTR_DTG_Pos);
TIM1->EGR |= TIM_EGR_UG;
TIM1->BDTR |= TIM_BDTR_MOE;

TIM1->DIER |= TIM_DIER_UDE;
TIM1->CR1 |= TIM_CR1_CKD_1 | TIM_CR1_ARPE | TIM_CR1_URS | TIM_CR1_CEN;

RCC->AHBENR |= RCC_AHBENR_DMA1EN;

DMA1_Channel5->CPAR = (uint32_t)&TIM1->CCR1;
DMA1_Channel5->CMAR = (uint32_t)data3;
DMA1_Channel5->CNDTR = 7;
DMA1_Channel5->CCR |= DMA_CCR_MSIZE_0 | DMA_CCR_PSIZE_0 | DMA_CCR_MINC | DMA_CCR_CIRC | DMA_CCR_DIR;
DMA1_Channel5->CCR |= DMA_CCR_EN;


void TIM1_PWM_DMA_Init(uint16_t *buffer, uint16_t length, uint16_t arr, uint16_t psc) {
    // 1. Enable clocks
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_IOPBEN | RCC_APB2ENR_TIM1EN;
    RCC->AHBENR  |= RCC_AHBENR_DMA1EN;

    // 2. Configure PA8 (TIM1_CH1), PB13 (CH1N) as AF push-pull
    GPIOA->CRH &= ~(0xF << 0);
    GPIOA->CRH |=  (0xB << 0);  // PA8 = AF output push-pull, 50MHz

    GPIOB->CRH &= ~(0xF << 20);
    GPIOB->CRH |=  (0xB << 20); // PB13 = AF output push-pull, 50MHz

    // 3. Timer setup
    TIM1->PSC  = psc;
    TIM1->ARR  = arr;
    TIM1->CCR1 = buffer[0];

    TIM1->CCMR1 = (0b110 << 4) | (1 << 3); // PWM Mode 1, preload enable
    TIM1->CCER  = TIM_CCER_CC1E | TIM_CCER_CC1NE; // Enable CH1 and CH1N

    TIM1->BDTR  = TIM_BDTR_OSSI | TIM_BDTR_OSSR | TIM_BDTR_MOE; // Main output enable

    TIM1->DIER |= TIM_DIER_UDE; // Enable DMA on update event
    TIM1->EGR  |= TIM_EGR_UG;   // Force update

    // 4. DMA1_Channel5 setup (TIM1_CH1)
    DMA1_Channel5->CCR &= ~DMA_CCR_EN;
    DMA1_Channel5->CPAR  = (uint32_t)&TIM1->CCR1;
    DMA1_Channel5->CMAR  = (uint32_t)buffer;
    DMA1_Channel5->CNDTR = length;

    DMA1_Channel5->CCR = DMA_CCR_DIR | DMA_CCR_MINC | DMA_CCR_PSIZE_0 | DMA_CCR_MSIZE_0;
    // Optional: DMA_CCR_CIRC if repeat

    DMA1_Channel5->CCR |= DMA_CCR_EN;

    // 5. Start TIM1
    TIM1->CR1 |= TIM_CR1_CEN | TIM_CR1_ARPE;
}

