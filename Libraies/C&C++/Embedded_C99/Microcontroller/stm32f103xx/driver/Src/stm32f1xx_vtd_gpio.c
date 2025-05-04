#include "stm32f1xx_vtd_gpio.h"

uint8_t GPIO_ReadPin(GPIO_TypeDef *GPIOx, uint16_t pin_number) {
    return (GPIOx->IDR & (1 << pin_number)) ? 1 : 0;
}

void GPIO_WritePin(GPIO_TypeDef *GPIOx, uint16_t pin_number, uint8_t value) {
    if (value) {
        GPIOx->BSRR = (1 << pin_number);         // Set HIGH
    } else {
        GPIOx->BSRR = (1 << (pin_number + 16));  // Set LOW
    }
}

//============ Ram to port GPIOA WITH DMA - done=====  
	RCC->AHBENR |= (1<<0);  
	RCC->APB2ENR |= (1<<2);  
	GPIOA->CRL |= (0b0011<<0) | (0b0011<<4) | (0b0011<<8)|(0b0011<<12)|(0b0011<<16)|(0b0011<<20)|(0b0011<<24)|(0b0011<<28);  																									 
	DMA1_Channel1->CPAR = (uint32_t)&GPIOA->ODR;  
	DMA1_Channel1->CMAR = (uint32_t)data1;        
	DMA1_Channel1->CNDTR = 6;                         																									 
	DMA1_Channel1->CCR |= (1<<14);  
	DMA1_Channel1->CCR |= (1<<7);   
	DMA1_Channel1->CCR |= (1<<6);   
	DMA1_Channel1->CCR |= (1<<4);   
	DMA1_Channel1->CCR |= (1<<5);   
	DMA1_Channel1->CCR |= (1<<0);   

// 1. Bat clock DMA1 va GPIOA
RCC->AHBENR  |= RCC_AHBENR_DMA1EN;
RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;

// 2. Cau hinh PA0–PA7 la Output Push-Pull toc do 50MHz
GPIOA->CRL = 0; // Reset toan bo cau hinh chan PA0–PA7
GPIOA->CRL |= (0b0011<<0) | (0b0011<<4) | (0b0011<<8)|(0b0011<<12)|(0b0011<<16)|(0b0011<<20)|(0b0011<<24)|(0b0011<<28);  																									 
	

// 3. Cau hinh DMA1_Channel1 de chuyen du lieu den GPIOA->ODR
DMA1_Channel1->CPAR  = (uint32_t)&GPIOA->ODR;   // Dia chi ngoai vi
DMA1_Channel1->CMAR  = (uint32_t)data1;         // Dia chi bo nho
DMA1_Channel1->CNDTR = 6;                       // So luong du lieu can gui

// 4. Cau hinh CCR cua DMA
DMA1_Channel1->CCR =
    DMA_CCR_MINC    |   // Tang dia chi bo nho sau moi lan chuyen
    DMA_CCR_DIR     |   // Chuyen du lieu tu bo nho den ngoai vi
    DMA_CCR_CIRC;       // Che do vong lap (lap lai vinh vien)

// 5. Kich hoat DMA1 Channel1
DMA1_Channel1->CCR |= DMA_CCR_EN;


