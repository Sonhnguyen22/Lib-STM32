#include "stm32f1xx_vtd_spi.h"


void Config_SPI(){
	RCC->APB2ENR |= (1<<2)|(1<<12);
	RCC->AHBENR |= (1<<0);
	
	SPI1->CR1 = 0;
	SPI1->CR2 = 0; 
	SPI1->CR1 |= (1<<14)| (1<<7)|(0b111<<3)|(1<<2)|(1<<1);
	SPI1->CR2 |= (1<<1);
	SPI1->CR1 |= (1<<6);

	DMA1_SPI1_TX->CCR |= (1<<4)|(1<<7);

}

uint8_t data= 0xA8;
uint8_t SPI_Transfer(uint8_t TX_Data){
	uint8_t RX_Data = 0;
	SPI1->CR1 |= (1u<<6);
	SPI1->DR = (uint16_t)(TX_Data << 8);
	while( ((SPI1->SR)&(1u<<7)) || (!((SPI1->SR)&(1u<<0))) );
	
	RX_Data = (uint8_t)SPI1->DR;
	return RX_Data;
}

void DMA_SPI1_Transfer(volatile DMA_Channel_TypeDef *DMAChannel, volatile void * MemPointer, volatile void* PeriPointer, unsigned long Length){
    DMAChannel->CCR &= ~(1<<0);
    DMAChannel->CNDTR = Length;
    DMAChannel->CMAR = (unsigned long)MemPointer;
    DMAChannel->CPAR = (unsigned long)PeriPointer;
    DMAChannel->CCR |= (1<<0);
  
}

uint8_t buff[2];
void send_data(uint8_t address,uint8_t data){
	HAL_Delay(1);
	while((SPI1->SR & SPI_SR_BSY));
	buff[0] = address;
  buff[1] = data;
	GPIOA->ODR &= ~(1<<4);
	DMA_SPI1_Transfer(DMA_SPI1_TX,&buff,&SPI1->DR,2);
	while((SPI1->SR & SPI_SR_BSY));
	GPIOA->ODR |= (1<<4);
	HAL_Delay(1);
}

void Init_max7219(void){
	
    // decode mode: use code B for all digits -> 0x09FF
    send_data(0x09, 0x00);
    // intensity: 70% --> 0x0A0B

    send_data(0x0A, 0x01);
    // scan limit

    send_data(0x0B, 7);

    // turn off display test, no shutdow
    send_data(0x0F, 0);

    send_data(0x0C, 1);
	
}

void Test_Led(uint8_t T_data){
	for(unsigned char i = 0; i < 8; i++){
		send_data(i+1,T_data);
		HAL_Delay(1);
	}
	send_data(8,T_data);
}
void Show_Team(uint16_t Team, uint8_t Index){
	uint8_t chuc = Team/10;
	uint8_t donvi = (Team%10);
	uint8_t chuc_In = Index/10;
	uint8_t donvi_In = (Index%10);
	send_data(chuc_In,Num[chuc]);
	send_data(donvi_In, Num[donvi]);
	
}
void Show_Shule(uint16_t TeamA, uint16_t TeamB, uint8_t Haf){
	send_data(8,Num[11]);
	Show_Team(TeamA, 76);
	send_data(5,Num[10]);
	Show_Team(TeamB, 43);
	send_data(2,Num[Haf]);
	send_data(1,Num[11]);
	send_data(1,Num[11]);
}


