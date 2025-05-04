#include "main.h"


void Config_SPI();


uint8_t SPI_Transfer(uint8_t TX_Data);


void DMA_SPI1_Transfer(volatile DMA_Channel_TypeDef *DMAChannel, volatile void * MemPointer, volatile void* PeriPointer, unsigned long Length);


void send_data(uint8_t address,uint8_t data);


void Init_max7219(void);


void Test_Led(uint8_t T_data);


void Show_Team(uint16_t Team, uint8_t Index);


void Show_Shule(uint16_t TeamA, uint16_t TeamB, uint8_t Haf);