#include "stm32f1xx_vtd_i2c.h"


extern I2C_HandleTypeDef hi2c1;
extern I2C_HandleTypeDef hi2c2;
extern I2C_HandleTypeDef hi2c3;
extern I2C_HandleTypeDef hi2c4;

void I2C_MasterInit(I2C_HandleTypeDef* hi2c, I2C_TypeDef* I2Cx, uint32_t Baudrate){
  GPIO_InitTypeDef GPIO_InitStruct = {0};
	
  if (I2Cx == I2C1) {
		__HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_I2C1_CLK_ENABLE();
    GPIO_InitStruct.Pin = GPIO_PIN_6 | GPIO_PIN_7; // SCL, SDA
  } else if (I2Cx == I2C2) {
		__HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_I2C2_CLK_ENABLE();
    GPIO_InitStruct.Pin = GPIO_PIN_10 | GPIO_PIN_11; // SCL, SDA
  } else {
    return; // Không h? tr? I2C khác
  }

	GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;  
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	hi2c->Instance = I2Cx;
	hi2c->Init.ClockSpeed = Baudrate;
	hi2c->Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c->Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c->Init.DutyCycle = I2C_DUTYCYCLE_2;
	hi2c->Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c->Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	hi2c->Init.OwnAddress1 = 0x00;
	hi2c->Init.OwnAddress2 = 0;
	HAL_I2C_Init(hi2c);
	
}
//
void I2C_Start(volatile I2C_TypeDef* I2Cx){
	I2Cx->CR1 |= I2C_CR1_START;

}
//
void I2C_Stop(volatile I2C_TypeDef* I2Cx){
	I2Cx->CR1 |= I2C_CR1_STOP;
}
//
void I2C_ACK(volatile I2C_TypeDef* I2Cx){
	I2Cx->CR1 |= I2C_CR1_ACK;
}
//
void I2C_NACK(volatile I2C_TypeDef* I2Cx){
	I2Cx->CR1 &= ~I2C_CR1_ACK;
}
//
uint8_t I2C_WaitFlagSet(volatile uint32_t *reg, uint32_t flag) {
  uint32_t timeout = 0;
  while (!(*reg & flag)) {
    if (timeout++ > I2C_TIMEOUT) return 0xFF;  // Loi timeout
  }
  return 0;  // Thanh cong
}
//
void I2C_Write(volatile I2C_TypeDef* I2Cx, uint8_t data){
	I2C_WaitFlagSet(&I2Cx->SR1, I2C_SR1_TXE);
	I2Cx->DR = data;
	I2C_WaitFlagSet(&I2Cx->SR1, I2C_SR1_BTF);
}
//
uint8_t I2C_Address(volatile I2C_TypeDef* I2Cx, uint8_t Address){
  I2Cx->DR = Address; 
  I2C_WaitFlagSet(&I2Cx->SR1, I2C_SR1_ADDR);  
  if (I2Cx->SR1 & I2C_SR1_AF) { 
      I2Cx->SR1 &= ~I2C_SR1_AF;
      I2C_Stop(I2Cx);  
      return 0xFF;
  }
  (void)I2Cx->SR1;(void)I2Cx->SR2; 
	return 0;
}
//
void I2C_Write2(volatile I2C_TypeDef* I2Cx, uint8_t Address){
	I2Cx->DR = Address; 
}
//
void I2C_Write_Register(volatile I2C_TypeDef* I2Cx, uint8_t Addr, uint8_t reg, uint8_t data) {
  I2C_Start(I2Cx); 
	I2C_Address(I2Cx, Addr << 1);
	I2C_Write(I2Cx, reg);	
	I2C_Write(I2Cx, data);
  I2C_Stop(I2Cx);  
}
//
uint8_t I2C_WriteByte(volatile I2C_TypeDef* I2Cx, uint8_t devAddr, uint8_t regAddr, uint8_t data) {

  // 1. Gui Start Condition
  I2C_Start(I2Cx);
	if(I2C_WaitFlagSet(&I2Cx->SR1, I2C_SR1_SB)) return 0xFF;
  (void)I2Cx->SR1; // Xoa co SB

  // 2. Gui dia chi thiet bi (che do ghi)
  I2Cx->DR = (devAddr << 1); 
	if(I2C_WaitFlagSet(&I2Cx->SR1, I2C_SR1_ADDR)) return 0xFF;
	if (I2Cx->SR1 & I2C_SR1_AF) { 
      I2Cx->SR1 &= ~I2C_SR1_AF;
      I2C_Stop(I2Cx);  
      return 0xFF;
	}	
  (void)I2Cx->SR1; (void)I2Cx->SR2; // Xoa co ADDR

  // 3. Ghi data vao thanh ghi 
  I2Cx->DR = regAddr;
	if(I2C_WaitFlagSet(&I2Cx->SR1, I2C_SR1_TXE)) return 0xFF;
	I2Cx->DR = data;
	
	if(I2C_WaitFlagSet(&I2Cx->SR1, I2C_SR1_TXE)) return 0xFF;
	if(I2C_WaitFlagSet(&I2Cx->SR1, I2C_SR1_BTF)) return 0xFF;
	
	I2C_Stop(I2Cx);

  return 0;

}

//
uint8_t I2C_ReadByte(volatile I2C_TypeDef* I2Cx, uint8_t devAddr, uint8_t regAddr) {
  uint8_t data = 0;

  // 1. Gui Start Condition
  I2C_Start(I2Cx);
	if(I2C_WaitFlagSet(&I2Cx->SR1, I2C_SR1_SB)) return 0xFF;
  (void)I2Cx->SR1; // Xoa co SB

  // 2. Gui dia chi thiet bi (che do ghi)
  I2Cx->DR = (devAddr << 1); 
	if(I2C_WaitFlagSet(&I2Cx->SR1, I2C_SR1_ADDR)) return 0xFF;	
  (void)I2Cx->SR1; (void)I2Cx->SR2; // Xoa co ADDR
	if (I2Cx->SR1 & I2C_SR1_AF) { 
    I2Cx->SR1 &= ~I2C_SR1_AF;
    I2C_Stop(I2Cx);  
    return 0xFF;
	}
  // 3. Gui dia chi thanh ghi can doc
  I2Cx->DR = regAddr;
	if(I2C_WaitFlagSet(&I2Cx->SR1, I2C_SR1_TXE)) return 0xFF;

  // 4. Gui START condition lan 2
  I2C_Start(I2Cx);
  if(I2C_WaitFlagSet(&I2Cx->SR1, I2C_SR1_SB)) return 0xFF;
  (void)I2Cx->SR1; // Xoa co SB

  // 5. Gui dia chi thiet bi (che do doc)
  I2Cx->DR = (devAddr << 1) | 0x01; 
  if(I2C_WaitFlagSet(&I2Cx->SR1, I2C_SR1_ADDR)) return 0xFF;
  (void)I2Cx->SR1; (void)I2Cx->SR2; // Xoa co ADDR

  // 6. Tat ACK de doc byte cuoi cung
  I2C_NACK(I2Cx);

  // 7. Gui STOP truoc khi doc byte cuoi cung
  I2C_Stop(I2Cx);

  // 8. Doc du lieu
  if(I2C_WaitFlagSet(&I2Cx->SR1, I2C_SR1_RXNE)) return 0xFF;
  data = I2Cx->DR;

  // 9. Bat lai ACK cho cac lan doc tiep theo
  I2C_ACK(I2Cx);

  return data;

}









//
uint8_t I2C_ReadBytes(volatile I2C_TypeDef* I2Cx, uint8_t devAddr, uint8_t regAddr, uint8_t* pData, uint16_t len) {
  // 1. Gui Start Condition
  I2C_Start(I2Cx);
  if(I2C_WaitFlagSet(&I2Cx->SR1, I2C_SR1_SB)) return 1;
  (void)I2Cx->SR1; // Xoa co SB                                                                                                    
  // 2. Gui dia chi thiet bi (che do ghi)
  I2Cx->DR = (devAddr << 1);
  if(I2C_WaitFlagSet(&I2Cx->SR1, I2C_SR1_ADDR)) return 2;
  (void)I2Cx->SR1; (void)I2Cx->SR2; // Xoa co ADDR
	if (I2Cx->SR1 & I2C_SR1_AF) { 
    I2Cx->SR1 &= ~I2C_SR1_AF;
    I2C_Stop(I2Cx);  
    return 0xFF;
	}
  // 3. Gui dia chi thanh ghi
  I2Cx->DR = regAddr;
  if(I2C_WaitFlagSet(&I2Cx->SR1, I2C_SR1_TXE)) return 3;  
  // 4. Gui Start Condition lan 2
  I2C_Start(I2Cx);
  if(I2C_WaitFlagSet(&I2Cx->SR1, I2C_SR1_SB)) return 4;
  (void)I2Cx->SR1; // Xoa co SB 
  // 5. Gui dia chi thiet bi (che do doc)
  I2Cx->DR = (devAddr << 1) | 0x01;
  if(I2C_WaitFlagSet(&I2Cx->SR1, I2C_SR1_ADDR)) return 5;
  (void)I2Cx->SR1; (void)I2Cx->SR2; // Xoa co ADDR  
  // 6. Doc tung byte
  while(len > 0) {
    if(len == 1) {
      // Byte cuoi - gui NACK truoc khi doc
      I2C_NACK(I2Cx);
      I2C_Stop(I2Cx);
    } else {
      // Cac byte truoc - gui ACK
      I2C_ACK(I2Cx);
    }    
    // Cho du lieu san sang
    if(I2C_WaitFlagSet(&I2Cx->SR1, I2C_SR1_RXNE)) return 6;    
    // Doc du lieu
    *pData++ = I2Cx->DR;
    len--;
  }  
  // 7. Bat lai ACK cho cac lan doc tiep theo
  I2C_ACK(I2Cx); 
  return 0; // Thanh cong
}
//
/**
  * @brief  Ki?m tra tr?ng thái I2C
  * @param  I2Cx: Port I2C (I2C1, I2C2,...)
  * @param  devAddr: Ð?a ch? thi?t b? (7-bit)
  * @retval I2C_Status: Tr?ng thái bus I2C
  */
uint8_t I2C_Check_Error(I2C_TypeDef *I2Cx, uint8_t devAddr){
//{
//    
//    
//    /* 1. Ki?m tra tr?ng thái BUSY */
//   
//    while((I2Cx->SR2 & I2C_FLAG_BUSY) && ((DWT->CYCCNT - startTime) < timeout)) {
//        // Ch? cho d?n khi bus không b?n ho?c timeout
//    }
//    if(I2Cx->SR2 & I2C_FLAG_BUSY) return I2C_BUSY;
//    
//    /* 2. Kích ho?t I2C n?u chua b?t */
//    if(!(I2Cx->CR1 & I2C_CR1_PE)) {
//        I2Cx->CR1 |= I2C_CR1_PE;
//    }
//    
//    /* 3. T?o Start condition */
//    I2Cx->CR1 |= I2C_CR1_START;
//    
//    /* 4. Ch? SB flag */
//    startTime = DWT->CYCCNT;
//    while(!(I2Cx->SR1 & I2C_FLAG_SB) && ((DWT->CYCCNT - startTime) < timeout)) {
//        // Ch? Start condition ho?c timeout
//    }
//    if(!(I2Cx->SR1 & I2C_FLAG_SB)) {
//        I2Cx->CR1 |= I2C_CR1_STOP; // T?o Stop condition
//        return I2C_START_FAIL;
//    }
//    
//    /* 5. G?i d?a ch? thi?t b? */
//    I2Cx->DR = (devAddr << 1); // Ch? d? ghi
//    
//    /* 6. Ch? ADDR ho?c AF flag */
//    startTime = DWT->CYCCNT;
//    while(!(I2Cx->SR1 & (I2C_FLAG_ADDR | I2C_FLAG_AF)) && ((DWT->CYCCNT - startTime) < timeout)) {
//        // Ch? ph?n h?i t? thi?t b?
//    }
//    
//    /* 7. X? lý k?t qu? */
//    if(I2Cx->SR1 & I2C_FLAG_ADDR) {
//        // Thi?t b? ph?n h?i
//        (void)I2Cx->SR1; // Ð?c SR1 d? xóa ADDR flag
//        (void)I2Cx->SR2; // Ð?c SR2 (b?t bu?c)
//        I2Cx->CR1 |= I2C_CR1_STOP; // T?o Stop condition
//        return I2C_OK;
//    }
//    else if(I2Cx->SR1 & I2C_FLAG_AF) {
//        // Thi?t b? không ph?n h?i (NACK)
//        I2Cx->SR1 &= ~I2C_FLAG_AF; // Xóa AF flag
//        I2Cx->CR1 |= I2C_CR1_STOP; // T?o Stop condition
//        return I2C_ADDR_NACK;
//    }
//    else {
//        // Timeout
//        I2Cx->CR1 |= I2C_CR1_STOP; // T?o Stop condition
//        return I2C_TIMEOUT;
//    }
  return 0;
}
//
void I2C_UnlockBus(I2C_HandleTypeDef* hi2c, I2C_TypeDef* I2Cx, uint32_t Baudrate) {
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  // 1. C?u hình SCL & SDA thành Output Mode
  __HAL_RCC_GPIOB_CLK_ENABLE(); 
  GPIO_InitStruct.Pin = GPIO_PIN_6 | GPIO_PIN_7;  
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;     
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  // 2. Ki?m tra n?u SDA b? k?t ? m?c th?p
  if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_7) == GPIO_PIN_RESET) {
    for (int i = 0; i < 9; i++) {
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);  // SCL = HIGH
      HAL_Delay(1);
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);  // SCL = LOW
      HAL_Delay(1);
    }
  }

  // 3. G?i tín hi?u STOP
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);  // SDA = HIGH
  HAL_Delay(1);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);  // SCL = HIGH
  HAL_Delay(1);

  // 4. Kh?i d?ng l?i I2C
    I2C_MasterInit(hi2c, I2Cx, 100000);
}
//
uint8_t I2C_CheckDevice(I2C_TypeDef* I2Cx, uint8_t DevAddr) {
  // 1. Kiem tra trang thai BUSY de dam bao I2C san sang hoat dong
  if (I2C_WaitFlagSet(&I2Cx->SR2, I2C_SR2_BUSY)) return 1;  

  // 2. Gui Start Condition de bat dau truyen du lieu
  I2C_Start(I2Cx);
  if (I2C_WaitFlagSet(&I2Cx->SR1, I2C_SR1_SB)) return 2;  

  // 3. Gui dia chi thiet bi (7-bit) kem bit ghi (0) de kiem tra ket noi
  I2Cx->DR = (DevAddr << 1) & 0xFE;

  // 4. Cho phan hoi tu thiet bi (co the la ADDR hoac AF)
  if (I2C_WaitFlagSet(&I2Cx->SR1, I2C_SR1_ADDR))return 3;

  // 5. Neu nhan duoc NACK (AF = 1) 
  if (I2Cx->SR1 & I2C_SR1_AF) {
      I2Cx->SR1 &= ~I2C_SR1_AF;  // Xoa co loi AF
			
      I2C_Stop(I2Cx);  // Gui Stop Condition de ket thuc
      return 4;  // thiet bi khong ton tai tren bus
  }

  // 6. Neu nhan duoc ACK (ADDR = 1), thiet bi da phan hoi
  (void)I2Cx->SR2; (void)I2Cx->SR1;  // Doc thanh ghi SR1, SR2 de xoa co ADDR

  // 7. Gui Stop Condition de ket thuc qua trinh
  I2C_Stop(I2Cx);
  return 0;  // OK, thiet bi hoat dong
}
//








