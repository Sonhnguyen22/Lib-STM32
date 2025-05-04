#ifndef VTD_I2C_H
#define VTD_I2C_H

#include "main.h"

#define I2C_TIMEOUT 10000  // Timeout d? tránh treo máy



void I2C_MasterInit(I2C_HandleTypeDef* hi2c, I2C_TypeDef* I2Cx, uint32_t Baudrate);

void I2C_Start(volatile I2C_TypeDef* I2Cx);

void I2C_Stop(volatile I2C_TypeDef* I2Cx);

void I2C_NACK(volatile I2C_TypeDef* I2Cx);

void I2C_ACK(volatile I2C_TypeDef* I2Cx);

uint8_t I2C_WaitFlagSet(volatile uint32_t *reg, uint32_t flag);

uint8_t I2C_Address(volatile I2C_TypeDef* I2Cx, uint8_t Address);

void I2C_Write(volatile I2C_TypeDef* I2Cx, uint8_t data);

void I2C_Write2(volatile I2C_TypeDef* I2Cx, uint8_t Address);

void I2C_WriteMulti(uint8_t *Data, uint8_t size);

void I2C_Write_Register(volatile I2C_TypeDef* I2Cx,uint8_t Addr, uint8_t reg, uint8_t data);

uint8_t I2C_WriteByte(volatile I2C_TypeDef* I2Cx, uint8_t devAddr, uint8_t regAddr, uint8_t data);

uint8_t I2C_ReadByte(volatile I2C_TypeDef* I2Cx, uint8_t devAddr, uint8_t regAddr);

/**
  * @brief  Ð?c nhi?u byte liên ti?p t? thi?t b? I2C
  * @param  I2Cx: Port I2C (I2C1, I2C2,...)
  * @param  devAddr: Ð?a ch? thi?t b? (7-bit)
  * @param  regAddr: Ð?a ch? thanh ghi b?t d?u
  * @param  pData: Buffer chua du lieu doc duoc
  * @param  len: So byte can doc
  * @retval 0: Thành công, khác 0: L?i
  */
uint8_t I2C_ReadBytes(volatile I2C_TypeDef* I2Cx, uint8_t devAddr, uint8_t regAddr, uint8_t* pData, uint16_t len);
//
void I2C_UnlockBus(I2C_HandleTypeDef* hi2c, I2C_TypeDef* I2Cx, uint32_t Baudrate);

uint8_t I2C_CheckDevice(I2C_TypeDef* I2Cx, uint8_t DevAddr);

uint8_t I2C_Check_Error(I2C_TypeDef *I2Cx, uint8_t devAddr);







#endif