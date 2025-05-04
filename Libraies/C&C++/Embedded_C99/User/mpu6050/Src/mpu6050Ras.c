#include "mpu6050Ras.h"
#include <math.h>


MPU6050_Data_t mpu_D;

float MPU6050_AccelScale = 0.0f;
float MPU6050_GyroScale = 0.0f;





void MPU6050_Init(int mpu, const MPU6050_Config* config) {
  // Cau hinh mac dinh
  static const MPU6050_Config default_config = {
    .PowerMgmt.BITS     = { .ClockSource = 1, .SleepMode = 0, .TemperatureDIS = 0 },
    .GyroConfig.BITS    = { .GyroRange = 1 },   // ±500°/s
    .AccelConfig.BITS   = { .AccelRange = 1 }, // ±4g
    .DLPF_Config.BITS   = { .DLPF_Config = 3 }, // DLPF 44Hz
    .SampleRateDiv.BITS = { .SampleRateDiv = 7 }, // Sample Rate = 1kHz
    .InterruptEN.BITS   = { .DataReadyInt = 1 }
  };
  // Neu khong co cau hinh, dung mac dinh
  if (!config) {
    config = &default_config;
  }
  // Reset thiet bi
  wiringPiI2CWriteReg8(mpu, RegPWR_Managment, 0x80);
  for (volatile int i = 0; i < 100000; i++); // Delay

  // Ghi cau hinh vao cac thanh ghi
  wiringPiI2CWriteReg8(mpu, RegSample_Rate  	, config->SampleRateDiv.REG);		// Sample Rate Divider
  wiringPiI2CWriteReg8(mpu, RegConfig       	, config->DLPF_Config.REG);  		// DLPF Config
  wiringPiI2CWriteReg8(mpu, RegGyro_Config  	, config->GyroConfig.REG);   		// Gyro Config
  wiringPiI2CWriteReg8(mpu, RegAcc_Config   	, config->AccelConfig.REG);  		// Accel Config
  wiringPiI2CWriteReg8(mpu, RegPWR_Managment	, config->PowerMgmt.REG);    		// Power Management
  wiringPiI2CWriteReg8(mpu, RegInterrupt    	, config->InterruptEN.REG);  		// Interrupt Enable
	
  // Tinh he so scale
	switch (config->AccelConfig.BITS.AccelRange){
		case 0:{ MPU6050_AccelScale = INVA_2; break;}
		case 1:{ MPU6050_AccelScale = INVA_4; break;}
		case 2:{ MPU6050_AccelScale = INVA_8; break;}
		case 3:{ MPU6050_AccelScale = INVA_16; break;}
	}
	switch (config->AccelConfig.BITS.AccelRange){
		case 0:{ MPU6050_GyroScale = INVG_250; break;}
		case 1:{ MPU6050_GyroScale = INVG_500; break;}
		case 2:{ MPU6050_GyroScale = INVG_1000; break;}
		case 3:{ MPU6050_GyroScale = INVG_2000; break;}
	}
}
//



void MPU6050_ReadAcc(int mpu, uint8_t ResAddr) {
  wiringPiI2CWrite(mpu,RegAcc_X,RMPU6050_RR,14);
	if (read(mpu, RMPU6050_RR, 14) != 14);

	RMPU6050_A.Acc_x = (float)((int16_t)((RMPU6050_RR[0] << 8) | RMPU6050_RR[1]))*MPU6050_AccelScale;
	RMPU6050_A.Acc_y = (float)((int16_t)((RMPU6050_RR[2] << 8) | RMPU6050_RR[3]))*MPU6050_AccelScale;
	RMPU6050_A.Acc_z = (float)((int16_t)((RMPU6050_RR[4] << 8) | RMPU6050_RR[5]))*MPU6050_AccelScale;
	
	RMPU6050_A.Gcc_x = ((float)((int16_t)((RMPU6050_RR[8] << 8) | RMPU6050_RR[9])))*MPU6050_GyroScale;
	RMPU6050_A.Gcc_y = ((float)((int16_t)((RMPU6050_RR[10] << 8) | RMPU6050_RR[11])))*MPU6050_GyroScale;
	RMPU6050_A.Gcc_z = ((float)((int16_t)((RMPU6050_RR[12] << 8) | RMPU6050_RR[13])))*MPU6050_GyroScale;

  RMPU6050_A2.Acc_x = RMPU6050_A.Acc_x * RMPU6050_A.Acc_x;
  RMPU6050_A2.Acc_y = RMPU6050_A.Acc_y * RMPU6050_A.Acc_y;
  RMPU6050_A2.Acc_z = RMPU6050_A.Acc_z * RMPU6050_A.Acc_z;
}
//
float MPU6050_ReadAgleX(){
	RMPU6050_An.pitch = atan2f(RMPU6050_A.Acc_x, sqrtf(RMPU6050_A2.Acc_y + RMPU6050_A2.Acc_z)) * 180.0f / M_PI;
	return RMPU6050_An.pitch;
}
//
float MPU6050_ReadAgleY(){
  RMPU6050_An.roll = atan2f(RMPU6050_A.Acc_y, sqrtf(RMPU6050_A2.Acc_x + RMPU6050_A2.Acc_z)) * 180.0f / M_PI;
	return RMPU6050_An.roll;
}
//===================================================




















