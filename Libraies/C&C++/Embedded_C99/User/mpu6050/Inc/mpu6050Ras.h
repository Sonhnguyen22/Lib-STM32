#ifndef MPU6050RAS_H
#define MPU6050RAS_H

#include "main.h"


typedef struct {
	union {
    uint8_t RawReg[14];  // Buffer raw cho accelerometer + gyro (6x2 + 1x2 = 14 bytes)
    struct {
      // Raw sensor data (little-endian)
      int16_t RawA_x;
      int16_t RawA_y;
      int16_t RawA_z;
      int16_t RawTempSensor;  // nhiet do
      int16_t RawG_x;
      int16_t RawG_y;
      int16_t RawG_z;
    } Raw;
  } Raw;
	union {
    float AccReg[7];
		struct {
			float Acc_x;
			float Acc_y;
			float Acc_z;
			float Gcc_x;
			float Gcc_y;
			float Gcc_z;
		} Acc;
	} Acc;
	union {
    float Acc2Reg[7];
		struct {
			float Acc_x;
			float Acc_y;
			float Acc_z;
			float Gcc_x;
			float Gcc_y;
			float Gcc_z;			
		} Acc2;
	} Acc2;
		union {
    float AngleReg[7];
		struct {
			float pitch;
			float roll;
			float yaw;	
		} Angle;
	} Angle;
} MPU6050_Data_t;
extern MPU6050_Data_t mpu_D;
#define RMPU6050_RR mpu_D.Raw.RawReg
#define RMPU6050_R mpu_D.Raw.Raw
#define RMPU6050_AR mpu_D.Acc.AccReg
#define RMPU6050_A mpu_D.Acc.Acc
#define RMPU6050_AnR mpu_D.Angle.AngleReg
#define RMPU6050_An mpu_D.Angle.Angle
#define RMPU6050_A2R mpu_D.Acc2.Acc2Reg
#define RMPU6050_A2 mpu_D.Acc2.Acc2


typedef struct {
	  // Cau hinh bo chia tan so
  union {
    uint8_t REG;
    struct {
      uint8_t SampleRateDiv       : 8; //												   f/(samplerate + 1)
    } BITS;										     		
  } SampleRateDiv;                
	// Cau hinh bo loc DLPF         
  union {                         
    uint8_t REG;                  
    struct {                      
      uint8_t DLPF_Config         : 3; // 												(0->6) 0: DLPF 260Hz
      uint8_t EXT_SYNC_SET        : 3; //																 1: DLPF 184Hz
			uint8_t _Reserved           : 2; //                                2: DLPF 94Hz
    } BITS;                            //                                3: DLPF 44Hz
  } DLPF_Config;                       //                                4: DLPF 21Hz
  // Cau hinh Gyro                     //																 5: DLPF 10Hz
  union {                              //																 6: DLPF 5Hz
    uint8_t REG;                  
    struct {                      
      uint8_t _Reserved           : 3; //
      uint8_t GyroRange           : 2; //  												(0->3) 0: ±250°/s
      uint8_t SelfTest            : 3; //  Auto Check             			 1: ±500°/s
    } BITS;                            //                         			 2: ±1000°/s
  } GyroConfig;                        //                         			 3: ±2000°/s
  // Cau hinh Accel               
  union {                         
    uint8_t REG;                       
    struct {                          
      uint8_t _Reserved           : 3; //	
      uint8_t AccelRange          : 2; //                         (0->3) 0: ±2g
      uint8_t SelfTest            : 3; // Auto Check										 1: ±4g
		} BITS;                            //																 2: ±8g
	} AccelConfig;                       //																 3: ±16g
  // Cau hinh nguon và che do ngu      
  union {                         
    uint8_t REG;                  
    struct {                      
      uint8_t ClockSource         : 3; // Source Clock 					  (0->7) 0: Internal 8MHz  
			uint8_t TemperatureDIS      : 1; // On/Off Temperature					   1: PLL with X_Gyro
      uint8_t _Reserved           : 1; //	  													 	 2: PLL with Y_Gyro
			uint8_t CycleMode           : 1; // On/Off Temperature					   3: PLL with Z_Gyro
      uint8_t SleepMode           : 1; // Sleep Mode              		   4: PLL with External 32.768kHz
      uint8_t Reset               : 1; // Reset                   		   5: PLL with External 19.2MHz
    } BITS;                       		 //                         		   6: _Reserved
	} PowerMgmt;                         //                         		   7: Stops the clock and keeps the timing generatorin rese
  // Cau hinh Interrupt            
  union {                         
    uint8_t REG;                  
    struct {                      
      uint8_t DataReadyInt        : 1; // Iterrupt EN when data Ready
			uint8_t _Reserved           : 7;
			uint8_t I2C_MSB_INT_EN      : 1; // 
			uint8_t FIFO_OFLOW_EN       : 1; // 
      uint8_t _Reserved1          : 3;
    } BITS;
  } InterruptEN;
} MPU6050_Config;
//

extern float MPU6050_AccelScale;
extern float MPU6050_GyroScale;
 


  




#define M_PI                     3.14159265358979f
#define INVA_2                  (1.0f / 16384.0f)
#define INVA_4                  (1.0f / 8192.0f)
#define INVA_8                  (1.0f / 4096.0f)
#define INVA_16                 (1.0f / 2048.0f)
#define INVG_250                (1.0f / 131.0f)
#define INVG_500                (1.0f / 65.5f)
#define INVG_1000               (1.0f / 32.8f)
#define INVG_2000               (1.0f / 16.4f)

#define MPU6050_Addr 						0x68
#define RegSample_Rate					25
#define RegConfig               26
#define RegGyro_Config          27
#define RegAcc_Config           28
#define RegInterrupt            56
#define RegPWR_Managment        107
#define	RegAcc_X                59
#define RegAcc_Y                61
#define RegAcc_Z                63




void MPU6050_Init(I2C_TypeDef *I2Cx, const MPU6050_Config* config);

void MPU6050_ReadAcc(volatile I2C_TypeDef* I2Cx, uint8_t ResAddr);

float MPU6050_ReadAgleX();

float MPU6050_ReadAgleY();



#endif