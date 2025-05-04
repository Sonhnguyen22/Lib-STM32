#ifndef MPU6050_H
#define MPU6050_H

#include "main.h"
#include "stm32f1xx_vtd_i2c.h"

typedef struct {
  uint8_t Raw[14];  // Buffer raw cho accelerometer + gyro (6x2 + 1x2 = 14 bytes)
  struct {
      float AccX;
      float AccY;
      float AccZ;  

      float Temp;

      float GyrX;
      float GyrY;
      float GyrZ;
  } ATG ;  
  struct {
      float pitch;
      float roll;
      float yaw;	
  } Angle;
} MPU6050_Data_t;
extern MPU6050_Data_t MPU6050_D;



typedef struct {
	  // Cau hinh bo chia tan so
  union {
    uint8_t REG;
    struct {
      uint8_t SampleRateDiv       : 8; //												   f/(samplerate + 1)
    } ;										     		
  } SampleRateDiv;                
	// Cau hinh bo loc DLPF         
  union {                         
    uint8_t REG;                  
    struct {                      
      uint8_t DLPF_Config         : 3; // 												(0->6) 0: DLPF 260Hz
      uint8_t EXT_SYNC_SET        : 3; //																 1: DLPF 184Hz
			uint8_t _Reserved           : 2; //                                2: DLPF 94Hz
    } ;                            //                                3: DLPF 44Hz
  } DLPF_Config;                       //                                4: DLPF 21Hz
  // Cau hinh Gyro                     //																 5: DLPF 10Hz
  union {                              //																 6: DLPF 5Hz
    uint8_t REG;                  
    struct {                      
      uint8_t _Reserved           : 3; //
      uint8_t GyroRange           : 2; //  												(0->3) 0: ±250°/s
      uint8_t SelfTest            : 3; //  Auto Check             			 1: ±500°/s
    } ;                            //                         			 2: ±1000°/s
  } GyroConfig;                        //                         			 3: ±2000°/s
  // Cau hinh Accel               
  union {                         
    uint8_t REG;                       
    struct {                          
      uint8_t _Reserved           : 3; //	
      uint8_t AccelRange          : 2; //                         (0->3) 0: ±2g
      uint8_t SelfTest            : 3; // Auto Check										 1: ±4g
		} ;                            //																 2: ±8g
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
    } ;                       		 //                         		   6: _Reserved
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
    } ;
  } InterruptEN;
} MPU6050_Config;
//
extern MPU6050_Config MPU6050_CF;
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



// Kalman Filter

typedef struct {
    float x;     // gia tri uot luong
    float P;     // phuong sai nhieu
    float Q;     // nhieu qua trinh
    float R;     // nhieu do luong
} KalmanFilter1D;

void Kalman_Init(KalmanFilter1D *kf, float init_value, float init_P, float Q, float R);
float Kalman_Update(KalmanFilter1D *kf, float z);


















//typedef struct {
//  float q; // do nhieu cua qua trinh 
//  float r; // do nhieu cua do luong 
//  float p; // hiep phuong sai loi uot luong
//  float k; // he so Kalman
//  float x; // Gia tri uot luong
//} KalmanFilter;


//void Kalman_Init(KalmanFilter* kf, float q, float r);
//float Kalman_Update(KalmanFilter* kf, float measurement);
//typedef struct {
//   float q0, q1, q2, q3;
//} Quaternion;

//// Bien trang thai
//typedef struct {
//  float velocity[3];       // Van toc 3 truc (m/s)
//  float gravity[3];        // Vector trong luc (m/s²)
//  Quaternion orientation;  // Huong cam bien
//  KalmanFilter kf[3];      // Bo loc Kalman cho 3 truc
//} MPU6050_State;
//extern MPU6050_State mpu_state;


////void MPU6050_Velocity_Init();
////void Update_Quaternion(float gx, float gy, float gz, float dt);
////void Update_Quaternion(float gx, float gy, float gz, float dt);
////void MPU6050_CalculateVelocity(float dt);





















#endif