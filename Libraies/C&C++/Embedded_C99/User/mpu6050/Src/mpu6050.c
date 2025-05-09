#include "mpu6050.h"
#include <math.h>


MPU6050_Data_t MPU6050_D;
MPU6050_Config MPU6050_CF = {
	.PowerMgmt.REG = 0,
	.GyroConfig.REG = 0,   
	.AccelConfig.REG = 0, 
	.DLPF_Config.REG = 0,   
	.SampleRateDiv.REG = 0,
	.InterruptEN.REG = 0
};

float MPU6050_AccelScale = 0.0f;
float MPU6050_GyroScale = 0.0f;



void MPU6050_Init(I2C_TypeDef *I2Cx, const MPU6050_Config* config) {
  // Cau hinh mac dinh
  static const MPU6050_Config default_config = {
    .PowerMgmt     = { .ClockSource = 1, .SleepMode = 0, .TemperatureDIS = 0 },
    .GyroConfig    = { .GyroRange = 1 },   // ±500°/s
    .AccelConfig   = { .AccelRange = 1 }, // ±4g
    .DLPF_Config   = { .DLPF_Config = 3 }, // DLPF 44Hz
    .SampleRateDiv = { .SampleRateDiv = 7 }, // Sample Rate = 1kHz
    .InterruptEN   = { .DataReadyInt = 1 }
  };
  // Neu khong co cau hinh, dung mac dinh
  if (!config) {
    config = &default_config;
  }
  // Reset thiet bi
  I2C_WriteByte(I2Cx, MPU6050_Addr, RegPWR_Managment, 0x80);
  for (volatile int i = 0; i < 100000; i++); // Delay

  // Ghi cau hinh vao cac thanh ghi
  I2C_WriteByte(I2Cx, MPU6050_Addr, RegSample_Rate   , config->SampleRateDiv.REG);		// Sample Rate Divider
  I2C_WriteByte(I2Cx, MPU6050_Addr, RegConfig        , config->DLPF_Config.REG);  		// DLPF Config
  I2C_WriteByte(I2Cx, MPU6050_Addr, RegGyro_Config   , config->GyroConfig.REG);   		// Gyro Config
  I2C_WriteByte(I2Cx, MPU6050_Addr, RegAcc_Config    , config->AccelConfig.REG);  		// Accel Config
  I2C_WriteByte(I2Cx, MPU6050_Addr, RegPWR_Managment , config->PowerMgmt.REG);    		// Power Management
  I2C_WriteByte(I2Cx, MPU6050_Addr, RegInterrupt     , config->InterruptEN.REG);  		// Interrupt Enable
																										 
  // Tinh he so scale
	switch (config->AccelConfig.AccelRange){
		case 0:{ MPU6050_AccelScale = INVA_2; break;}
		case 1:{ MPU6050_AccelScale = INVA_4; break;}
		case 2:{ MPU6050_AccelScale = INVA_8; break;}
		case 3:{ MPU6050_AccelScale = INVA_16; break;}
	}
	switch (config->AccelConfig.AccelRange){
		case 0:{ MPU6050_GyroScale = INVG_250; break;}
		case 1:{ MPU6050_GyroScale = INVG_500; break;}
		case 2:{ MPU6050_GyroScale = INVG_1000; break;}
		case 3:{ MPU6050_GyroScale = INVG_2000; break;}
	}
}

//
void MPU6050_ReadAcc(volatile I2C_TypeDef* I2Cx, uint8_t ResAddr) {
  I2C_ReadBytes(I2Cx,MPU6050_Addr,RegAcc_X,MPU6050_D.Raw,14);

  MPU6050_D.ATG.AccX = ((float)((int16_t)((MPU6050_D.Raw[0] << 8) | MPU6050_D.Raw[1])))*MPU6050_AccelScale;
  MPU6050_D.ATG.AccY = ((float)((int16_t)((MPU6050_D.Raw[2] << 8) | MPU6050_D.Raw[3])))*MPU6050_AccelScale;
  MPU6050_D.ATG.AccZ = ((float)((int16_t)((MPU6050_D.Raw[4] << 8) | MPU6050_D.Raw[5])))*MPU6050_AccelScale;
  MPU6050_D.ATG.Temp = ((float)((int16_t)((MPU6050_D.Raw[6] << 8) | MPU6050_D.Raw[7])));
  MPU6050_D.ATG.GyrX = ((float)((int16_t)((MPU6050_D.Raw[8] << 8) | MPU6050_D.Raw[9])))*MPU6050_GyroScale;
  MPU6050_D.ATG.GyrY = ((float)((int16_t)((MPU6050_D.Raw[10] << 8) | MPU6050_D.Raw[11])))*MPU6050_GyroScale;
  MPU6050_D.ATG.GyrZ = ((float)((int16_t)((MPU6050_D.Raw[12] << 8) | MPU6050_D.Raw[13])))*MPU6050_GyroScale;
}
//
float MPU6050_ReadAgleX(){
	float x = MPU6050_D.ATG.AccX * MPU6050_D.ATG.AccX;
  float y = MPU6050_D.ATG.AccY * MPU6050_D.ATG.AccY;
  float z = MPU6050_D.ATG.AccZ * MPU6050_D.ATG.AccZ;
  
  MPU6050_D.Angle.roll  = atan2f(MPU6050_D.ATG.AccY, sqrtf(x + z)) * 180.0f / M_PI;
	return MPU6050_D.Angle.pitch;
}
//
float MPU6050_ReadAgleY(){
	float x = MPU6050_D.ATG.AccX * MPU6050_D.ATG.AccX;
  float y = MPU6050_D.ATG.AccY * MPU6050_D.ATG.AccY;
  float z = MPU6050_D.ATG.AccZ * MPU6050_D.ATG.AccZ;
  MPU6050_D.Angle.roll  = atan2f(MPU6050_D.ATG.AccY, sqrtf(x + z)) * 180.0f / M_PI;
  return MPU6050_D.Angle.roll;
}

//===================================================

void Kalman_Init(KalmanFilter1D *kf, float init_value, float init_P, float Q, float R) {
    kf->x = init_value;
    kf->P = init_P;
    kf->Q = Q;
    kf->R = R;
}


float Kalman_Update(KalmanFilter1D *kf, float z) {
    // Du doan
    kf->P = kf->P + kf->Q;

    // Cap nhat
    float K = kf->P / (kf->P + kf->R);
    kf->x = kf->x + K * (z - kf->x);
    kf->P = (1 - K) * kf->P;

    return kf->x;
}


















// Bien trang thai Kalman Filter
//typedef struct {
//  float q; // do nhieu cua qua trinh 
//  float r; // do nhieu cua do luong 
//  float p; // hiep phuong sai loi uot luong
//  float k; // he so Kalman
//  float x; // Gia tri uot luong
//} KalmanFilter;

// Khoi tao bo loc Kalman
//void Kalman_Init(KalmanFilter* kf, float q, float r) {
//  kf->q = q; // do nhieu cua qua trinh 
//  kf->r = r; // do nhieu cua do luong 
//  kf->p = 0; // hiep phuong sai loi uot luong
//  kf->k = 0; // he so Kalman
//  kf->x = 0; // Gia tri uot luong
//}

//// Cap nhat bo loc Kalman
//float Kalman_Update(KalmanFilter* kf, float measurement) {
//  // Du doan gia tri tiep theo 
//  kf->p = kf->p + kf->q;
//  // Cap nhat he so Kalman
//  kf->k = kf->p / (kf->p + kf->r);
//	// Dieuchinh gia tri uot luong dua tren thang do moi
//  kf->x = kf->x + kf->k * (measurement - kf->x);
//	// Cap nhat phuong sai loi
//  kf->p = (1 - kf->k) * kf->p;
//  
//  return kf->x;
//}

// Quaternion structure
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

//MPU6050_State mpu_state;

//// Khoi tao he thong
//void MPU6050_Velocity_Init() {
//  // Khoi tao Kalman Filter cho moi truc
//  Kalman_Init(&mpu_state.kf[0], 0.0006f, 0.8f); // x
//  Kalman_Init(&mpu_state.kf[1], 0.01f, 0.1f); // y
//  Kalman_Init(&mpu_state.kf[2], 0.01f, 0.1f); // z
//  
//  // Gia dinh ban dau cam bien nam ngang
//  mpu_state.orientation.q0 = 1.0f;
//  mpu_state.orientation.q1 = 0.0f;
//  mpu_state.orientation.q2 = 0.0f;
//  mpu_state.orientation.q3 = 0.0f;
//  
//  // Khoi tao trong luc mac dinh
//  mpu_state.gravity[0] = 0.0f;
//  mpu_state.gravity[1] = 0.0f;
//  mpu_state.gravity[2] = 9.81f;
//}

//// Cap nhat quaternion tu gyro
//void Update_Quaternion(float gx, float gy, float gz, float dt) {
//  Quaternion q = mpu_state.orientation;
//  
//  // Dao ham quaternion
//  float qDot0 = 0.5f * (-q.q1 * gx - q.q2 * gy - q.q3 * gz);
//  float qDot1 = 0.5f * ( q.q0 * gx + q.q2 * gz - q.q3 * gy);
//  float qDot2 = 0.5f * ( q.q0 * gy - q.q1 * gz + q.q3 * gx);
//  float qDot3 = 0.5f * ( q.q0 * gz + q.q1 * gy - q.q2 * gx);
//  
//  // Tich phan
//  q.q0 += qDot0 * dt;
//  q.q1 += qDot1 * dt;
//  q.q2 += qDot2 * dt;
//  q.q3 += qDot3 * dt;
//  
//  // Chuan hoa quaternion
//  float norm = sqrtf(q.q0*q.q0 + q.q1*q.q1 + q.q2*q.q2 + q.q3*q.q3);
//  q.q0 /= norm;
//  q.q1 /= norm;
//  q.q2 /= norm;
//  q.q3 /= norm;
//  
//  mpu_state.orientation = q;
//}

//// Tinh vector trong luc tu quaternion
//void Update_Gravity_Vector() {
//  Quaternion q = mpu_state.orientation;
//  
//  // Tinh vector trong luc trong he toa do cam bien
//  mpu_state.gravity[0] = 2.0f * (q.q1*q.q3 - q.q0*q.q2) * 9.81f;
//  mpu_state.gravity[1] = 2.0f * (q.q0*q.q1 + q.q2*q.q3) * 9.81f;
//  mpu_state.gravity[2] = (q.q0*q.q0 - q.q1*q.q1 - q.q2*q.q2 + q.q3*q.q3) * 9.81f;
//}

//// Ham chinh tinh van toc
//void MPU6050_CalculateVelocity(float dt) {
////  // Doc du lieu tu cam bien (gia su do cu)
////  float acc_x = RMPU6050_A.Acc_x * 9.81f; // Chuyen tu g sang m/s²
////  float acc_y = RMPU6050_A.Acc_y * 9.81f;
////  float acc_z = RMPU6050_A.Acc_z * 9.81f;
////  
////  float gyro_x = RMPU6050_R.RawG_x / 131.0f * (M_PI/180.0f); // Chuyen sang rad/s
////  float gyro_y = RMPU6050_R.RawG_y / 131.0f * (M_PI/180.0f);
////  float gyro_z = RMPU6050_R.RawG_z / 131.0f * (M_PI/180.0f);
////  
////  // 1. Cap nhat huong cam bien bang quaternion
////  Update_Quaternion(gyro_x, gyro_y, gyro_z, dt);
////  
////  // 2. Cap nhat vector trong luc
////  Update_Gravity_Vector();
////  
////  // 3. Loai bo thanh phan trong luc khoi gia toc do duoc
////  float linear_acc_x = acc_x - mpu_state.gravity[0];
////  float linear_acc_y = acc_y - mpu_state.gravity[1];
////  float linear_acc_z = acc_z - mpu_state.gravity[2];
////  
////  // 4. Ap dung bo loc Kalman de giam nhieu
////  linear_acc_x = Kalman_Update(&mpu_state.kf[0], linear_acc_x);
////  linear_acc_y = Kalman_Update(&mpu_state.kf[1], linear_acc_y);
////  linear_acc_z = Kalman_Update(&mpu_state.kf[2], linear_acc_z);
////  
////  // 5. Tich phan de tinh van toc
////  mpu_state.velocity[0] += linear_acc_x * dt;
////  mpu_state.velocity[1] += linear_acc_y * dt;
////  mpu_state.velocity[2] += linear_acc_z * dt;
////  
////  // 6. Bo loc trong luc dong (giam drift)
////  for(int i = 0; i < 3; i++) {
////    if(fabs(mpu_state.velocity[i]) < 0.1f) { // Nguong van toc nho
////      mpu_state.velocity[i] *= 0.95f;     // Giam dan
////    }
////  }
//}
////



















