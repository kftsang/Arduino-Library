//------------------------------------------------------------------------------
// MPU6050.cpp
//------------------------------------------------------------------------------
//

// Header files
//------------------------------------------------------------------------------

#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include "MPU6050.h"

// I2C address
//------------------------------------------------------------------------------

#define MPU6050_ADDRESS_DEFAULT     0x68
#define MPU6050_ADDRESS_LOW         0x68  // I2C address of the MPU-6050
#define MPU6050_ADDRESS_HIGH        0x69  // I2C address of the MPU-6050

// Registers address
//------------------------------------------------------------------------------

#define MPU6050_RA_XG_OFFS_TC       0x00  //[7] PWR_MODE, [6:1] XG_OFFS_TC, [0] OTP_BNK_VLD
#define MPU6050_RA_YG_OFFS_TC       0x01  //[7] PWR_MODE, [6:1] YG_OFFS_TC, [0] OTP_BNK_VLD
#define MPU6050_RA_ZG_OFFS_TC       0x02  //[7] PWR_MODE, [6:1] ZG_OFFS_TC, [0] OTP_BNK_VLD

#define MPU6050_RA_XA_OFFS_H        0x06  //[15:0] XA_OFFS
#define MPU6050_RA_XA_OFFS_L_TC     0x07
#define MPU6050_RA_YA_OFFS_H        0x08  //[15:0] YA_OFFS
#define MPU6050_RA_YA_OFFS_L_TC     0x09
#define MPU6050_RA_ZA_OFFS_H        0x0A  //[15:0] ZA_OFFS
#define MPU6050_RA_ZA_OFFS_L_TC     0x0B

#define MPU6050_CONFIG              0x1A
#define MPU6050_GYRO_CONFIG         0x1B
#define MPU6050_ACCEL_CONFIG        0x1C
#define MPU6050_ACCEL_CONFIG_2      0x1D

#define MPU6050_ACCEL_XOUT_H        0x3B

#define MPU6050_PWR_MGMT_1          0x6B

// Functions
//------------------------------------------------------------------------------

MPU6050::MPU6050(){
  mpu_addr = MPU6050_ADDRESS_DEFAULT;

}

MPU6050::MPU6050(uint8_t address){
  mpu_addr = address;
}

void MPU6050::initialize(){
  int16_t ax_os = 0, ay_os = 0, az_os = 0, gx_os = 0, gy_os = 0, gz_os = 0;
  float accel_scale = 1, gyro_scale = 1;

  Wire.beginTransmission(mpu_addr);
  Wire.write(MPU6050_PWR_MGMT_1);  // register for power management
  Wire.write(0);                   // set to zero to wakes up the MPU-6050
  Wire.endTransmission(true);
  setGyroScale(3);
  setAccelScale(3);
  setDLPF(6);
  readOffSet();
}

void MPU6050::readRawData(int16_t* ax, int16_t* ay, int16_t* az, int16_t* gx, int16_t* gy, int16_t* gz){
  int16_t buf_temp;

  Wire.beginTransmission(mpu_addr);
  Wire.write(MPU6050_ACCEL_XOUT_H);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(mpu_addr, 14, true);  // request a total of 14 registers

  *ax = Wire.read() << 8 | Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
  *ay = Wire.read() << 8 | Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  *az = Wire.read() << 8 | Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)

  buf_temp = Wire.read() << 8 | Wire.read();

  *gx = Wire.read() << 8 | Wire.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  *gy = Wire.read() << 8 | Wire.read();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  *gz = Wire.read() << 8 | Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
}

void MPU6050::readScaledData(float* ax_s, float* ay_s, float* az_s, float* gx_s, float* gy_s, float* gz_s){
  int16_t buf_temp;

  Wire.beginTransmission(mpu_addr);
  Wire.write(MPU6050_ACCEL_XOUT_H);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(mpu_addr, 14, true);  // request a total of 14 registers

  *ax_s = ((Wire.read() << 8 | Wire.read()) - ax_os) * 9.81 / accel_scale;
  *ay_s = ((Wire.read() << 8 | Wire.read()) - ay_os) * 9.81 / accel_scale;
  *az_s = ((Wire.read() << 8 | Wire.read()) - az_os) * 9.81 / accel_scale;

  buf_temp = Wire.read() << 8 | Wire.read();

  *gx_s = ((Wire.read() << 8 | Wire.read()) - gx_os) / gyro_scale;
  *gy_s = ((Wire.read() << 8 | Wire.read()) - gy_os) / gyro_scale;
  *gz_s = ((Wire.read() << 8 | Wire.read()) - gz_os) / gyro_scale;

}

void MPU6050::setDLPF(uint8_t bandwidth){
  Wire.beginTransmission(mpu_addr);
  Wire.write(MPU6050_CONFIG);
  if (bandwidth >=0 && bandwidth <= 6)
    Wire.write(bandwidth);
  else
    Wire.write(0x00);

  Wire.beginTransmission(mpu_addr);
  Wire.write(MPU6050_ACCEL_CONFIG_2);
  if (bandwidth >=0 && bandwidth <= 6)
    Wire.write(bandwidth);
  else
    Wire.write(0x00);
}

void MPU6050::setGyroScale(int gyro){
  Wire.beginTransmission(mpu_addr);
  Wire.write(MPU6050_GYRO_CONFIG);
  switch (gyro) {
    case 0:
      Wire.write(0b00000000);  // Range: 250deg/s
      gyro_scale = 131.068;    // 131.068 LSB/(deg/s)
      break;
    case 1:
      Wire.write(0b00001000);  // Range: 500deg/s
      gyro_scale = 65.534;     // 65.5 LSB/(deg/s)
      break;
    case 2:
      Wire.write(0b00010000);  // Range: 1000deg/s
      gyro_scale = 32.767;     // 32.8 LSB/(deg/s)
      break;
    case 3:
      Wire.write(0b00011000);  // Range: 2000deg/s
      gyro_scale = 16.3835;    // 16.4 LSB/(deg/s)
      break;
    default:
      gyro_scale = 1;          // Raw Data
      break;
  }
  Wire.endTransmission(true);
}

void MPU6050::setAccelScale(int accel){
  Wire.beginTransmission(mpu_addr);
  Wire.write(MPU6050_ACCEL_CONFIG);
  switch (accel) {
    case 0:
      Wire.write(0b00000000);  // Range: 2g
      accel_scale = 16384;     // 16384 LSB/g
      break;
    case 1:
      Wire.write(0b00001000);  // Range: 4g
      accel_scale = 8192;      // 8192 LSB/g
      break;
    case 2:
      Wire.write(0b00010000);  // Range: 8g
      accel_scale = 4096;      // 4096 LSB/g
      break;
    case 3:
      Wire.write(0b00011000);  // Range: 16g
      accel_scale = 2048;      // 2048 LSB/g
      break;
    default:
      accel_scale = 1;         // Raw Data
      break;
  }
  Wire.endTransmission(true);
}

void MPU6050::readOffSet(){
  int16_t ax_los, ay_los, az_los, gx_los, gy_los, gz_los;

  delay(1000);   // wait until the system is stablized
  readRawData(&ax_os, &ay_os, &az_os, &gx_os, &gy_os, &gz_os);
  az_os = 0;     // set z offset to be 0 to account for gravity
}
