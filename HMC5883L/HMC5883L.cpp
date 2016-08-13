//------------------------------------------------------------------------------
// HMC5883L.cpp
//------------------------------------------------------------------------------
//

// Header files
//------------------------------------------------------------------------------

#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include "Condition.h"
#include "HMC5883L.h"

// I2C address
//------------------------------------------------------------------------------

#define HMC5883L_ADDRESS             0x1E  // 7-bit I2C address of HMC5883L

// Registers address
//------------------------------------------------------------------------------
#define HMC5883L_CONFIG_A            0x00
#define HMC5883L_CONFIG_B            0x01
#define HMC5883L_MODE                0x02
#define HMC5883L_X_MSB               0x03
#define HMC5883L_X_LSB               0x04
#define HMC5883L_Z_MSB               0x05
#define HMC5883L_Z_LSB               0x06
#define HMC5883L_Y_MSB               0x07
#define HMC5883L_Y_LSB               0x08

// Functions
//------------------------------------------------------------------------------

HMC5883L::HMC5883L(){
  hmc_addr = HMC5883L_ADDRESS;
}

void HMC5883L::initialize(){
  setMode(1);
  setGain(1);
}

//
//  Mode        Operation
//------------------------------------------
//   0          Continous Measurement
//   1          Single Measurement (Default)
//   2          Device Idle Mode
//   3          Device Idle Mode

void HMC5883L::setMode(uint8_t mode){
  Wire.beginTransmission(hmc_addr);
  Wire.write(HMC5883L_MODE);
  if (IsBetween(mode, 0, 3))
    Wire.write(mode);
  else
    Wire.write(0x01);
  Wire.endTransmission(true);
}

//
//  Mode   Sensor Range(Ga)    Gain(LSB/Ga)
//-----------------------------------------------
//   0        +/- 0.88            1370
//   1        +/- 1.3             1090 (Default)
//   2        +/- 1.9             820
//   3        +/- 2.5             660
//   4        +/- 4.0             440
//   5        +/- 4.7             390
//   6        +/- 5.6             330
//   7        +/- 8.1             230

void HMC5883L::setGain(uint8_t gain){
  Wire.beginTranmission(hmc_addr);
  Wire.write(HMC5883L_CONFIG_B);
  if (IsBetween(gain, 0, 7))
    Wire.write(gain << 5);
  else
    Wire.write(0x01);
}

void HMC5883L::readRawData(int16_t* mx, int16_t* my, int16_t* mz){
  Wire.beginTransmission(hmc_addr);
  Wire.write(HMC5883L_X_MSB);            // starting with register 0x03 (HMC5883L_X_MSB)
  Wire.endTransmission(false);
  Wire.requestFrom(hmc_addr, 6, true);   // request a total of 6 registers

  *mx = Wire.read() << 8 | Wire.read();  // 0x03 (HMC5883L_X_MSB) & 0x04 (HMC5883L_X_LSB)
  *my = Wire.read() << 8 | Wire.read();  // 0x05 (HMC5883L_Z_MSB) & 0x06 (HMC5883L_Z_LSB)
  *mz = Wire.read() << 8 | Wire.read();  // 0x07 (HMC5883L_Y_MSB) & 0x08 (HMC5883L_Y_LSB)
}
