#ifndef MPU6050_h
#define MPU6050_h

class MPU6050{
  public:
    MPU6050();
    MPU6050(uint8_t address);

    int16_t ax_os, ay_os, az_os, gx_os, gy_os, gz_os;
    float accel_scale, gyro_scale;

    void initialize();
    void readRawData(int16_t* ax, int16_t* ay, int16_t* az, int16_t* gx, int16_t* gy, int16_t* gz);
    void readScaledData(float* ax_s, float* ay_s, float* az_s, float* gx_s, float* gy_s, float* gz_s);
    void setDLPF(uint8_t bandwidth);
    void setGyroScale(int gyro);
    void setAccelScale(int accel);
    void readOffSet();

  private:
    uint8_t mpu_addr;
};
#endif

