#ifndef HMC5883L_h
#define HMC5883L_h

class HMC5883L{
  public:
    HMC5883L();

    void initialize();
    void setMode(uint8_t mode);
    void setGain(uint8_t gain);
    void readRawData(int16_t* mx, int16_t* my, int16_t* mz);

  private:
    uint8_t hmc_addr;
};
#endif
