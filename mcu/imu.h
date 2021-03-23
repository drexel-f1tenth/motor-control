#pragma once

#include <ICM_20948.h>
#include <SPI.h>

class IMU
{
  ICM_20948_SPI device;

public:
  inline void init(uint8_t cs_pin)
  {
    SPI.begin();
    device.begin(cs_pin, SPI);
  }

  inline ICM_20948_Status_e status() const
  {
    return device.status;
  }

  inline char const* status_str()
  {
    return device.statusString();
  }
};
