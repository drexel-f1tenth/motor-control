#pragma once

#include <ICM_20948.h>
#include <SPI.h>

class IMU
{
  ICM_20948_SPI _device;

public:
  inline void init(uint8_t cs_pin)
  {
    SPI.begin();
    _device.begin(cs_pin, SPI);
  }

  inline bool ready()
  {
    return _device.dataReady();
  }

  inline ICM_20948_Status_e status() const
  {
    return _device.status;
  }

  inline char const* status_str()
  {
    return _device.statusString();
  }

  inline void update()
  {
    _device.getAGMT();
  }

  inline Array<float, 3> accelerometer_data()
  {
    return {_device.accX(), _device.accY(), _device.accZ()};
  }
};
