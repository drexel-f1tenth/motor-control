#pragma once

#include "ds/array.hh"

#include <ICM_20948.h>
#include <Wire.h>

class IMU
{
  ICM_20948_I2C _device;

public:
  inline void init()
  {
    Wire.begin();
    Wire.setClock(400'000);
    _device.begin(Wire, 1);
  }

  inline ICM_20948_Status_e reset()
  {
    _device.swReset();
    return status();
  }


  inline ICM_20948_Status_e wakeup()
  {
    _device.sleep(false);
    _device.lowPower(false);
    return status();
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

  inline Array<float, 3> gyroscope_data()
  {
    return {_device.gyrX(), _device.gyrY(), _device.gyrZ()};
  }

  inline Array<float, 3> magnetometer_data()
  {
    return {_device.magX(), _device.magY(), _device.magZ()};
  }
};

