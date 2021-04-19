#pragma once

#include "ds/array.hh"

#include <ICM_20948.h>
#include <Wire.h>

#define WIRE_PORT Wire

class IMU
{
  ICM_20948_I2C _device;

public:
  inline void init()
  {
    WIRE_PORT.begin();
    WIRE_PORT.setClock(400000);
    _device.begin(WIRE_PORT, true);
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
