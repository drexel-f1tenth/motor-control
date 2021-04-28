#pragma once

#include "ds/array.hh"

#include <ICM_20948.h>
#include <Wire.h>

/// Inertial Measurement Unit connection.
class IMU
{
  ICM_20948_I2C _device;

public:
  /// Setup the I2C connection to the device.
  inline void init()
  {
    Wire.begin();
    Wire.setClock(400'000);
    _device.begin(Wire, 1);
  }

  /// Return the device status.
  inline ICM_20948_Status_e status() const
  {
    return _device.status;
  }

  /// Return the status string for the current status.
  inline char const* status_str()
  {
    return _device.statusString();
  }

  /// Reset the device.
  inline ICM_20948_Status_e reset()
  {
    _device.swReset();
    return status();
  }

  /// Return true if data from the device is ready. If true, use `update` to
  /// update the values.
  inline bool ready()
  {
    return _device.dataReady();
  }

  /// Update the data from the device.
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
