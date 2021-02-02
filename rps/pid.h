#include <Arduino.h>

class PID
{
  float _min;
  float _max;
  float _kp;
  float _ki;
  float _kd;
  float _setpoint = 0.0;
  float _integral = 0.0;
  float _prev_err = 0.0;

public:
  PID(float min, float max, float kp, float ki, float kd)
  : _min(min), _max(max), _kp(kp), _ki(ki), _kd(kd)
  {}

  float update(float input, float setpoint)
  {
    float const err = setpoint - input;
    _integral += err;
    float const p = _kp * err;
    float const i = _ki * err;
    float const d = _kd * err;
    _prev_err = err;
    return constrain(p + i + d, _min, _max);
  }
};
