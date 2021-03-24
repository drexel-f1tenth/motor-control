#pragma once

class PID
{
  float _kp;
  float _ki;
  float _kd;
  float _integral = 0.0;
  float _prev_err = 0.0;

public:
  PID(float kp, float ki, float kd) : _kp{kp}, _ki{ki}, _kd{kd} {}

  float update(float value, float setpoint)
  {
    float const err = setpoint - value;
    _integral += err;
    float const p = _kp * err;
    float const i = _ki * _integral;
    float const d = _kd * (err - _prev_err);
    _prev_err = err;
    return p + i + d;
  }
};
