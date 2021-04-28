#pragma once

#include "ds/ring_buffer.hh"

template<size_t d_filter_depth = 4>
class PID
{
  RingBuffer<float, d_filter_depth> _d_filter;
  float _i = 0.0;
  float _kp;
  float _ki;
  float _kd;

public:
  PID(float kp, float ki, float kd) : _kp{kp}, _ki{ki}, _kd{kd} {}

  float update(float value, float setpoint)
  {
    auto const err = setpoint - value;
    _i += err;
    _d_filter.push(err - _d_filter.current());
    auto const p = _kp * err;
    auto const i = _ki * _i;
    auto const d = _kd * _d_filter.sum() / (float)_d_filter.cap();
    return p + i + d;
  }
};
