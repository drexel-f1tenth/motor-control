#pragma once

#include "ds/array.hh"
#include "ds/ring_buffer.hh"

#include <Arduino.h>
#include <stddef.h>
#include <stdint.h>

// Spoke Count FSM:
//
//              rising edge a +-------+ rising edge b +---------------+
//            +-------------->| rev=0 |-------------->| count++,dir++ |-->reset
// +-------+  |               +-------+               +---------------+
// | reset |--|
// +-------+  | rising edge b +-------+ rising edge a +---------------+
//            +-------------->| rev=1 |-------------->| count++,dir-- |-->reset
//                            +-------+               +---------------+

/// Tacks the angular velocity (in RPS) of wheels.
/// Instantaneous samples at 64Hz, Accurate accumulation at 8Hz.
class RPSSensor
{
  /// Frequency of RPS sensor data collection, in Hz.
  static constexpr size_t update_freq_hz = 64;
  /// Frequency of "useful" RPS sensor data accumulation, in Hz.
  static constexpr size_t sample_freq_hz = 8;
  static constexpr int16_t threshold = 700;
  static constexpr size_t wheel_spokes = 6;
  static constexpr size_t buf_len = update_freq_hz / sample_freq_hz;

  struct SensorState
  {
    uint8_t pin;
    bool prev = false;

    SensorState(uint8_t pin_) : pin(pin_)
    {
      pinMode(pin, INPUT);
    }

    bool detects_spoke()
    {
      return analogRead(pin) < threshold;
    }
  };

  volatile bool& _timer_interrupt_flag;
  float _rps = 0.0;
  RingBuffer<uint8_t, buf_len> _spoke_counts;
  Array<SensorState, 2> _sensors;
  int_fast8_t _direction = 0;
  int_fast8_t _current_direction = 0;
  bool _reset = true;

public:
  RPSSensor(uint8_t pin0, uint8_t pin1, volatile bool& timer_interrupt_flag)
  : _timer_interrupt_flag{timer_interrupt_flag},
    _sensors{{SensorState{pin0}, SensorState{pin1}}}
  {}

  /// Must be run on each loop to accumulate sensor data. Return true if the
  /// angular velocity has been updated.
  inline bool update()
  {
    bool const has_update = _timer_interrupt_flag;
    bool const a_high = _sensors[0].detects_spoke();
    bool const b_high = _sensors[1].detects_spoke();
    bool const a_rising = !_sensors[0].prev && a_high;
    bool const b_rising = !_sensors[1].prev && b_high;
    if (a_rising && b_rising)
    {
      _reset = true;
      _spoke_counts.current() += 1;
    }
    else if (_reset && a_rising)
    {
      _reset = false;
      _current_direction = 1;
    }
    else if (_reset && b_rising)
    {
      _reset = false;
      _current_direction = -1;
    }
    else if (!_reset && (a_rising || b_rising))
    {
      _reset = true;
      _spoke_counts.current() += 1;
      _direction += _current_direction;
      _current_direction = 0;
    }
    _sensors[0].prev = a_high;
    _sensors[1].prev = b_high;

    if (has_update)
    {
      int16_t sum = _spoke_counts.sum<int16_t>();
      if (_direction < 0)
        sum *= -1;

      static constexpr float multiplier =
        (float)sample_freq_hz / (float)wheel_spokes;
      _rps = (float)sum * multiplier;

      _direction = 0;
      _spoke_counts.shift();
      _spoke_counts.current() = 0;
    }

    if (has_update)
      _timer_interrupt_flag = false;

    return has_update;
  }

  /// Return the RPS value.
  inline float value() const
  {
    return _rps;
  }
};
