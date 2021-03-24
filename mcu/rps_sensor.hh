#pragma once

#include "ds/array.hh"
#include "ds/ring_buffer.hh"

#include <Arduino.h>
#include <stddef.h>
#include <stdint.h>

// TODO: document Spoke Count FSM

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

  enum class State : uint_fast8_t
  {
    Reset = 0,
    Count = 1,
    Mark = 2,
    Forward = 3,
    Reverse = 4,
  };

  struct Sensor
  {
    uint8_t pin;

    Sensor(uint8_t pin_) : pin(pin_)
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
  Array<Sensor, 2> _sensors;
  State _state = State::Reset;
  int_fast8_t _direction = 0;
  int_fast8_t _direction_prev = 0;

public:
  RPSSensor(uint8_t pin0, uint8_t pin1, volatile bool& timer_interrupt_flag)
  : _timer_interrupt_flag{timer_interrupt_flag},
    _sensors{{Sensor{pin0}, Sensor{pin1}}}
  {}

  /// Must be run on each loop to accumulate sensor data. Return true if the
  /// angular velocity has been updated.
  inline bool update()
  {
    bool const has_update = _timer_interrupt_flag;
    bool const a = _sensors[0].detects_spoke();
    bool const b = _sensors[1].detects_spoke();

    if ((_state == State::Reset) && (a ^ b))
    {
      _state = State::Count;
    }
    else if ((_state == State::Reset) && (a & b))
    {
      _state = State::Mark;
    }
    else if ((_state == State::Count) && (!a & !b))
    {
      _spoke_counts.current() += 1;
      _state = State::Reset;
    }
    else if ((_state == State::Count) && (a & b))
    {
      _spoke_counts.current() += 1;
      _state = State::Mark;
    }
    else if ((_state == State::Mark) && (!a & !b))
    {
      _state = State::Reset;
    }
    else if ((_state == State::Mark) && (a & !b))
    {
      _direction += 1;
      _state = State::Forward;
    }
    else if ((_state == State::Mark) && (!a & b))
    {
      _direction -= 1;
      _state = State::Reverse;
    }
    else if ((_state == State::Forward) && (!a & !b))
    {
      _spoke_counts.current() += 1;
      _state = State::Reset;
    }
    else if ((_state == State::Reverse) && (!a & !b))
    {
      _spoke_counts.current() += 1;
      _state = State::Reset;
    }

    if (has_update)
    {
      int16_t sum = _spoke_counts.sum<int16_t>();
      if (_direction < 0 || ((_direction == 0) && (_direction_prev < 0)))
        sum *= -1;

      static constexpr float multiplier =
        (float)sample_freq_hz / (float)wheel_spokes;
      _rps = (float)sum * multiplier;

      if (_direction != 0)
        _direction_prev = _direction;

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
