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
  static constexpr size_t encoder_spokes = 5;
  static constexpr size_t buf_len = update_freq_hz / sample_freq_hz;

  struct Sensor
  {
    static constexpr size_t filter_depth = 8;
    static_assert(__builtin_popcount(filter_depth) == 1);

    uint8_t pin;
    RingBuffer<uint16_t, filter_depth> filter;

    Sensor(uint8_t pin_) : pin(pin_)
    {
      pinMode(pin, INPUT);
    }

    bool detects_spoke()
    {
      filter.push(analogRead(pin));
      auto const value = filter.sum() / filter_depth;
      return value < threshold;
    }
  };

  float _rps = 0.0;
  RingBuffer<int8_t, buf_len> _spoke_counts;
  Array<Sensor, 2> _sensors;
  bool _armed = false;

public:
  RPSSensor(uint8_t pin0, uint8_t pin1) : _sensors{{Sensor{pin0}, Sensor{pin1}}}
  {}

  /// Must be run on each loop to accumulate sensor data. Updated values for
  /// angular velocity are made available at 64Hz.
  inline void update(bool timer_fired)
  {
    bool const a = _sensors[0].detects_spoke();
    bool const b = _sensors[1].detects_spoke();

    if (a && b)
    {
      _armed = true;
    }
    else if (_armed && (bool)(a ^ b))
    {
      _armed = false;
      int16_t const direction = a ? 1 : -1;
      _spoke_counts.current() += direction;
    }

    if (timer_fired)
    {
      auto const sum = _spoke_counts.sum();
      static constexpr float multiplier =
        (float)sample_freq_hz / (float)encoder_spokes;
      _rps = (float)sum * multiplier;

      _spoke_counts.shift();
      _spoke_counts.current() = 0;
    }
  }

  /// Return the RPS value.
  inline float value() const
  {
    return _rps;
  }
};
