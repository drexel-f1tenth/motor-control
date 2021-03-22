#pragma once

#include "ds/array.h"
#include "ds/ring_buffer.h"

#include <Arduino.h>
#include <stddef.h>
#include <stdint.h>

/// time interrupt used to update RPS sensors.
// TODO: atomic
static volatile bool rps_timer_interrupt_fired = false;
#ifdef __AVR_ATmega2560__
/// (interferes with analog pins 9, 10).
ISR(TIMER2_COMPA_vect)
#else
ISR(TIMER3_COMPA_vect)
#endif
{
  rps_timer_interrupt_fired = true;
}

/// Tacks the angular velocity (in RPS) of wheels.
/// Instantaneous samples at 64Hz, Accurate accumulation at 8Hz.
/// Note: `setup_timer_interrupt` must be called on setup to set the necessary
/// timer interrupts.
template<size_t _count>
class RPSSensors
{
  /// Frequency of RPS sensor data collection, in Hz.
  static constexpr size_t update_freq_hz = 64;
  /// Frequency of "useful" RPS sensor data accumulation, in Hz.
  static constexpr size_t sample_freq_hz = 8;

  static constexpr size_t threshold = 700;
  static constexpr size_t wheel_spokes = 6;

  struct SensorState
  {
    static constexpr size_t buf_len = update_freq_hz / sample_freq_hz;
    RingBuffer<uint8_t, buf_len> spoke_counts;
    float rps = 0.0;
    uint8_t pin = UINT8_MAX;
    bool on_spoke = false;

    SensorState() {}

    SensorState(uint8_t pin_) : pin(pin_)
    {
      pinMode(pin, INPUT);
    }
  };

  Array<SensorState, _count> _sensor_states;
  bool _shift = false;

  static void setup_timer_interrupt()
  {
    noInterrupts();
#ifdef __AVR_ATmega2560__
    TCCR2A = 0;
    TCCR2B = 0;
    TCNT2 = 0;
    OCR2A = 243;
    TCCR2A |= (1 << WGM21);
    TCCR2B |= (1 << CS22) | (1 << CS21) | (1 << CS20);
    TIMSK2 |= (1 << OCIE2A);
#else
    TCCR3A = 0;
    TCCR3B = 0;
    TCNT3 = 0;
    OCR3A = 31249;
    TCCR3A |= (1 << WGM32);
    TCCR3B |= (1 << CS31);
    TIMSK3 |= (1 << OCIE3A);
#endif
    interrupts();
  }

public:
  RPSSensors(Array<uint8_t, _count> pins)
  {
    for (size_t i = 0; i < _count; i++)
      _sensor_states[i] = SensorState(pins[i]);

    setup_timer_interrupt();
  }

  /// Must be run on each loop to accumulate sensor data. Return true if the
  /// angular velocity has been updated.
  bool update()
  {
    bool has_update = rps_timer_interrupt_fired;
    for (auto& state : _sensor_states)
    {
      size_t value = analogRead(state.pin);
      bool detect_spoke = value < threshold;
      if (!state.on_spoke && detect_spoke)
      { // rising edge of spoke detection
        state.spoke_counts.current() += 1;
      }
      state.on_spoke = detect_spoke;

      if (has_update)
      {
        static constexpr float multiplier =
          (float)sample_freq_hz / (float)wheel_spokes;
        state.rps = (float)state.spoke_counts.sum() * multiplier;

        state.spoke_counts.shift();
        state.spoke_counts.current() = 0;
      }
    }

    if (has_update)
      rps_timer_interrupt_fired = false;

    return has_update;
  }

  /// Return the RPS values for each sensor.
  Array<float, _count> values() const
  {
    Array<float, _count> vals;
    for (size_t i = 0; i < _count; i++)
      vals[i] = _sensor_states[i].rps;

    return vals;
  }
};
