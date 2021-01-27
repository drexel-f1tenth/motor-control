#pragma once

#include "array.h"
#include "ring_buffer.h"

#include <Arduino.h>
#include <stddef.h>
#include <stdint.h>

/// Timer2 interrupt used to update RPS sensors.
/// (interferes with analog pins 9, 10).
// TODO: atomic
static bool timer2_interrupt_fired;
ISR(TIMER2_COMPA_vect)
{
  timer2_interrupt_fired = true;
}

/// Tacks the angular velocity (in RPS) of a wheel.
/// Instantaneous samples at 64Hz, Accurate accumulation at 8Hz.
/// Note: `setup_timer2` must be called on setup to set the necessary interrupts
/// on Timer2.
class RPSSensor
{
  /// Frequency of RPS sensor data collection, in Hz.
  static constexpr size_t update_freq_hz = 64;
  /// Frequency of "useful" RPS sensor data accumulation, in Hz.
  static constexpr size_t sample_freq_hz = 8;

  static constexpr size_t threshold = 700;
  static constexpr size_t buf_len = update_freq_hz / sample_freq_hz;
  static constexpr size_t wheel_spokes = 6;

  RingBuffer<uint8_t, buf_len> _spoke_counts;
  uint16_t _pin;
  bool _on_spoke = false;
  bool _shift = false;

public:
  RPSSensor(uint16_t pin) : _pin(pin) {}

  static void setup_timer2()
  {
    // generated from
    // https://www.arduinoslovakia.eu/application/timer-calculator

    noInterrupts();
    // Clear registers
    TCCR2A = 0;
    TCCR2B = 0;
    TCNT2 = 0;
    // 64.03688524590164 Hz (16000000/((243+1)*1024))
    OCR2A = 243;
    // CTC
    TCCR2A |= (1 << WGM21);
    // Prescaler 1024
    TCCR2B |= (1 << CS22) | (1 << CS21) | (1 << CS20);
    // Output Compare Match A Interrupt Enable
    TIMSK2 |= (1 << OCIE2A);
    interrupts();
  }

  /// Must be run on each loop to accumulate sensor data. Return true if the
  /// angular velocity has been updated.
  bool update()
  {
    if (_shift)
    {
      _shift = false;
      _spoke_counts.current() = 0;
    }

    size_t value = analogRead(_pin);
    bool detect_spoke = value < threshold;
    if (!_on_spoke && detect_spoke)
    { // rising edge of spoke detection
      _spoke_counts.current() += 1;
    }
    _on_spoke = detect_spoke;

    if (timer2_interrupt_fired)
    {
      _spoke_counts.shift();
      timer2_interrupt_fired = false;
      _shift = true;
      return true;
    }
    return false;
  }

  float rps() const
  {
    static constexpr float multiplier =
      (float)sample_freq_hz / (float)wheel_spokes;
    return (float)_spoke_counts.sum() * multiplier;
  }
};
