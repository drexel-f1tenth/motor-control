#pragma once

#include "array.h"
#include "ring_buffer.h"

#include <Arduino.h>
#include <stddef.h>
#include <stdint.h>

/// Timer2 interrupt used to update RPS sensors.
/// (interferes with analog pins 9, 10).
// TODO: atomic
static volatile bool timer2_interrupt_fired = false;
ISR(TIMER2_COMPA_vect)
{
  timer2_interrupt_fired = true;
}

/// Tacks the angular velocity (in RPS) of a wheel.
/// Instantaneous samples at 64Hz, Accurate accumulation at 8Hz.
/// Note: `setup_timer2` must be called on setup to set the necessary interrupts
/// on Timer2.
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
    uint8_t pin;
    bool on_spoke = false;

    SensorState(uint8_t pin_) : pin(pin_)
    {
      pinMode(pin, INPUT);
    }
  };

  Array<SensorState, 2> _sensor_states;
  bool _shift = false;

  static void setup_timer2()
  {
    // generated from
    // https://www.arduinoslovakia.eu/application/timer-calculator

    noInterrupts();
    // Clear registers
    TCCR1A = 0;
    TCCR1B = 0;
    TCNT1 = 0;
    // 64 Hz (16000000/((31249+1)*8))
    OCR1A = 31249;
    // CTC
    TCCR1A |= (1 << WGM12);
    // Prescaler 8
    TCCR1B |= (1 << CS11);
    // Output Compare Match A Interrupt Enable
    TIMSK1 |= (1 << OCIE1A);
    interrupts();
  }

public:
  RPSSensors(uint8_t pin1, uint8_t pin2)
  : _sensor_states({SensorState(pin1), SensorState(pin2)})
  {
    setup_timer2();
  }

  /// Must be run on each loop to accumulate sensor data. Return true if the
  /// angular velocity has been updated.
  bool update()
  {
    bool has_update = timer2_interrupt_fired;
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
      timer2_interrupt_fired = false;

    return has_update;
  }

  /// Return the RPS values for each sensor.
  Array<float, 2> values() const
  {
    return {_sensor_states[0].rps, _sensor_states[1].rps};
  }
};
