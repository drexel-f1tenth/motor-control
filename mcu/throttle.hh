#pragma once

#include "timer_interrupt.hh"

#include <Arduino.h>
#include <Servo.h>

// See documentation at: https://github.com/drexel-f1tenth/docs/blob/main/throttle-control.md
class Throttle
{
  static constexpr int16_t esc_neutral = 90;
  static constexpr int16_t throttle_cap = 40;
  static constexpr int16_t brake_magnitude = throttle_cap;

  enum struct State : uint8_t
  // clang-format off
  {
    Stop           = 0b0000,
    Forward        = 0b0100,
    ForwardNeutral = 0b0101,
    ForwardBrake   = 0b0110,
    Reverse        = 0b1000,
    ReverseNeutral = 0b1001,
    ReverseBrake   = 0b1010,
  };
  // clang-format on

  Servo _esc;
  int16_t _position = 0;
  State _state = State::Stop;

public:
  void init(uint8_t esc_pin)
  {
    _esc.attach(esc_pin);
    write_position(0);
  }

  void update(bool wheels_stopped)
  {
    if (!wheels_stopped)
      return;

    // Transition from neutral or brake to stop.
    if ((uint8_t)_state & 0b0011)
    {
      write_position(0);
      // TODO: Deal with the ESC lockout here instead of blocking everything
      // with `delay`.
      delay(40);
      _state = State::Stop;
    }
  }

  void set_position(int16_t position)
  {
    _position = position;

    if (position == 0)
    {
      write_position(0);
      if (moving_forward())
        _state = State::ForwardNeutral;
      else if (moving_reverse())
        _state = State::ReverseNeutral;
    }
    else if ((_state != State::Stop) && ((position > 0) != moving_forward()))
    {
      brake();
    }
    else if (position > 0)
    {
      write_position(position);
      _state = State::Forward;
    }
    else if (position < 0)
    {
      write_position(position);
      _state = State::Reverse;
    }
  }

  void brake()
  {
    if (_state == State::Stop)
      return;

    if (moving_forward())
    {
      _esc.write(esc_neutral - brake_magnitude);
      _state = State::ForwardBrake;
    }
    else if (moving_reverse())
    {
      _esc.write(esc_neutral + brake_magnitude);
      _state = State::ReverseBrake;
    }
  }

private:
  inline bool moving_forward() const
  {
    return ((uint8_t)_state & 0b0100) != 0;
  }

  inline bool moving_reverse() const
  {
    return ((uint8_t)_state & 0b1000) != 0;
  }

  inline void write_position(int16_t position)
  {
    position = constrain(position, -throttle_cap, throttle_cap);
    _esc.write(esc_neutral + position);
  }
};
