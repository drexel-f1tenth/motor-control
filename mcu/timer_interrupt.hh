#pragma once

#include <Arduino.h>

// Set to true at a frequency of 64 Hz. It is the responsibility of the `loop`
// callback to unset this flag before returning.
// TODO: probably not volatile...
volatile bool timer_interrupt_flag = false;

#if defined(ARDUINO_AVR_UNO) || defined(__AVR_ATmega2560__)
/// (interferes with analog pins 9, 10).
ISR(TIMER2_COMPA_vect)
#else
ISR(TIMER3_COMPA_vect)
#endif
{
  timer_interrupt_flag = true;
}

static inline void setup_timer_interrupt()
{
  noInterrupts();
#if defined(ARDUINO_AVR_UNO) || defined(__AVR_ATmega2560__)
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
