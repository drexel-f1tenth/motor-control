#include "ring_buffer.h"

#include <Arduino.h>

// Arduino Mega: Timer2 is tied to pins 9, 10

static constexpr size_t timer2_freq_hz = 64;
// A sampling rate of 8 Hz is about as fast as we can get meaningful data.
static constexpr size_t timer2_freq_divider = 8;

void setup_timer2()
{
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

struct RPSState
{
  size_t spoke_count = 0;
  int16_t pin;
  bool on_spoke = false;
  // RingBuffer<uint8_t, timer2_freq_hz> spoke_counts;

  RPSState(int16_t pin_) : pin(pin_) {}
};

static RPSState rps_states[] = {RPSState(A0), RPSState(A8)};
static constexpr size_t rps_states_len =
  sizeof(rps_states) / sizeof(*rps_states);

void setup()
{
  Serial.begin(9600);

  for (size_t i = 0; i < rps_states_len; i++)
  {
    pinMode(rps_states[i].pin, INPUT);
  }

  setup_timer2();
}

static constexpr size_t threshold = 700;
static bool interrupt_fired = false;

void loop()
{
  for (size_t i = 0; i < rps_states_len; i++)
  {
    auto& rps = rps_states[i];
    size_t value = analogRead(rps.pin);
    bool detect_spoke = (value < threshold);
    if (!rps.on_spoke && detect_spoke)
    { // rising edge of spoke detection
      if (detect_spoke)
        rps.spoke_count += 1;
    }
    rps.on_spoke = detect_spoke;
  }

  if (interrupt_fired)
  {
    for (size_t i = 0; i < rps_states_len; i++)
    {
      auto& rps = rps_states[i];
      static constexpr float multiplier =
        (float)timer2_freq_hz / (float)timer2_freq_divider / 6.0;
      float value = (float)rps.spoke_count * multiplier;
      Serial.println(
        String(i) + ": value: " + String(analogRead(rps.pin)) +
        ", count: " + String(rps.spoke_count) + ", RPS: " + String(value));
      rps.spoke_count = 0;
    }
    interrupt_fired = false;
  }
}

ISR(TIMER2_COMPA_vect)
{
  static size_t count = 0;
  count++;
  if (count == timer2_freq_divider)
  {
    count = 0;
    interrupt_fired = true;
  }
}
