#include <Arduino.h>

void setup_timer()
{
  static constexpr size_t freq_hz = 1;
  static constexpr uint16_t cmp_value = (16000000 / (64 * freq_hz)) - 1;
  noInterrupts();
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1 = 0;
  OCR1A = cmp_value;
  // CTC mode, prescalar 64
  TCCR1B |= (1 << WGM12) | (1 << CS11) | (1 << CS10);
  TIMSK1 |= (1 << OCIE1A);
  interrupts();
}

struct RPMState
{
  int16_t pin;
  size_t spoke_count = 0;
  bool on_spoke = false;

  RPMState(int16_t pin_) : pin(pin_) {}
};

static RPMState rpm_states[] = {RPMState(A0), RPMState(A8)};

void setup()
{
  Serial.begin(9600);

  for (size_t i = 0; i < (sizeof(rpm_states) / sizeof(*rpm_states)); i++)
  {
    pinMode(rpm_states[i].pin, INPUT);
  }

  setup_timer();
}

static constexpr size_t threshold = 700;
static bool interrupt_fired = false;

void loop()
{
  for (size_t i = 0; i < (sizeof(rpm_states) / sizeof(*rpm_states)); i++)
  {
    auto& rpm = rpm_states[i];
    size_t value = analogRead(rpm.pin);
    bool detect_spoke = (value < threshold);
    if (!rpm.on_spoke && detect_spoke)
    { // rising edge of spoke detection
      if (detect_spoke)
        rpm.spoke_count += 1;
    }
    rpm.on_spoke = detect_spoke;
  }

  if (interrupt_fired)
  {
    for (size_t i = 0; i < (sizeof(rpm_states) / sizeof(*rpm_states)); i++)
    {
      auto& rpm = rpm_states[i];
      Serial.println(
        String(i) + ": value: " + String(analogRead(rpm.pin)) + ", count: " +
        String(rpm.spoke_count) + ", RPS: " + String(rpm.spoke_count));
      rpm.spoke_count = 0;
    }
    interrupt_fired = false;
  }
}

ISR(TIMER1_COMPA_vect)
{
  interrupt_fired = true;
}
