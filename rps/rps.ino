#include "ring_buffer.h"
#include "rps_sensor.h"

#include <Arduino.h>

RPSSensor rps_bl(A0);
RPSSensor rps_br(A8);

void setup()
{
  Serial.begin(9600);
  RPSSensor::setup_timer2();
}

void loop()
{
  bool velocity_update = rps_bl.update();
  velocity_update |= rps_br.update();

  if (!velocity_update)
    return;

  static size_t divider = 0;
  divider++;
  if (divider < 4)
    return;

  divider = 0;

  long t_ms = millis();
  Serial.println(
    "t_ms: " + String(t_ms) + ", rps1: " + String(rps_bl.rps()) +
    ", rps2: " + String(rps_br.rps()));
}
