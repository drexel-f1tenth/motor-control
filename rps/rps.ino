#include "ring_buffer.h"
#include "rps_sensor.h"

#include <Arduino.h>

RPSSensors rps(A0, A8);

void setup()
{
  Serial.begin(9600);
}

void loop()
{
  bool velocity_update = rps.update();
  if (!velocity_update)
    return;
}
