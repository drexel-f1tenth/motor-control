#include "imu.h"
#include "ros_node.h"
#include "rps_sensors.h"

#include <Arduino.h>
#include <Servo.h>

RPSSensors<2> rps({A0, A8});
IMU imu(2);
Servo throttle;

static int16_t throttle_setpoint = 0.0;
ROSNode node([](auto const& msg) { throttle_setpoint = (int16_t)msg.data; });

void setup()
{
  throttle.attach(8);
  if (imu.status() != ICM_20948_Stat_Ok)
    node.debug("%s", imu.status_str());
}

void loop()
{
  node.spin_once();

  bool rps_update = rps.update();
  if (!rps_update)
    return;

  Array<float, 2> rps_values = rps.values();
  float avg_rps = (rps_values[0] + rps_values[1]) / 2.0;

  node.debug(
    "value: %d, %d.%d",
    throttle_setpoint,
    (int16_t)round(avg_rps),
    (int16_t)round((avg_rps - floor(avg_rps)) * 10));

  static constexpr int16_t throttle_neutral = 90;
  throttle.write(throttle_neutral + throttle_setpoint);
}
