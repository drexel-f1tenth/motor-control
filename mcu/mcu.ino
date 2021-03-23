#include "imu.h"
#include "pid.h"
#include "ros_node.h"
#include "rps_sensors.h"

#include <Arduino.h>
#include <Servo.h>

RPSSensors<2> rps({A0, A8});
IMU imu;
Servo throttle;
PID pid(1.0, 0.0, 0.0);

static int16_t throttle_setpoint = 0.0;
ROSNode node([](auto const& msg) { throttle_setpoint = (int16_t)msg.data; });

void setup()
{
  throttle.attach(8);
  rps.init();
  node.init();
  imu.init(53);

  while (!node.connected())
  {
    node.spin_once();
    delay(500);
  }

  if (imu.status() != ICM_20948_Stat_Ok)
    node.log<ROSNode::Log::ERROR>("IMU error: %s", imu.status_str());
}

uint16_t first_decimal(float value)
{
  return (uint16_t)round((value - floor(value)) * 10);
}

void loop()
{
  node.spin_once();

  bool const rps_update = rps.update();
  if (!rps_update)
    return;

  node.log("IMU: %d, %s", imu.ready(), imu.status_str());

  Array<float, 2> const rps_values = rps.values();
  float const avg_rps = (rps_values[0] + rps_values[1]) / 2.0;

  static constexpr int16_t throttle_cap = 30;
  int16_t adjust = (int16_t)pid.update(avg_rps, (float)throttle_setpoint);
  adjust = constrain(adjust, -throttle_cap, throttle_cap);

  node.log(
    "RPS: %d, %d.%u, %d",
    throttle_setpoint,
    (int16_t)round(avg_rps),
    first_decimal(avg_rps),
    adjust);

  static constexpr int16_t throttle_neutral = 90;
  throttle.write(throttle_neutral + adjust);
}
