#include "imu.hh"
#include "pid.hh"
#include "ros_node.hh"
#include "rps_sensor.hh"
#include "timer_interrupt.hh"

#include <Arduino.h>
#include <Servo.h>

RPSSensor rps{A0, A8, timer_interrupt_flag};
IMU imu;
Servo throttle;
PID pid{1.4, 0.03, 0.0};

static int16_t throttle_setpoint = 0.0;
ROSNode node{[](auto const& msg) { throttle_setpoint = (int16_t)msg.data; }};

void setup()
{
  setup_timer_interrupt();
  throttle.attach(8);
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

static inline constexpr uint16_t first_decimal(float value)
{
  return (uint16_t)round((value - floor(value)) * 10);
}

void loop()
{
  node.spin_once();

  bool const rps_update = rps.update();
  if (!rps_update)
    return;

  // node.log("IMU: %d, %s", imu.ready(), imu.status_str());

  static constexpr int16_t throttle_cap = 30;
  int16_t adjust = (int16_t)pid.update(rps.value(), (float)throttle_setpoint);
  adjust = constrain(adjust, -throttle_cap, throttle_cap);

  node.log(
    "RPS: %d, %d.%u, %d",
    throttle_setpoint,
    (int16_t)rps.value(),
    first_decimal(rps.value()),
    adjust);

  static constexpr int16_t throttle_neutral = 90;
  throttle.write(throttle_neutral + adjust);
}
