#include "imu.hh"
#include "pid.hh"
#include "ros_node.hh"
#include "rps_sensor.hh"
#include "timer_interrupt.hh"

#include <Arduino.h>
#include <Servo.h>

RPSSensor rps{A0, A1, timer_interrupt_flag};
IMU imu;
Servo steering;
Servo throttle;
PID pid{1.8, 0.03, 0.0};

static int16_t throttle_setpoint = 0;
ROSNode node{[](auto const& serialized) {
  ROSNode::Ctl msg{serialized};

  throttle_setpoint = (int16_t)msg.throttle;

  static int16_t steering_setpoint = 0;
  static constexpr int16_t steering_neutral = 90;
  static constexpr int16_t steering_cap = 40;
  if (msg.steering != steering_setpoint)
  {
    steering_setpoint = constrain(msg.steering, -steering_cap, steering_cap);
    steering.write(steering_neutral + steering_setpoint);
  }
}};

void setup()
{
  setup_timer_interrupt();
  steering.attach(6);
  throttle.attach(8);
  node.init();
  imu.init();

  while (!node.connected())
  {
    node.spin_once();
    delay(500);
  }

  while (imu.status() != ICM_20948_Stat_Ok)
  {
    node.log<ROSNode::Log::ERROR>("IMU error: %s", imu.status_str());
    delay(500);
  }
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

  static constexpr int16_t throttle_cap = 30;

  int16_t adjust = 0;
  if (throttle_setpoint == 0)
    adjust = (int16_t)pid.update(rps.value(), (float)throttle_setpoint);
  else
    adjust = throttle_setpoint;

  adjust = constrain(adjust, -throttle_cap, throttle_cap);

  static constexpr int16_t throttle_neutral = 90;
  throttle.write(throttle_neutral + adjust);

  node.log(
    "RPS: %d, %d.%u, %d",
    throttle_setpoint,
    (int16_t)rps.value(),
    first_decimal(rps.value()),
    adjust);

  imu.update();
  if (imu.ready())
  {
    auto const acc = imu.accelerometer_data();
    node.log(
      "IMU: %d.%d, %d.%d",
      (int16_t)acc[0],
      first_decimal(acc[0]),
      (int16_t)acc[1],
      first_decimal(acc[1]));
  }
}
