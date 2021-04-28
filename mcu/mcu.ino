#include "imu.hh"
#include "pid.hh"
#include "ros_node.hh"
#include "rps_sensor.hh"
#include "timer_interrupt.hh"

#include <Arduino.h>
#include <Servo.h>

RPSSensor rps{A0, A1};
IMU imu;
Servo steering;
Servo throttle;
PID pid{1.8, 0.03, 0.0};

static void set_steering(int16_t);
static void set_throttle(int16_t);

static int16_t throttle_setpoint = 0;
ROSNode node{[](auto const& serialized) {
  ROSNode::Ctl msg{serialized};
  throttle_setpoint = (int16_t)msg.throttle;
  set_steering(msg.steering);
}};

static inline void set_steering(int16_t setpoint)
{
  static constexpr int16_t steering_neutral = 90;
  static constexpr int16_t steering_cap = 40;
  setpoint = constrain(setpoint, -steering_cap, steering_cap);
  steering.write(steering_neutral + setpoint);
}

static inline void set_throttle(int16_t adjust)
{
  static constexpr int16_t throttle_neutral = 90;
  static constexpr int16_t throttle_cap = 30;
  adjust = constrain(adjust, -throttle_cap, throttle_cap);
  throttle.write(throttle_neutral + adjust);
}

void setup()
{
  setup_timer_interrupt();

  steering.attach(6);
  set_steering(0);

  throttle.attach(8);
  set_throttle(0);

  node.init();
  imu.init();

  while (!node.connected())
  {
    node.spin_once();
    delay(500);
  }

  // while (imu.status() != ICM_20948_Stat_Ok)
  // {
  //   node.log<ROSNode::Log::ERROR>("IMU error: %s", imu.status_str());
  //   delay(500);
  // }
}

static inline constexpr uint16_t first_decimal(float value)
{
  return (uint16_t)round((value - floor(value)) * 10);
}

void update_64hz()
{
  int16_t adjust = 0;
  if (throttle_setpoint == 0)
    adjust = (int16_t)pid.update(rps.value(), (float)throttle_setpoint);
  else
    adjust = throttle_setpoint;

  static unsigned long prev_ms = 0;
  node.log(
    "RPS: %d %d %d.%u",
    throttle_setpoint,
    adjust,
    (int16_t)rps.value(),
    first_decimal(rps.value()));

  set_throttle(adjust);

  imu.update();
  if (imu.ready())
  {
    auto const acc = imu.accelerometer_data();
    auto const mag = imu.magnetometer_data();
    auto const gyr = imu.gyroscope_data();
    // node.log(
    //   "%d,%d.%u,%d,%d.%u,%d.%u,%d.%u,%d.%u,%d.%u,%d.%u,%d.%u",
    //   throttle_setpoint,
    //   (int16_t)rps.value(),
    //   first_decimal(rps.value()),
    //   adjust,
    //   (int16_t)acc[0],
    //   first_decimal(acc[0]),
    //   (int16_t)acc[1],
    //   first_decimal(acc[1]),
    //   (int16_t)acc[2],
    //   first_decimal(acc[2]),
    //   (int16_t)mag[0],
    //   first_decimal(mag[0]),
    //   (int16_t)mag[1],
    //   first_decimal(mag[1]),
    //   (int16_t)mag[2],
    //   first_decimal(mag[2]),
    //   (int16_t)gyr[2],
    //   first_decimal(gyr[2]));
  }
}

void loop()
{
  node.spin_once();

  bool const timer_fired = timer_interrupt_flag;
  if (timer_fired)
    timer_interrupt_flag = false;

  rps.update(timer_fired);

  if (timer_fired)
    update_64hz();
}
