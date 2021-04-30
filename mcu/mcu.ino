#include "imu.hh"
// #include "pid.hh"
#include "ros_node.hh"
#include "rps_sensor.hh"
#include "throttle.hh"
#include "timer_interrupt.hh"

#include <Arduino.h>
#include <PID_v2.h>
#include <Servo.h>

static constexpr bool use_pid = true;
static constexpr bool use_pid_when_breaking = false;

RPSSensor rps{A0, A1};
IMU imu;
Servo steering;
Throttle throttle;
// PID pid{1.5, 0.04, 0.0};
PID_v2 pid{2.5, 0.2, 0.1, PID::Direct, PID::P_On::Error};

static inline void set_steering(int16_t setpoint)
{
  static constexpr int16_t steering_neutral = 90;
  static constexpr int16_t steering_cap = 40;
  setpoint = constrain(setpoint, -steering_cap, steering_cap);
  steering.write(steering_neutral + setpoint);
}

static int16_t throttle_setpoint = 0;
ROSNode node{[](auto const& serialized) {
  ROSNode::Ctl msg{serialized};
  throttle_setpoint = (int16_t)msg.throttle;
  pid.Setpoint((double)throttle_setpoint);
  set_steering(msg.steering);
}};

void setup()
{
  setup_timer_interrupt();

  steering.attach(6);
  set_steering(0);

  throttle.init(8);

  node.init();
  imu.init();

  pid.Start(0, 0, 0);

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
  throttle.update(rps.value() == 0.0);

  auto const adjust = [] {
    if constexpr (use_pid)
      return (int16_t)pid.Run(rps.value());
    else
      return (int16_t)throttle_setpoint;
  }();

  node.log(
    "RPS: %d %d %d.%u",
    throttle_setpoint,
    adjust,
    (int16_t)rps.value(),
    first_decimal(rps.value()));

  if (!use_pid_when_breaking && (throttle_setpoint == 0))
    throttle.apply_break();
  else
    throttle.set_position(adjust);

  // imu.update();
  // if (imu.ready())
  // {
  //   auto const acc = imu.accelerometer_data();
  //   auto const mag = imu.magnetometer_data();
  //   auto const gyr = imu.gyroscope_data();
  //   node.log(
  //     "%d,%d.%u,%d,%d.%u,%d.%u,%d.%u,%d.%u,%d.%u,%d.%u,%d.%u",
  //     throttle_setpoint,
  //     (int16_t)rps.value(),
  //     first_decimal(rps.value()),
  //     adjust,
  //     (int16_t)acc[0],
  //     first_decimal(acc[0]),
  //     (int16_t)acc[1],
  //     first_decimal(acc[1]),
  //     (int16_t)acc[2],
  //     first_decimal(acc[2]),
  //     (int16_t)mag[0],
  //     first_decimal(mag[0]),
  //     (int16_t)mag[1],
  //     first_decimal(mag[1]),
  //     (int16_t)mag[2],
  //     first_decimal(mag[2]),
  //     (int16_t)gyr[2],
  //     first_decimal(gyr[2]));
  // }
}

void loop()
{
  node.spin_once();

  // For most accuate results, unset timer_interrupt_flag as soon as it is read.
  bool const timer_fired = timer_interrupt_flag;
  if (timer_fired)
    timer_interrupt_flag = false;

  rps.update(timer_fired);

  if (timer_fired)
    update_64hz();
}
