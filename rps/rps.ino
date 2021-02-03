#include "pid.h"
#include "ring_buffer.h"
#include "rps_sensor.h"

#include <Arduino.h>
#include <Servo.h>
#include <ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt8.h>

ros::NodeHandle node;

std_msgs::Float32 mcu_rps_msg;
ros::Publisher mcu_rps("mcu/rps", &mcu_rps_msg);

std_msgs::String mcu_dbg_msg;
ros::Publisher mcu_dbg("mcu/dbg", &mcu_dbg_msg);

void mcu_velocity_cb(std_msgs::UInt8 const&);
ros::Subscriber<std_msgs::UInt8> mcu_velocity("mcu/velocity", &mcu_velocity_cb);

static size_t constexpr throttle_pin = 8;
Servo throttle;

RPSSensors rps(A0, A8);
PID pid(0.1, 0.0, 0.0);
float rps_setpoint = 0.0;

void mcu_velocity_cb(std_msgs::UInt8 const& msg)
{
  // TODO: msg should be in MPS[0, ?], not RPS[0, 38]
  rps_setpoint = (float)msg.data;
}

void adjust_throttle(int16_t rps_adjust)
{
  static_assert(sizeof(size_t) > 1);
  static int16_t constexpr neutral_deg = 60;

  static int16_t throttle_position_deg = 0;
  // Note that no effort is made to convert adjustment RPS to degrees.
  throttle_position_deg += rps_adjust;
  throttle_position_deg = constrain(throttle_position_deg, -60, 60);

  static char buf[16];
  sprintf(buf, "throttle: %+d", throttle_position_deg);
  mcu_dbg_msg.data = buf;
  mcu_dbg.publish(&mcu_dbg_msg);

  size_t position_deg = throttle_position_deg + neutral_deg;
  throttle.write(position_deg);
}

void setup()
{
  throttle.attach(throttle_pin);

  node.initNode();
  node.advertise(mcu_rps);
  node.advertise(mcu_dbg);
  node.subscribe(mcu_velocity);
}

void loop()
{
  bool rps_update = rps.update();
  if (!rps_update)
    return;

  Array<float, 2> rps_values = rps.values();
  float avg_rps = (rps_values[0] + rps_values[1]) / 2;
  mcu_rps_msg.data = avg_rps;
  mcu_rps.publish(&mcu_rps_msg);

  int16_t rps_adjust = (int16_t)round(pid.update(avg_rps, rps_setpoint));

  static char buf[16];
  sprintf(buf, "rps_adjust: %+d", rps_adjust);
  mcu_dbg_msg.data = buf;
  mcu_dbg.publish(&mcu_dbg_msg);

  adjust_throttle(rps_adjust);

  node.spinOnce();
}
