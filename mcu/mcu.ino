#include "rps_sensors.h"

#include <Arduino.h>
#include <ArduinoHardware.h>
#include <Servo.h>
#include <ros.h>
#include <ros/node_handle.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt8.h>

// template arguments: MAX_SUBSCRIBERS, MAX_PUBLISHERS, INPUT_SIZE, OUTPUT_SIZE
ros::NodeHandle_<ArduinoHardware, 1, 1, 128, 128> node;

std_msgs::String mcu_dbg_msg;
ros::Publisher mcu_dbg("mcu/dbg", &mcu_dbg_msg);

void mcu_ctl_cb(std_msgs::UInt8 const&);
ros::Subscriber<std_msgs::UInt8> mcu_ctl("mcu/ctl", &mcu_ctl_cb);

RPSSensors<2> rps({A0, A8});

Servo throttle;

static int16_t throttle_setpoint = 0.0;
void mcu_ctl_cb(std_msgs::UInt8 const& msg)
{
  throttle_setpoint = (int16_t)msg.data;

  static char buf[16];
  sprintf(buf, "ctl: %x", msg.data);
  mcu_dbg_msg.data = buf;
  mcu_dbg.publish(&mcu_dbg_msg);
}

void setup()
{
  node.initNode();
  node.advertise(mcu_dbg);
  node.subscribe(mcu_ctl);

  throttle.attach(8);
}

void loop()
{
  node.spinOnce();

  bool rps_update = rps.update();
  if (!rps_update)
    return;

  Array<float, 2> rps_values = rps.values();
  float avg_rps = (rps_values[0] + rps_values[1]) / 2.0;

  static char buf[32];
  sprintf(
    buf,
    "value: %d, %d.%d",
    throttle_setpoint,
    (int16_t)round(avg_rps),
    (int16_t)round((avg_rps - floor(avg_rps)) * 10));
  mcu_dbg_msg.data = buf;
  mcu_dbg.publish(&mcu_dbg_msg);

  static constexpr int16_t throttle_neutral = 90;
  throttle.write(throttle_neutral + throttle_setpoint);
}
