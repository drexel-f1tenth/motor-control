#include "include/rps_sensors.h"

#include <Arduino.h>
#include <ArduinoHardware.h>
#include <Servo.h>
#include <ros.h>
#include <ros/node_handle.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt8.h>

// template arguments: MAX_SUBSCRIBERS, MAX_PUBLISHERS, INPUT_SIZE, OUTPUT_SIZE
ros::NodeHandle_<ArduinoHardware, 1, 1, 64, 64> node;

std_msgs::String mcu_dbg_msg;
ros::Publisher mcu_dbg("mcu/dbg", &mcu_dbg_msg);

void mcu_rps_cb(std_msgs::UInt8 const&);
ros::Subscriber<std_msgs::UInt8> mcu_rps("mcu/rps", &mcu_rps_cb);

RPSSensors<2> rps({A0, A8});

Servo throttle{8};

void mcu_rps_cb(std_msgs::UInt8 const& msg)
{
  // TODO
}

void setup()
{
  node.initNode();
  node.advertise(mcu_dbg);
  node.subscribe(mcu_rps);

  RPSSensors<2>::setup_timer_interrupt();
}

void loop()
{
  node.spinOnce();
}
