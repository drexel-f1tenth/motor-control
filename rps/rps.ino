#include "pid.h"
#include "ring_buffer.h"
#include "rps_sensor.h"

#include <Arduino.h>
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

RPSSensors rps(A0, A8);
PID pid(0.0, 40.0, 0.1, 0.1, 0.1);
float rps_setpoint = 0.0;

void setup()
{
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

  float err = pid.update(avg_rps, rps_setpoint);
  char buf[16];
  // char* p = dtostrf(err, 7, 2, buf);
  sprintf(buf, "%d", (int)round(err));
  // *(p + 10) = '\0';
  // mcu_dbg_msg.data = "yup";
  // strcpy(buf, "yup");
  mcu_dbg_msg.data = buf;
  mcu_dbg.publish(&mcu_dbg_msg);

  node.spinOnce();
}

void mcu_velocity_cb(std_msgs::UInt8 const& msg)
{
  rps_setpoint = (float)msg.data;

  // sprintf(mcu_dbg_msg.data, "setpoint: %d", msg.data);
  // mcu_dbg.publish(&mcu_dbg_msg);
}
