#include "ring_buffer.h"
#include "rps_sensor.h"

#include <Arduino.h>
#include <ros.h>
#include <std_msgs/Float32.h>

ros::NodeHandle node;
std_msgs::Float32 mcu_rps_msg;
ros::Publisher mcu_rps("mcu/rps", &mcu_rps_msg);

RPSSensors rps(A0, A8);

void setup()
{
  node.initNode();
  node.advertise(mcu_rps);
}

void loop()
{
  bool velocity_update = rps.update();
  if (!velocity_update)
    return;

  Array<float, 2> rps_values = rps.values();
  mcu_rps_msg.data = rps_values[1];
  mcu_rps.publish(&mcu_rps_msg);

  node.spinOnce();
}
