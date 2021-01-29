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
  Serial.begin(9600);
}

void loop()
{
  node.spinOnce();
  bool velocity_update = rps.update();
  if (!velocity_update)
    return;

  Array<float, 2> rps_values = rps.rps();
  mcu_rps_msg.data = rps_values[0];
  node.publish(&mcu_rps_msg);
  mcu_rps_msg.data = rps_values[1];
  node.publish(&mcu_rps_msg);
}
