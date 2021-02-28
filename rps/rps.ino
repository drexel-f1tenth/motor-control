#include "pid.h"
#include "ring_buffer.h"
#include "rps_sensor.h"

#include <Arduino.h>
#include <Servo.h>
#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt8.h>

ros::NodeHandle node;

std_msgs::String mcu_dbg_msg;
ros::Publisher mcu_dbg("mcu/dbg", &mcu_dbg_msg);

void mcu_rps_cb(std_msgs::UInt8 const&);
ros::Subscriber<std_msgs::UInt8> mcu_rps("mcu/rps", &mcu_rps_cb);

static size_t constexpr throttle_pin = 8;
Servo throttle;

static constexpr float throttle_cap = 25.0;

RPSSensors rps(A0, A8);
PID pid(1.30, 0.01, 0.00);

float rps_setpoint = 0.0;

void mcu_rps_cb(std_msgs::UInt8 const& msg)
{
  // TODO: msg should be in MPS[0, ?], not RPS[0, 38]
  rps_setpoint = (float)msg.data;
}

void adjust_throttle(int16_t rps_adjust)
{
  static int16_t constexpr neutral_deg = 90;
  int16_t pos = neutral_deg + rps_adjust;
  throttle.write(pos);
}

void setup()
{
  throttle.attach(throttle_pin);

  node.initNode();
  node.advertise(mcu_dbg);
  node.subscribe(mcu_rps);
}

void loop()
{
  bool rps_update = rps.update();
  if (!rps_update)
    return;

  Array<float, 2> rps_values = rps.values();
  float avg_rps = (rps_values[0] + rps_values[1]) / 2;

  int16_t rps_adjust = (int16_t)round(pid.update(avg_rps, rps_setpoint));
  rps_adjust = constrain(rps_adjust, -throttle_cap, throttle_cap);

  static char buf[16];
  sprintf(
    buf,
    "rps: %+d %+d %+d",
    (int16_t)round(avg_rps),
    (int16_t)round(rps_setpoint),
    rps_adjust);
  mcu_dbg_msg.data = buf;
  mcu_dbg.publish(&mcu_dbg_msg);

  adjust_throttle(rps_adjust);

  node.spinOnce();
}
