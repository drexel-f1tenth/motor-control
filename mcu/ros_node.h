#pragma once

#include <Arduino.h>
#include <ArduinoHardware.h>
#include <ros.h>
#include <ros/node_handle.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt8.h>

class ROSNode
{
public:
  static constexpr char const* debug_topic = "mcu/dbg";
  static constexpr char const* control_topic = "mcu/ctl";

  using Msg = std_msgs::UInt8;
  using Callback = void (*)(Msg const&);

private:
  // template arguments: MAX_SUBSCRIBERS, MAX_PUBLISHERS, INPUT_SIZE,
  // OUTPUT_SIZE
  ros::NodeHandle_<ArduinoHardware, 1, 1, 128, 128> _node;
  std_msgs::String _mcu_dbg_msg;
  ros::Publisher _mcu_dbg{debug_topic, &_mcu_dbg_msg};
  ros::Subscriber<std_msgs::UInt8> _mcu_ctl;

public:
  ROSNode(Callback cb) : _mcu_ctl{control_topic, cb}
  {
    _node.initNode();
    _node.advertise(_mcu_dbg);
    _node.subscribe(_mcu_ctl);
  }

  inline void spin_once()
  {
    _node.spinOnce();
  }

  inline void debug(char const* fmt, ...)
  {
    va_list args;
    va_start(args, fmt);
    static char msg_buf[32];
    vsnprintf(msg_buf, sizeof(msg_buf), fmt, args);
    _mcu_dbg_msg.data = msg_buf;
    _mcu_dbg.publish(&_mcu_dbg_msg);
  }
};
