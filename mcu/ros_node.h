#pragma once

#include <Arduino.h>
#include <ArduinoHardware.h>
#include <ros.h>
#include <ros/node_handle.h>
#include <std_msgs/UInt8.h>

class ROSNode
{
public:
  static constexpr char const* control_topic = "mcu/ctl";

  using Log = rosserial_msgs::Log;
  using Msg = std_msgs::UInt8;
  using Callback = void (*)(Msg const&);

private:
  // Explicitly restrict memory utilization from NodeHandle.
  // template args: MAX_SUBSCRIBERS, MAX_PUBLISHERS, INPUT_SIZE, OUTPUT_SIZE
  ros::NodeHandle_<ArduinoHardware, 1, 0, 128, 128> _node;
  ros::Subscriber<std_msgs::UInt8> _mcu_ctl;
  char _msg_buf[32];

public:
  ROSNode(Callback cb) : _mcu_ctl{control_topic, cb} {}

  inline void init()
  {
    _node.initNode();
    _node.subscribe(_mcu_ctl);
  }

  inline bool connected()
  {
    return _node.connected();
  }

  inline void spin_once()
  {
    _node.spinOnce();
  }

  template<Log::_level_type level = Log::INFO>
  inline void log(char const* fmt, ...)
  {
    Log msg;
    msg.level = level;
    msg.msg = _msg_buf;
    va_list args;
    va_start(args, fmt);
    vsnprintf(_msg_buf, sizeof(_msg_buf), fmt, args);
    _node.publish(rosserial_msgs::TopicInfo::ID_LOG, &msg);
  }
};
