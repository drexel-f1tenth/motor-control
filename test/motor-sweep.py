#!/usr/bin/env python3

# Warning: The car will reach full throttle during this test. The car should be
# propped up such that the wheels are not making contact with any surface or
# wires.

import rospy
from std_msgs.msg import String
from std_msgs.msg import UInt8

STEERING = 0
THROTTLE = 1

def motor_control_msg(motor, position):
  assert (motor == STEERING) or (motor == THROTTLE)
  assert position < (1 << 7)
  return (motor << 7) | (position)

rospy.init_node("mcu_test")
pub = rospy.Publisher("mcu", UInt8, queue_size=3)

for pos in range(20, 100 + 1, 5):
  pub.publish(data=motor_control_msg(STEERING, pos))
  rospy.sleep(0.8)
  pub.publish(data=motor_control_msg(STEERING, 60))
  rospy.sleep(0.4)

for pos in range(0, 120 + 1, 10):
  pub.publish(data=motor_control_msg(THROTTLE, pos))
  rospy.sleep(0.8)
  pub.publish(data=motor_control_msg(THROTTLE, 60))
  rospy.sleep(0.4)
