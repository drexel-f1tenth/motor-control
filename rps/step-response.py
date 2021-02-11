#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from std_msgs.msg import UInt8
import time

def dbg_cb(msg):
  print(msg)

rospy.init_node("step_response")
pub = rospy.Publisher("/mcu/velocity", UInt8, queue_size=3)
sub = rospy.Subscriber("/mcu/dbg", String, dbg_cb)

pub.publish(data=0)
rospy.Timer(rospy.Duration(3), lambda _: pub.publish(data=10), oneshot=True)
rospy.Timer(rospy.Duration(8), lambda _: pub.publish(data=20), oneshot=True)
rospy.Timer(rospy.Duration(13), lambda _: pub.publish(data=0), oneshot=True)
rospy.sleep(15)
