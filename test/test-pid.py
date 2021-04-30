import rospy
from std_msgs.msg import UInt16

pub = rospy.Publisher('mcu/ctl', UInt16, queue_size=1)

rospy.init_node('test', anonymous=True)

rospy.sleep(rospy.Duration(1))
for setpoint in [6, 10, 20, 6, 0]:
  pub.publish(setpoint << 8)
  rospy.sleep(rospy.Duration(4))
