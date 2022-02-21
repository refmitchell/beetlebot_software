#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import Float64MultiArray

def talker():
    pub = rospy.Publisher('arm_control', Float64MultiArray, queue_size=10)
    rospy.init_node('controller', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        joint_message = Float64MultiArray()
        joint_message.data = [0,2,2,2,2,2]
        pub.publish(joint_message)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
