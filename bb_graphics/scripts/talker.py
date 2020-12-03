#!/usr/bin/python

import rospy
from std_msgs.msg import String
from db_util.msg import test_population
import math

def rotate(array, n=1):
    return array[n:] + array[:n]

def talker():
    # Generate sin wave
    wave = [math.sin(math.radians(x)) for x in range(360)]
    pub = rospy.Publisher('chatter', test_population, queue_size=1)
    rospy.init_node('talker', anonymous=True)
    rate=rospy.Rate(100)
    rospy.loginfo("Broadcasting...")

    # The rotating sin wave is broadcast properly
    while not rospy.is_shutdown():
        msg = test_population()
        msg.tb1 = wave
        pub.publish(msg)
        wave = rotate(wave)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
