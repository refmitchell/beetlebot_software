#!/usr/bin/python

# Possible solution:
# https://answers.ros.org/question/264767/plotting-real-time-data/
# Want to try animations

#
# Imports
#
import rospy
import numpy as np
from std_msgs.msg import String
from bb_util.msg import test_population
from matplotlib import pyplot as plt
from matplotlib.animation import FuncAnimation

#
# Init global figures
#
#fig, ax = plt.subplots()
#fig = plt.figure()
fig, (ax1, ax2) = plt.subplots(2,1)

xdata = range(360)
ydata = []
ln1, = ax1.plot([], [], 'r')
ln2, = ax2.plot([], [], 'go')
ln = [ln1, ln2]

#
# Update ydata based on incoming messages
#
def ros_callback(data):
    global ydata
    rospy.loginfo("ros_callback")
    ydata = data.tb1
#    print("ydatalen: " + str(len(ydata)))

#
# Animation initialisation
#
def animation_init():
    ax1.set_xlim(0,360)
    ax1.set_ylim(-1.1, 1.1)
    ax2.set_xlim(0,360)
    ax2.set_ylim(-2, 2)
    return ln

#
# For animation loop
#
def animation_callback(frames):
    global xdata
    global ydata

    rospy.loginfo("animation_callback")
    xdata = range(len(ydata))
    ln[0].set_data(xdata, ydata)
    ln[1].set_data(xdata, ydata)
    return ln

if __name__ == '__main__':
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("chatter", test_population, ros_callback)

    ani = FuncAnimation(fig,
                        animation_callback,
                        init_func=animation_init,
                        blit=True,
                        interval=10)

    plt.show()
    rospy.spin()


