#!/usr/bin/python

#
# Imports
#
import rospy
import numpy as np
from matplotlib import pyplot as plt
from matplotlib.animation import FuncAnimation

from bb_util.msg import vec2d_msg, encoding_status

#
# Init global figures
#

fig = plt.figure()
polar_plot_ax = plt.subplot(111, projection='polar')#, polar=True)

#
# Know x ranges and limits in advance,
# similarly know all Y values will be between 0 and 1
#
wind_x = 0
wind_y = 0
light_x = 0
light_y = 0

cl1_x = 0
cl1_y = 0
# true_cl_x = 0
# true_cl_y = 0

#
# Line2Ds for blitting
#
wind_ln, = polar_plot_ax.plot([], [], 'bo')
light_ln, = polar_plot_ax.plot([], [], 'go')
cl1_ln, = polar_plot_ax.plot([], [], 'ro')
true_cl_ln, = polar_plot_ax.plot([], [], 'orange')


# fig.tight_layout()

# # Required for blitting, iterable of all Artists which were updated
# # on the animation callback.
status_ln = [wind_ln, light_ln, cl1_ln]#, true_cl_ln]

#
# Update ydata based on incoming messages
#
def ros_callback(data):
    global wind_x
    global wind_y
    global light_x
    global light_y
    global cl1_x
    global cl1_y
    # global true_cl_x
    # global true_cl_y

    scale = 1

    wind_x = data.tls[0].theta * scale
    wind_y = 0.5#data.tls[0].r * scale

    light_x = data.tls[1].theta * scale
    light_y = 0.5#  data.tls[1].r * scale

    cl1_x = data.cl1.theta * scale
    cl1_y = 0.8 #data.cl1.r * scale

    # true_cl_x = data.true_cl1.x * scale
    # true_cl_y = data.true_cl1.y * scale

    print("wind_x: " + str(wind_x))
    print("wind_y: " + str(wind_y))
    print("wind_r: " + str(data.tls[0].r))
    print("wind_t: " + str(data.tls[0].theta))
    print("----------------------------------------")

#
# Animation initialisation
#
def animation_init():
#    polar_plot_ax.set_xlim(0, 1)
    polar_plot_ax.set_ylim(0, 1)
    polar_plot_ax.set_title("Integration Status");

    return status_ln

#
# For animation loop
#
def animation_callback(frames):
    global wind_x
    global wind_y
    global light_x
    global light_y
    global cl1_x
    global cl1_y
    # global true_cl_x
    # global true_cl_y

    status_ln[0].set_data(wind_x, wind_y)
    status_ln[1].set_data(light_x, light_y)
    status_ln[2].set_data(cl1_x, cl1_y)
    # status_ln[3].set_data(true_cl_x, true_cl_y)

    return status_ln

if __name__ == '__main__':
    rospy.init_node('integration_status_listener', anonymous=True)
    rospy.Subscriber("mmcx_encoding_list", encoding_status, ros_callback)

    ani = FuncAnimation(fig,
                        animation_callback,
                        init_func=animation_init,
                        blit=True,
                        interval=10)

    plt.show()
    rospy.spin()


