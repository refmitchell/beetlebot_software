#!/usr/bin/python

# See listerner.py and talker.py for templates

#
# Imports
#
import rospy
import numpy as np
from matplotlib import pyplot as plt
from matplotlib.animation import FuncAnimation

from db_util.msg import vmcx_activity


#
# Init global figures
#

# Plotting layers (Viewing these as bar charts with linked top)
# Ultimately need to pick something, can ask Barbara if it's okay,
# given that we have rate populations it might be best to graph those.

# 16 CL1
# 16 TL2
# 8 TB1
# 16 CPU4
# 16 CPU1
# 16 VM
# 16 active (CPU4 + weighted VM)

# Also, what about the "memory" layer? Think this was just a copy of CPU4

fig, (tb1_ax, cpu4_ax, vm_ax, active_ax, cpu1_ax) = plt.subplots(5,1)

#
# Know x ranges and limits in advance,
# similarly know all Y values will be between 0 and 1
#

tb1_xdata = range(8)
tb1_ydata = []

cpu4_xdata = range(16)
cpu4_ydata = []

vm_xdata = range(16)
vm_ydata = []

active_xdata = range(16)
active_ydata = []

cpu1_xdata = range(16)
cpu1_ydata = []

cpu1_lft_ydata = 0
cpu1_rgt_ydata = 0

xdata = range(360)
ydata = []

#
# Line2Ds for blitting
#

tb1_ln, = tb1_ax.plot([], [], 'g')
cpu4_ln, = cpu4_ax.plot([], [], 'red')
vm_ln, = vm_ax.plot([], [], 'yellow')
active_ln, = active_ax.plot([], [], 'orange')

cpu1_labels = ['CPU1 Left', 'CPU1 Right']
cpu1_bar = cpu1_ax.bar(cpu1_labels, [1,1])
cpu1_lft = cpu1_bar.patches[0]
cpu1_rgt = cpu1_bar.patches[1]

fig.tight_layout()

# tl2_ln, = tl2_ax.bar([1], [1], color='r')
# cl1_ln, = cl1_ax.bar([1], [1], color='r')
# tb1_ln, = tb1_ax.bar([1], [1], color='r')
# cpu4_ln, = cpu4_ax.bar([1], [1], color='r')
# cpu1_ln, = cpu1_ax.bar([1], [1], color='r')

circuit_ln = [tb1_ln, cpu4_ln, vm_ln, active_ln, cpu1_lft, cpu1_rgt]

#
# Update ydata based on incoming messages
#
def ros_callback(data):
    global vm_ydata
    global active_ydata
    global tb1_ydata
    global cpu4_ydata
    global cpu1_ydata
    global cpu1_lft_ydata
    global cpu1_rgt_ydata

    vm_ydata = data.vm
    active_ydata = data.active
    tb1_ydata = data.tb1
    cpu4_ydata = data.cpu4
    cpu1_ydata = data.cpu1
    cpu1_lft_ydata = sum(data.cpu1[:8]) / sum(data.cpu1)
    cpu1_rgt_ydata = sum(data.cpu1[8:]) / sum(data.cpu1)

    print(cpu1_lft_ydata)
    print(cpu1_rgt_ydata)

#
# Animation initialisation
#
def animation_init():
    vm_ax.set_xlim(1, 16)
    vm_ax.set_ylim(-0.6, 0.6)
    vm_ax.set_xticks(range(1,17))
    vm_ax.set_title("VM Activity")

    active_ax.set_xlim(1, 16)
    active_ax.set_ylim(0, 1.2)
    active_ax.set_xticks(range(1,17))
    active_ax.set_title("ACTIVE Activity")

    tb1_ax.set_xlim(1, 8)
    tb1_ax.set_ylim(0 ,1.2)
    tb1_ax.set_xticks(range(1,9))
    tb1_ax.set_title("TB1 Activity")

    cpu4_ax.set_xlim(1, 16)
    cpu4_ax.set_ylim(0, 1.2)
    cpu4_ax.set_xticks(range(1,17))
    cpu4_ax.set_title("CPU4 Activity")

    cpu1_ax.set_ylim(0, 1)
    cpu1_ax.set_title("CPU1 Activity")

    return circuit_ln

#
# For animation loop
#
def animation_callback(frames):
    global vm_ydata
    global vm_xdata
    global active_ydata
    global active_xdata
    global tb1_ydata
    global tb1_xdata
    global cpu4_ydata
    global cpu4_xdata
    global cpu1_ydata
    global cpu1_xdata

#    rospy.loginfo("animation_callback")

    vm_xdata = range(1, 1 + len(vm_ydata))
    active_xdata = range(1, 1 + len(active_ydata))
    tb1_xdata = range(1, 1 + len(tb1_ydata))
    cpu4_xdata = range(1, 1 + len(cpu4_ydata))
    cpu1_xdata = range(1, 1 + len(cpu1_ydata))

    circuit_ln[0].set_data(tb1_xdata, tb1_ydata)
    circuit_ln[1].set_data(cpu4_xdata, cpu4_ydata)
    circuit_ln[2].set_data(vm_xdata, vm_ydata)
    circuit_ln[3].set_data(active_xdata, active_ydata)
    circuit_ln[4].set_height(cpu1_lft_ydata)
    circuit_ln[5].set_height(cpu1_rgt_ydata)

    return circuit_ln

if __name__ == '__main__':
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("vmcx_status", vmcx_activity, ros_callback)

    ani = FuncAnimation(fig,
                        animation_callback,
                        init_func=animation_init,
                        blit=True,
                        interval=10)

    plt.show()
    rospy.spin()


