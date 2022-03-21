#!/usr/bin/python

import rosbag
from std_msgs.msg import Float64, UInt32MultiArray
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import argparse

parser = argparse.ArgumentParser(description="Plot polarisation data")
parser.add_argument("-b", "--bagfile", type=str,
                    help="ROS Bagfile to process")
parser.add_argument("-f", "--filename", type=str,
                    help="Figure filename")
parser.add_argument("-t", "--title", type=str,
                    help="Figure title")
parser.add_argument("-s", "--show", action='store_true',
                    help="Show the figure.")

args = parser.parse_args()

#
# Set args with defaults
#
bagfile = "1.bag" if args.bagfile == None else args.bagfile
filename = "fig.pdf" if args.filename == None else args.filename + ".pdf"
title = "Photodiode readings (1-4, odom, angle)" if args.title == None else args.title

#
# Extract raw data
#
bag = rosbag.Bag(bagfile)
messages = bag.read_messages(topics=['pol_op_0', 'yaw'])

yaws = []
pols = []
time = []

counter = 0
pol_op_data = [0,0,0,0]
start_time = 0
for topic, msg, t in messages:
    if topic == 'yaw':
        yaws.append(msg.data)
    elif topic == 'pol_op_0':
        # Timer check
        if list(msg.data) != pol_op_data:
            time = int(t.to_nsec()/1000000)
            duration = time  - start_time
            start_time = time
            pol_op_data = list(msg.data)
            counter = 0

        pols.append(msg.data)
        counter+=1


bag.close()


#
# Construct df representation
#
cols = ["yaw", "pd1", "pd2", "pd3", "pd4"]
pd_1 = [x for (x,_,_,_) in pols]
pd_2 = [x for (_,x,_,_) in pols]
pd_3 = [x for (_,_,x,_) in pols]
pd_4 = [x for (_,_,_,x) in pols]
pds = [pd_1, pd_2, pd_3, pd_4]

df = pd.DataFrame(columns=cols)
df["yaw"] = yaws

for i in range(len(pds)): # Lazy
    l = i+1
    lbl = "pd{}".format(l)
    df[lbl] = pds[i]
#    df[lbl] = df[lbl] * 244.14e-9 # Jan's voltage conversion
#
# Stokes parameters - Optics of life version
# Need to double check PD arrangement
#
Q = df["pd3"] - df["pd4"]
U = df["pd2"] - df["pd1"]
avg = df["pd1"] + df["pd2"] + df["pd3"] + df["pd4"]
avg = avg/4

p_linear = np.sqrt(Q*Q + U*U) / avg
angle = np.arctan2(U, Q) / 2

df["Q"] = Q
df["U"] = U
df["p_linear"] = p_linear
df["angle"] = angle # np.arctan2((df["pd4"] + df["pd1"] - 2*df["pd3"]),(df["pd4"] - df["pd1"])) / 2#angle

fig, axs = plt.subplots(6,1, sharex=True)
fig.set_size_inches(4,8)
axs[0].plot(df["pd1"])
axs[0].set_title(title)
axs[1].plot(df["pd2"])
axs[2].plot(df["pd3"])
axs[3].plot(df["pd4"])
axs[4].plot(df["yaw"])
axs[5].plot(df["angle"])
plt.savefig(filename, bbox_inches="tight")
if args.show:
    print(df)
    plt.show()
