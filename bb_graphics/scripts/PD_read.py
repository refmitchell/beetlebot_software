#!/usr/bin/python

import rosbag
from std_msgs.msg import Float64, UInt32MultiArray
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

#
# Extract raw data
#
bag = rosbag.Bag('1.bag')
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
#        print("Yaw_t: {}".format(int(t.to_nsec()/1000)))
    elif topic == 'pol_op_0':
        print("data:old - {}:{}".format(msg.data, pol_op_data))
        if list(msg.data) != pol_op_data:
            time = int(t.to_nsec()/1000000)
            duration = time  - start_time
            start_time = time

            print("duration = {}".format(duration))
            print("count = {}".format(counter))
            pol_op_data = list(msg.data)
            counter = 0

        pols.append(msg.data)
        # print("Pol_r: {}".format(msg.data))
        # print("Pol_t: {}".format(int(t.to_nsec()/1000000)))
        counter+=1
        #print("Pol_t: {}".format(t.to_sec())

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
    df[lbl] = df[lbl] * 244.14e-9 # Jan's voltage conversion
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

plt.subplot(611)
plt.plot(df["pd1"])
plt.subplot(612)
plt.plot(df["pd2"])
plt.subplot(613)
plt.plot(df["pd3"])
plt.subplot(614)
plt.plot(df["pd4"])
plt.subplot(615)
plt.plot(df["yaw"])
plt.subplot(616)
plt.plot(df["angle"])
plt.show()

#plt.plot(angle)
#plt.plot(df["angle"])
#plt.plot(avg)

print(df)
