#!/usr/bin/python
import rosbag
from std_msgs.msg import Float64, Float64MultiArray
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d
import argparse

parser = argparse.ArgumentParser(description="Decode and polarisation sensor output")
parser.add_argument("-b", "--bagfile", type=str,
                    help="ROS Bagfile to process")
parser.add_argument("-f", "--filename", type=str,
                    help="Figure filename")
parser.add_argument("-t", "--title", type=str, help="Figure title")
parser.add_argument("-s", "--show", action='store_true',
                    help="Show the figure generated")

args = parser.parse_args()

bagfile = "s_1.bag" if args.bagfile == None else args.bagfile
filename = "fig.pdf" if args.filename == None else args.filename + ".pdf"
title = "Photodiode readings (1-4, odom, angle)" if args.title == None else args.title

# Read from specified bagfile and divide topics into their own datastructures.
bag = rosbag.Bag(bagfile)
messages = bag.read_messages(topics=['yaw',
                                     'pol_op_0',
                                     'pol_op_1',
                                     'pol_op_2',
                                     'pol_op_3'
])

yaws=[]
sol_vecs=[]
po0 = []
po1 = []
po2 = []
po3 = []
time=[]

for topic, msg, t in messages:
    if topic == 'yaw':
        yaws.append(msg.data)
    elif topic == 'pol_op_0':
        po0.append(msg.data)
    elif topic == 'pol_op_1':
        po1.append(msg.data)
    elif topic == 'pol_op_2':
        po2.append(msg.data)
    elif topic == 'pol_op_3':
        po3.append(msg.data)

bag.close()

aops = []
for idx in range(len(po0)):
    current = [
        po0[idx],
        po1[idx],
        po2[idx],
        po3[idx]
    ]

    Q = [t - f for [_, _, t, f] in current]
    U = [t - o for [o, t, _, _] in current]

    # AoP for each pol_op
    aop = [np.arctan2(u, q)/2 for (u,q) in zip(U,Q)]
    p_linear = [np.sqrt(q*q + u*u) for (u,q) in zip(U,Q)]
    aops.append((aop,p_linear)) # list of AOPS for each pol_op

# Work out solar vector for each timestep
sensor_angles = [180, 270, 0, 90]
sensor_angles = [np.radians(x) for x in sensor_angles]
gamma = np.radians(45) # Zenith angle, I think inclination

C1 = [ np.array([
    [np.cos(alpha), -np.sin(alpha),0],
    [np.sin(alpha), np.cos(alpha), 0],
    [0, 0, 1]
]) for alpha in sensor_angles ]

C2 = np.array([
    [np.cos(gamma), 0, np.sin(gamma)],
    [0,1,0],
    [-np.sin(gamma), 0, np.cos(gamma)]
])

# C for each pol-op is known in advance
C = [np.dot(x, C2) for x in C1]
sols = []
for (aop, p_linear) in aops:
    PE = [np.array([np.cos(a), np.sin(a), 0]) for a in aop]
    PE = [np.reshape(x, (3,1)) for x in PE]
    e = [c @ pe for (c, pe) in zip(C, PE)]



    E = np.zeros((3, len(aop))) # Always 3x#Pixels

    # E should be a concatenation of column vectors
    for idx in range(len(e)):
        E[:,idx] = np.reshape(e[idx], (3,))

    # print(E)
    EET = E @ E.T
    w, v = np.linalg.eig(EET)
    s = v[np.argmin(w)]#np.argmin(w)] # Solar vector
    s = s.reshape((3, 1))
    alpha_s = np.arctan2(s[1], s[0])

    sols.append(alpha_s)

aop0 = [x for ([x,_,_,_],_) in aops]
aop1 = [x for ([_,x,_,_],_) in aops]
aop2 = [x for ([_,_,x,_],_) in aops]
aop3 = [x for ([_,_,_,x],_) in aops]

p_linear0 = [x for (_, [x,_,_,_]) in aops]
p_linear1 = [x for (_, [_,x,_,_]) in aops]
p_linear2 = [x for (_, [_,_,x,_]) in aops]
p_linear3 = [x for (_, [_,_,_,x]) in aops]

#plt.plot([s % np.pi for s in sols])
plt.plot(sols)
# plt.plot(p_linear0)
# plt.plot(p_linear1)
# plt.plot(p_linear2)
# plt.plot(p_linear3)
plt.plot(yaws)
# plt.plot(aop0, label="aop0")
# plt.plot(aop1, label="aop1")
# plt.plot(aop2, label="aop2")
# plt.plot(aop3, label="aop3")
plt.legend()
plt.savefig(filename)

