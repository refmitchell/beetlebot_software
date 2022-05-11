#!/usr/bin/python
import rosbag
from std_msgs.msg import Float64, Float64MultiArray
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d
import argparse

parser = argparse.ArgumentParser(description="Plot polarisation sensor output")
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
messages = bag.read_messages(topics=['solar_vector',
                                     'yaw'])
                                     # 'pol_op_0',
                                     # 'pol_op_1',
                                     # 'pol_op_2',
                                     # 'pol_op_3'
#])

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
    elif topic == 'solar_vector':
        sol_vecs.append(msg.data)
    elif topic == 'pol_op_0':
        po0.append(msg.data)
    elif topic == 'pol_op_1':
        po1.append(msg.data)
    elif topic == 'pol_op_2':
        po2.append(msg.data)
    elif topic == 'pol_op_3':
        po3.append(msg.data)

bag.close()

# R, INCLINATION, AZIMUTH

sol_vecs = sol_vecs[1:]
print(sol_vecs)

# sph_sol_vecs = [[1, np.arctan2(np.sqrt(x**2 + y**2), z), np.arctan2(y,x)] for [x,y,z] in sol_vecs]
xs = [ x for (x, _, _) in sol_vecs ]
ys = [ y for (_, y, _) in sol_vecs ]
zs = [ z for (_, _, z) in sol_vecs ]

origins = list(np.zeros(len(xs)))

# # R, AZIMUTH
pol_sol_vecs = [ (1, np.arctan2(y,x)) for (x,y) in zip(xs, ys) ]
gammas = [ x for (_,x) in pol_sol_vecs ]


# fig = plt.figure()
# ax = plt.axes(projection='3d')
# scale=3
# ax.quiver(origins, origins, origins, xs, ys, origins,
#           arrow_length_ratio=0
# )
plt.plot(gammas)
plt.plot(yaws)
plt.show()
