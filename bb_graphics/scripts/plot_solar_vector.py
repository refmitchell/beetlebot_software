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
messages = bag.read_messages(topics=['solar_vector', 'yaw'])

yaws=[]
sol_vecs=[]
time=[]

for topic, msg, t in messages:
    if topic == 'yaw':
        yaws.append(msg.data)
    elif topic == 'solar_vector':
        sol_vecs.append(msg.data)

bag.close()

# R, INCLINATION, AZIMUTH
sph_sol_vecs = [[1, np.arctan2(np.sqrt(x**2 + y**2), z), np.arctan2(y,x)] for [x,y,z] in sol_vecs]
xs = [ x for [x, _, _] in sol_vecs ]
ys = [ y for [_, y, _] in sol_vecs ]
zs = [ z for [_, _, z] in sol_vecs ]

# R, AZIMUTH
pol_sol_vecs = [[r, gamma] for [r, _, gamma] in sph_sol_vecs]
deg_gammas = [np.degrees(gamma) for [_, _, gamma] in sph_sol_vecs]

fig = plt.figure()
ax = plt.axes(projection='3d')
ax.scatter3D(xs, ys, zs)
plt.show()

