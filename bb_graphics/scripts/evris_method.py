#!/usr/bin/python
import rosbag
from std_msgs.msg import Float64, Float64MultiArray
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d
import argparse


def ring2sph(I, axis=-1):
    """
    -Evripidis Gkanias-
    Creates the spherical coordinates given an array of responses (population code).
    Parameters
    ----------
    I: np.ndarray[float]
        the array of responses representing spherical coordinates.
    axis: int
        the axis to apply the calculations on.
    Returns
    -------
    np.ndarray[float]
        theta, phi - the spherical coordinates calculated using the input population codes.
    """
    fund_freq = np.fft.fft(I, axis=axis)[1]
    phi = (np.pi - np.angle(np.conj(fund_freq))) % (2 * np.pi) - np.pi
    theta = np.absolute(fund_freq)
    return np.array([theta, phi])


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

ops = []
pos = []
r_pols = []
for idx in range(len(po0)):
    current = [
        po0[idx],
        po1[idx],
        po2[idx],
        po3[idx]
    ]
    print(current)
    r_horiz = [ x[2] for x in current ]
    r_vert = [ x[3] for x in current ]

    pol_op = np.array(np.log(r_horiz) - np.log(r_vert))
    pol_po = np.array(np.log(r_horiz) + np.log(r_vert))

    ops.append(pol_op)
    pos.append(pol_po)
    r_pols.append(pol_op / pol_po)

r_po0 = [ x[0] for x in pos ]
r_po1 = [ x[1] for x in pos ]
r0 = [x[0] for x in r_pols]
r1 = [x[1] for x in r_pols]
r2 = [x[2] for x in r_pols]
r3 = [x[3] for x in r_pols]
r = [sum(x) for x in r_pols]

phis = [0, 90, 180, 270] # pol neuron angles
#phis = [0, 135, 225, 270]# asymmetric pol neuron angles
sol_prefs = [0, 45, 90, 135, 180, 225, 270, 315] # sol preferred angles
phis = [np.radians(x) for x in phis]
alphas = [x - np.radians(90) for x in phis]
sol_prefs = [np.radians(x) for x in sol_prefs]

n_sols = 8
r_sols = np.zeros((n_sols, len(po0)))

for idx in range(len(po0)): # For each timestep
    rs = r_pols[idx]
    for j in range(n_sols): # For each sol neuron
        # Compute total inputs from connected pol neurons
        r_sols[j, idx] = sum([np.sin(alpha - sol_prefs[j]) * r for (r, alpha) in zip(rs, alphas)])

complex_out = []
for idx in range(len(po0)):
    outsum = 0
    for jdx in range(n_sols):
        r_sol = r_sols[jdx, idx]
        component =  np.exp(-(2j*np.pi))
        outsum += r_sol * np.exp(-(2j*np.pi*(jdx))/n_sols)

    complex_out.append(outsum)

#print(complex_out)
angles = [ np.angle(x.conjugate()) for x in complex_out ]
#angles = [ (x - angles[0]) for x in angles]

# for i in range(n_sols):
#     # if i < 2:
#     #     for j in range(len(r_sols[i,:])):
#     #         r_sols[i,j] *= -1

#     print(r_sols[i,:])
#     plt.plot(r_sols[i, :], alpha=0.5)

theta, phi = ring2sph(r_sols, axis=0)
phi = (-phi) % (2*np.pi) - np.pi
phi = (yaws[0]-phi[0]+phi-np.pi) % (2*np.pi) - np.pi

plt.imshow(r_sols, cmap="hot", vmin=-.03, vmax=.03, aspect="auto")

plt.figure("decoded")
plt.plot(phi, label="phi")
#plt.plot(theta, label="theta")
plt.plot(yaws, label="IMU Yaw")
plt.legend()


#plt.plot(complex_out, alpha=0.2)

#plt.plot((angles))
#plt.plot(yaws)

plt.show()

