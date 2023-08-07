#!/usr/bin/python

#
# Imports
#
import rospy
import numpy as np
import matplotlib.animation as animation
from matplotlib import pyplot as plt
from bb_util.msg import erm_status

#
# Init global figures
#

mosaic = [
    ["r1_epg", "r1_epg", "r1", "r2"],
    ["r1_epg", "r1_epg", "epg", "epg"],
    ["blank", "blank", "d7", "d7"],
    ["r2_epg", "r2_epg", "peg", "peg"],
    ["r2_epg", "r2_epg", "pen", "pen"],
]

fig, ax_dict = plt.subplot_mosaic(mosaic)

ax_dict["blank"].axis('off')

init_image = np.ones((8,8))
r1_epg_artist = ax_dict["r1_epg"].imshow(init_image,animated=True, vmin=0, vmax=0.25)
r2_epg_artist = ax_dict["r2_epg"].imshow(init_image,animated=True, vmin=0, vmax=0.25)

r1_artist, = ax_dict["r1"].plot([],[], 'tab:red')
r2_artist, = ax_dict["r2"].plot([],[], 'tab:blue')
epg_artist, = ax_dict["epg"].plot([],[], 'tab:purple')
peg_artist, = ax_dict["peg"].plot([],[], 'hotpink')
pen_artist, = ax_dict["pen"].plot([],[], 'tab:green')
d7_artist, = ax_dict["d7"].plot([],[], 'darkorange')
animation_artists = [r1_epg_artist,
                     r2_epg_artist,
                     r1_artist,
                     r2_artist,
                     epg_artist,
                     d7_artist,
                     pen_artist,
                     peg_artist
]

fig.tight_layout()

# Data structures
w_r1_epg = init_image
w_r2_epg = init_image

r1_rates = []
r2_rates = []
epg_rates = []
d7_rates = []
pen_rates = []
peg_rates = []

n_r1 = 8
n_r2 = 8
n_epg = 8

# Listen on erm_status topic and update underlying data structures
def ros_callback(msg):
    global w_r1_epg
    global w_r2_epg
    global n_r1
    global n_r2
    global n_epg
    global r1_rates
    global r2_rates
    global epg_rates
    global d7_rates
    global pen_rates
    global peg_rates

    # ROS parameter server would probably be better for these...
    n_r1 = msg.n_r1
    n_r2 = msg.n_r2
    n_epg = msg.n_epg

    w_r1_epg = np.reshape(msg.r1_epg, (n_r1,n_epg))
    w_r2_epg = np.reshape(msg.r2_epg, (n_r2,n_epg))

    r1_rates = msg.r1
    r2_rates = msg.r2
    epg_rates = msg.epg
    d7_rates = msg.d7
    pen_rates = msg.pen
    peg_rates = msg.peg

def animation_init():
    global n_r1
    global n_r2
    global n_epg

    ax_dict["r1_epg"].set_xlim(0,n_r1 - 1)
    ax_dict["r1_epg"].set_ylim(0,n_epg - 1)
    ax_dict["r1_epg"].set_title(r"R1 $\rightarrow$ E-PG")

    ax_dict["r2_epg"].set_xlim(0,n_r2 - 1)
    ax_dict["r2_epg"].set_ylim(0,n_epg - 1)
    ax_dict["r2_epg"].set_title(r"R2 $\rightarrow$ E-PG")

    ax_dict["r1"].set_xlim(0, n_r1 - 1)
    ax_dict["r1"].set_ylim(0, 1)
    ax_dict["r1"].set_title("R1 population")

    ax_dict["r2"].set_xlim(0, n_r2 - 1)
    ax_dict["r2"].set_ylim(0, 1)
    ax_dict["r2"].set_title("R2 population")

    ax_dict["epg"].set_xlim(0, n_epg - 1)
    ax_dict["epg"].set_ylim(0, 1)
    ax_dict["epg"].set_title("EPG population")

    ax_dict["d7"].set_xlim(0, 7)
    ax_dict["d7"].set_ylim(0, 1)
    ax_dict["d7"].set_title("D7 population")

    ax_dict["pen"].set_xlim(0, 15)
    ax_dict["pen"].set_ylim(0, 1)
    ax_dict["pen"].set_title("PEN population")

    ax_dict["peg"].set_xlim(0, 15)
    ax_dict["peg"].set_ylim(0, 1)
    ax_dict["peg"].set_title("PEG population")

    fig.tight_layout()

    return animation_artists

def animation_callback(frames):
    global w_r1_epg
    global w_r2_epg
    global r1_rates
    global r2_rates
    global epg_rates
    global d7_rates
    global pen_rates
    global peg_rates

    r1_xdata = range(len(r1_rates))
    r2_xdata = range(len(r2_rates))
    epg_xdata = range(len(epg_rates))
    d7_xdata = range(len(d7_rates))
    pen_xdata = range(len(pen_rates))
    peg_xdata = range(len(peg_rates))

    animation_artists[0].set_data(w_r1_epg)
    animation_artists[1].set_data(w_r2_epg)
    animation_artists[2].set_data(r1_xdata,r1_rates)
    animation_artists[3].set_data(r2_xdata,r2_rates)
    animation_artists[4].set_data(epg_xdata,epg_rates)
    animation_artists[5].set_data(d7_xdata,d7_rates)
    animation_artists[6].set_data(pen_xdata,pen_rates)
    animation_artists[7].set_data(peg_xdata,peg_rates)

    return animation_artists

if __name__ == "__main__":
    rospy.init_node('erm_vis')
    rospy.Subscriber('erm_status', erm_status, ros_callback)

    layer_ani = animation.FuncAnimation(fig,
                                        animation_callback,
                                        init_func=animation_init,
                                        blit=True,
                                        interval=10)

    # mapping_ani = animation.ArtistAnimation(fig,
    #                                         mapping_artists,
    #                                         blit=True,
    #                                         interval=10)

    plt.show()
    rospy.spin()
