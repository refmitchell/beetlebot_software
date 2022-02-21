#!/usr/bin/env python
import rospy
from numpy import maximum,minimum
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from time import time, sleep

joint_state_sub = None   # Receive joint state feedback from the robot
joint_control_sub = None # Receive joint control messages from external sources
joint_control_pub = None # Send sanitised joint control messages to the robot

current_target_position = [0, 0, 0, 0, 0, 0] # Default start position

def clean_joint_states(data):
    """
    Sanitise target joint states to avoid damage to hardware.
    Do not change these limits.
    """
    lower_limits = [0, -1.57, -1.57, -1.57, -1.57,   -1]
    upper_limits = [0,  1.57,  1.57,  1.57,  1.57, 1.57]
    clean_lower = maximum(lower_limits,data)
    clean_upper = minimum(clean_lower,upper_limits)
    return list(clean_upper)

def joint_control_callback(control_msg):
    """
    Receives external target joint position from /joint_control, sanitises
    the joint positions and then sends them to the hardware listener,
    /joint_trajectory_point.
    """
    target = list(control_msg.data)

    # Joint Position vector should contain 6 elements:
    # [0, shoulder1, shoulder2, elbow, wrist, gripper]
    joint_pos = Float64MultiArray()
    joint_pos.data = clean_joint_states(target)
    print(joint_pos.data)
    joint_control_pub.publish(joint_pos)

if __name__ == '__main__':
    """
    Listen on /joint_control until either roscore or node shut down.
    """
    rospy.init_node('arm_control_node',anonymous=True)
    joint_control_sub = rospy.Subscriber("arm_control", Float64MultiArray, joint_control_callback)
    joint_control_pub = rospy.Publisher("joint_trajectory_point",Float64MultiArray, queue_size =10)

    rospy.spin()

