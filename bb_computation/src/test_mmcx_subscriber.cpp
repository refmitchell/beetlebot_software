
/*
 test_mmcx_subscriber.cpp
 Node to test a limited multimodal cue integration implmenetation.
 Need to check my intuition.

*/

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>
#include <tf/transform_datatypes.h>
#include "std_msgs/String.h"

#include "bb_computation/icm.h"
#include "bb_computation/cue_vector.h"
#include "bb_computation/vm_command.h"
#include "bb_computation/cx_model.hpp"
#include "bb_computation/mmcx_model.hpp"

#include "bb_util/velocity.h"
#include "bb_util/vmcx_activity.h"
#include "bb_util/cue_list.h"
#include "bb_util/cue_msg.h"

#define GOAL_ANGLE -2.0
#define TRAVERSE_TIME 30

// Init CX
MMCX cx;

// Broadcast CX status
ros::Publisher pub;

// Send movement commands to the update_velocity service
ros::ServiceClient client;

// Server to accept VM commands because it seems like the most
// direct solution at the moment.
ros::ServiceServer vm_server;

// Weird wee function that can help clean the linear velocity component
// of the odometry information. There's a little noise which could cause
// issues and it's more straightforward for now just to remove it.
// This is by no means required. Didn't use it for the yaw because I
// don't think there's any advantage to cleaning that up.
inline double clean_velocity(double lin_vel, int factor=100){
  if (factor == 0) return 0;
  return ((double) ((int) (lin_vel * factor))) / factor;
}

//
// Publish for graphing
//
// void cx_status_publish(std::vector<std::vector<double>> &status){
//   bb_util::vmcx_activity msg;

//   msg.tl2 = status[0];
//   msg.cl1 = status[1];
//   msg.tb1 = status[2];
//   msg.cpu4 = status[3];
//   msg.mem = status[4];
//   msg.cpu1 = status[5];
//   msg.vm = status[6];
//   msg.active = status[7];
// 
//   pub.publish(msg);
// }


/**
 * Cue list update
 */
void cue_list_callback(const bb_util::cue_list::ConstPtr& cue_list){

}

int main(int argc, char **argv){
  ros::init(argc, argv, "test_mmcx_subscriber");
  ros::NodeHandle n;

  // ROS networking

  // Get cue - Sub to visual cues
  ros::Subscriber sub = n.subscribe("cue_list", 1000, cue_list_callback);

  // Request velocity changes from velocity service
  client = n.serviceClient<bb_util::velocity>("update_velocity");

  // Set up VM Server
  // vm_server = n.advertiseService("vm_management", manage_vm);

  // Set up publisher for CX
  //pub = n.advertise<bb_util::vmcx_activity>("vmcx_status", 1);

  ROS_INFO("Running...");

  ros::spin();

  return 0;
}
