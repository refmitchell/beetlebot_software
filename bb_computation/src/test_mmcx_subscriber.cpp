
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

#include "bb_computation/mmcx_model.hpp"

#include "bb_util/velocity.h"
#include "bb_util/cue_list.h"
#include "bb_util/cue_msg.h"
#include "bb_util/cue.hpp"
#include "bb_util/vector.hpp"
#include "bb_util/vec2d_msg.h"
#include "bb_util/encoding_status.h"


#define GOAL_ANGLE -2.0
#define TRAVERSE_TIME 30

// Init CX
MMCX mmcx = MMCX(2);

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
  std::vector<bb_util::cue_msg> cues_from_msg = cue_list->cues;
  std::vector<bb_util::Cue> cues;

  for (int i = 0; i < cues_from_msg.size(); ++i){
    cues.push_back(bb_util::Cue::toCue(cues_from_msg[i]));}

  double CXMotor = mmcx.input(cues, 0);

  // For now, only update the angular velocity.
  bb_util::velocity vel_msg;
  vel_msg.request.angular = (CXMotor < 0) ? 0.5 : -0.5;
  vel_msg.request.linear = 0;
  client.call(vel_msg);

  // Retrieve cue encodings.
  // encodings[n-1] is the CL
  // encodings[n-2] is the "true" average of the TLs
  // The rest are the TL encodings
  std::vector<bb_util::Vec2D> encodings;
  bb_util::encoding_status encoding_msg = mmcx.get_cue_encoding(encodings);
  pub.publish(encoding_msg);
}

int main(int argc, char **argv){
  ros::init(argc, argv, "test_mmcx_subscriber");
  ros::NodeHandle n;

  // ROS networking

  // Get cue - Sub to visual cues
  ros::Subscriber sub = n.subscribe("cue_list", 1000, cue_list_callback);

  // Request velocity changes from velocity service
  client = n.serviceClient<bb_util::velocity>("update_velocity");

  // Set up publisher for CX
  pub = n.advertise<bb_util::encoding_status>("mmcx_encoding_list", 1);

  ROS_INFO("Running...");

  ros::spin();

  return 0;
}
