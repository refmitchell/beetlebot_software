/*
  test_cue_subscriber.cpp
  Test module to subscribe to the output of a cue-detection node
  (v = [theta, magnitude]). This module will attempt to maintain
  a goal angle by interacting with the locomotion package.
*/

#include <ros/ros.h>
#include "std_msgs/String.h"

#include "bb_computation/icm.h"
#include "bb_computation/cue_vector.h"

#include "bb_util/cue.hpp"
#include "bb_util/cue_msg.h"

#include "bb_util/velocity.h"

#define GOAL_ANGLE 2.0
#define TRAVERSE_TIME 30

ros::ServiceClient client;

// Init CX
//CentralComplex cx;

void cvCallback(const bb_util::cue_msg::ConstPtr& cue_msg){
  ROS_INFO("Cue info: (R: %f, T: %f)",
           cue_msg->reliability,
           cue_msg->theta);

  float current = cue_msg->theta;
  float error = GOAL_ANGLE - current;
  float correction_velocity = -0.09*error;

  bb_util::velocity msg;

  msg.request.angular = correction_velocity;

  msg.request.linear = 0.0;
  client.call(msg);
}

int main(int argc, char **argv){
  ros::init(argc, argv, "test_cue_subscriber");
  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("wind_cue", 1000, cvCallback);
  client = n.serviceClient<bb_util::velocity>("update_velocity");

  ROS_INFO("Running...");

  ros::spin();
  return 0;
}
