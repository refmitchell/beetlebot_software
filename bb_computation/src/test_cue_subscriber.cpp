/**
  \file test_cue_subscriber.cpp
  \brief Orientate the robot w.r.t. an external cue.

  This node will allow the robot to turn on the spot in response
  to an external cue. This is primarily useful for testing but can
  be used for experiments in its own right (precision of response to
  a given cue, e.g. polarisation or wind where it's useful to
  try and characterise the reliability of the sensor).

  \note This node uses a simple proportional controller to steer the
  robot back to its target. This is sufficient for functionality
  testing but if you wanted to look at actual precision then a better
  controller would probably be warranted. No neural models are used
  for control here.

  \warning If bb_locomotion cmd_vel_service is running then starting
  this node will cause the robot to move.
*/

#include <ros/ros.h>
#include "std_msgs/String.h"

#include "bb_util/cue.hpp"
#include "bb_util/cue_msg.h"
#include "bb_util/velocity.h"
#include "bb_util/argparse.h"

#define GOAL_ANGLE 2.0
#define TRAVERSE_TIME 30

ros::ServiceClient client;

argparse::ArgumentParser parser("parser");

/**
   Initialise the argument parser
   \param parser The parser object
   \param argc The command-line argument count
   \param argv The command-line argument values
   \return true on success
*/
bool initParser(argparse::ArgumentParser &parser, int argc, char **argv){
  parser.add_argument()
    .names({"-s", "--subscribe"})
    .description("Set the subscription topic, must be of type bb_util/cue_msg")
    .required(true);

  parser.enable_help();

  auto err = parser.parse(argc, const_cast<const char**>(argv));
  if(err){
    std::stringstream ss;
    ss << err;
    ROS_ERROR("argparse error: %s", ss.str().c_str());
    return false;
  }

  return true;
}

/**
   The cue callback, will simply rotate the robot to a given
   orientation w.r.t. a cue.
   \param bb_util::cue_msg of any type
*/
void cvCallback(const bb_util::cue_msg::ConstPtr& cue_msg){
  ROS_INFO("Cue info: (R: %f, T: %f)",
           cue_msg->contrast,
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
  if (!initParser(parser, argc, argv)) return -1;
  if(parser.exists("help")){
    parser.print_help();
    return 0;
  }

  ros::init(argc, argv, "test_cue_subscriber");
  ros::NodeHandle n;

  ros::Subscriber sub =
    n.subscribe(parser.get<std::string>("subscribe").c_str(),
                1000,
                cvCallback);
  
  client = n.serviceClient<bb_util::velocity>(bb_util::defs::UPDATE_VELOCITY);

  ROS_INFO("Running...");

  ros::spin();
  return 0;
}
