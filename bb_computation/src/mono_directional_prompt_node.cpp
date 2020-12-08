/**
   @file mono_directional_prompt_node.cpp
   @brief Compute single image-wide cue from cropped image.

   Available cues are currently brightest vector and centroid vector.
   Current relationship with IntensityCueManager and Cue classes is overly
   convoluted and should be simplified in the future.
*/

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <sstream>
#include <cmath>

#include "bb_computation/argparse.h"
#include "bb_computation/cue.h"
#include "bb_computation/icm.h"

#include "bb_util/bb_util.h"

//Global, bad but should be safe in this case
argparse::ArgumentParser parser("Parser");

bool initParser(argparse::ArgumentParser &parser, int argc, char **argv){
  //Global, all bb_computation nodes should have these options.
  parser.add_argument()
    .names({"-v", "--video"})
    .description("Enable video output for this node.")
    .required(false);

  parser.add_argument()
    .names({"-s", "--subscribe"})
    .description("Set the subscription topic.")
    .required(true);

  parser.add_argument()
    .names({"-p", "--publish"})
    .description("Set the publication topic.")
    .required(true);

  parser.add_argument()
    .names({"-n", "--name"})
    .description("Set the node name.")
    .required(true);

  // Node-specific
  parser.add_argument()
    .names({"-m", "--method"})
    .description("Define a method for determining a directional prompt.")
    .required(true);

  parser.enable_help();

  auto err = parser.parse(argc, const_cast<const char**>(argv));
  if (err) {
    std::stringstream ss;
    ss << err;
    ROS_ERROR("argparse error: %s", ss.str().c_str());
    return false;
  }

  return true;
}

int main(int argc, char **argv){
  // Parser initialisation
  if (!initParser(parser, argc, argv)) return -1;

  // Parser decode
  if (parser.exists("help")){
    parser.print_help();
    return 0;
  }

  // If parser option defined, use it, else default.
  // publish to
  std::string pub_topic = parser.get<std::string>("publish");

  // subscribe to
  std::string sub_topic = parser.get<std::string>("subscribe");

  // name
  std::string node_name = parser.get<std::string>("name");

  // method
  std::string cue = parser.get<std::string>("method");

  ROS_INFO("Node information: ");
  ROS_INFO("Node name: %s", node_name.c_str());
  ROS_INFO("Subscribing to: %s", sub_topic.c_str());
  ROS_INFO("Publishing to: %s", pub_topic.c_str());
  ROS_INFO("Cue: %s", cue.c_str());

  // ROS init
  ros::init(argc, argv, node_name.c_str());
  ros::NodeHandle n;

  IntensityCueManager icm(n,
                          sub_topic,
                          pub_topic,
                          cue,
                          parser.exists("video")
                          );

  ros::spin();


  return 0;
}
