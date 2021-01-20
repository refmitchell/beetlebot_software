/**
   @file wind_cue_detector.cpp
   @brief Compute a Cue from the available wind sensor information.

   Subscribes to the wind_direction and wind_speed topics (from bb_sensors)
   and formats this information into a bb_util::Cue for the cue manager.
*/

#include "ros/ros.h"
#include "std_msgs/Float64.h"

#include <sstream>
#include <cmath>

#include "bb_util/bb_util.h"
#include "bb_util/argparse.h"
#include "bb_util/cue.hpp"
#include "bb_util/cue_msg.h"

//Global, bad but should be safe in this case
argparse::ArgumentParser parser("Parser");

// The wind cue representation, global so that it can be
// modified by a callback function and published.
bb_util::Cue wind_cue = bb_util::Cue("wind", 1, 0, 0);

bool initParser(argparse::ArgumentParser &parser, int argc, char **argv){
  //Global, all bb_computation nodes should have these options.
  parser.add_argument()
    .names({"-sd", "--subscribe_direction"})
    .description("Set the subscription topic for wind direction.")
    .required(false);

    parser.add_argument()
    .names({"-ss", "--subscribe_speed"})
    .description("Set the subscription topic for wind speed.")
    .required(false);

  parser.add_argument()
    .names({"-p", "--publish"})
    .description("Set the publication topic.")
    .required(false);

  parser.add_argument()
    .names({"-n", "--name"})
    .description("Set the node name.")
    .required(false);

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

void directionUpdateCallback(const std_msgs::Float64::ConstPtr& msg){
  // Convert direction to radians.
  double wind_direction = msg->data * bb_util::defs::PI / 180;
  wind_cue.setAzimuth(wind_direction);
}

void speedUpdateCallback(const std_msgs::Float64::ConstPtr& msg){
  // Cap wind speed at 100deg/sec (this needs tuned to the sensor in use)
  double wind_speed = msg->data < 100 ? msg->data : 100;
  wind_speed = wind_speed / 100; // Scale to be between 0 and 1
  wind_cue.setReliability(wind_speed);
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
  std::string pub_topic =
    parser.exists("publish") ?
    parser.get<std::string>("publish") :
    "wind_cue";

  // subscribe to
  std::string direction_sub_topic =
    parser.exists("subscribe_direction") ?
    parser.get<std::string>("subscribe_direction") :
    "wind_direction" ;

  std::string speed_sub_topic =
    parser.exists("subscribe_speed") ?
    parser.get<std::string>("subscribe_speed") :
    "wind_speed" ;

  // name
  std::string node_name =
    parser.exists("name") ?
    parser.get<std::string>("name") :
    "wind_detection_node";

  ROS_INFO("Node information: ");
  ROS_INFO("Node name: %s", node_name.c_str());
  ROS_INFO("Subscribing to: %s", direction_sub_topic.c_str());
  ROS_INFO("Subscribing to: %s", speed_sub_topic.c_str());
  ROS_INFO("Publishing to: %s", pub_topic.c_str());

  // ROS init
  ros::init(argc, argv, node_name.c_str());
  ros::NodeHandle n;

  ros::Publisher cue_pub =
    n.advertise<bb_util::cue_msg>(pub_topic.c_str(), 1000);

  ros::Subscriber speed_sub =
    n.subscribe(speed_sub_topic.c_str(), 1000, speedUpdateCallback);

  ros::Subscriber direction_sub =
    n.subscribe(direction_sub_topic.c_str(), 1000, directionUpdateCallback);

  while(ros::ok()){
    ros::spinOnce();
    cue_pub.publish(bb_util::Cue::toMsg(wind_cue));
  }

  return 0;
}
