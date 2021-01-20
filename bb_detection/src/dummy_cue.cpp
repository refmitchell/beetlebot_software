/**
   @file dummy_cue.cpp
   @brief Provdes dummy input signals which can be used by the robot.
   @author Robert Mitchell

   Designed for testing purposes. Cues can be synthesised with a
   magnitude, direction, and "sensitivity". This node is designed
   to be simple and general, we can have any number of them running
   on the network at any time (so long as they are uniquely named).
 */


#include <ros/ros.h>

#include <sstream>

#include "bb_util/bb_util.h"
#include "bb_util/cue.hpp"
#include "bb_util/cue_msg.h"
#include "bb_util/argparse.h"

argparse::ArgumentParser parser = argparse::ArgumentParser("Parser");

/**
   Initialises the argument parser defined in bb_util/argparse.h
   @param parser The argparse::ArgumentParser object.
   @param argc Passthrough argc from the command line.
   @param argv Passthrough argv from the command line.
*/
bool initParser(argparse::ArgumentParser &parser, int argc, char **argv){
  parser.add_argument()
    .names({"-t", "--type"})
    .description("Set the dummy cue type")
    .required(false);

  parser.add_argument()
    .names({"-s", "--sensitivity"})
    .description("Set the agent's sensitivity to this cue.")
    .required(false);

  parser.add_argument()
    .names({"-m", "--magnitude"})
    .description("Set the \"strength\" of the dummy cue.")
    .required(true);

  parser.add_argument()
    .names({"-a", "--angle"})
    .description("Set the angle for this cue (in degrees).")
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

int main(int argc, char **argv){
  // Initialise the parser
  if (!initParser(parser, argc, argv)) return -1;

  if (parser.exists("help")){
    parser.print_help();
    return 0;
  }

  //
  // Parser decode
  //
  std::string type =
    parser.exists("type") ?
    parser.get<std::string>("type") :
    "cue";

  double sensitivity =
    parser.exists("sensitivity") ?
    parser.get<double>("sensitivity") :
    1;

  double magnitude = parser.get<double>("magnitude");
  double angle = parser.get<double>("angle");
  angle = angle * (3.14159/180); // Deg to adian conversion: a * pi/180

  // Cue object defined by the arguments
  bb_util::Cue cue(type, sensitivity, magnitude, angle);

  std::stringstream name;
  std::stringstream topic;

  name << "dummy_cue_node_" << type;
  topic  << "dummy_cue_" << type;

  std::string namestring = name.str();
  std::string topicstring = topic.str();

  //
  // ROS initialisation and admin
  //
  ros::init(argc, argv, name.str());
  ros::NodeHandle n;
  ros::Publisher pub = n.advertise<bb_util::cue_msg>(topic.str(), 1000);

  ROS_INFO("\nDummy cue specification:"
           "Angle: %lf\nMagnitude: %lf\nSensitivity: %lf\nTopic: %s\nName: %s\n",
           angle,
           magnitude,
           sensitivity,
           name.str().c_str(),
           topic.str().c_str()
           );

  ros::Rate r(10);  // Publish the cue message at 10hz

  //
  // Publish loop, note this was chosen instead of
  // ros::spin() so that we can add cue updates in-place
  // later on.
  //
  while(ros::ok()){
    pub.publish(bb_util::Cue::toMsg(cue));
    ros::spinOnce();
    r.sleep();
  }

  return 0;
}
