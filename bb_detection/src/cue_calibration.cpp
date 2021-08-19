/**
   @file cue_calibration.cpp
   @brief Calibrate detection nodes relative to magnetometer

   Used to calibrate the different supported cues (move them all into the same
   frame of reference as the magnetometer). Functionally this aligns the
   receptive fields of the TL neurons downstream. Unlike most nodes this is
   designed to run for one cycle then terminate; figure out the calibration
   parmeters, store them in the parameter server, then exit.
 */

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>
#include <tf/transform_datatypes.h>
#include <std_msgs/String.h>
#include <iostream>

#include "bb_util/bb_util.h"
#include "bb_util/cue.hpp"
#include "bb_util/argparse.h"

argparse::ArgumentParser parser("Parser");

double global_yaw = 0;
double wind_offset = 0;
double intensity_offset = 0;

bool initParser(argparse::ArgumentParser &parser, int argc, char **argv){
  parser.add_argument()
    .names({"-i", "--intensity_subscribe"})
    .description("Set the subscription topic for the light intensity cue."
                 "Should be whatever topic the intensity cue detector is "
                 "publishing to."
                 )
    .required(true); // Always has to be specified by the intensity detection
                     // node.

  parser.add_argument()
    .names({"-w", "--wind_subscribe"})
    .description("Set the wind cue subscription topic.")
    .required(false);

  parser.enable_help();

  auto err = parser.parse(argc, const_cast<const char**>(argv));
  if (err){
    std::stringstream ss;
    ss << err;
    ROS_ERROR("argparse error: %s", ss.str().c_str());
    return false;
  }

  return true;
}

void odomCallback(const nav_msgs::Odometry::ConstPtr& odom_msg){
  // odom.pose.orientation.z // Heading - quaternion comes into play

  // Conversion from Quaternion to RPY, lifted from ROS forums.
  tf::Quaternion quat;
  tf::quaternionMsgToTF(odom_msg->pose.pose.orientation, quat);

  double roll, pitch, yaw;
  tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
  global_yaw = yaw;
  ROS_INFO("Yaw: %f", yaw);
}

void intensityUpdateCallback(const bb_util::cue_msg::ConstPtr& msg){
  bb_util::cue_msg cue_msg = *msg;
  bb_util::Cue cue = bb_util::Cue::toCue(cue_msg);
  intensity_offset = global_yaw - cue.getTheta();
}

void windUpdateCallback(const bb_util::cue_msg::ConstPtr& msg){
  bb_util::cue_msg cue_msg = *msg;
  bb_util::Cue cue = bb_util::Cue::toCue(cue_msg);
  wind_offset = global_yaw - cue.getTheta();
}

int main(int argc, char**argv){
  // Determine the detection nodes for the various cues
  std::string wind_cue_sub_topic =
    parser.exists("wind_subscribe") ?
    parser.get<std::string>("wind_subscribe") :
    bb_util::defs::WIND_CUE_TOPIC;

  std::string intensity_cue_sub_topic =
    parser.exists("intensity_subscribe") ?
    parser.get<std::string>("intensity_subscribe") :
    bb_util::defs::INTENSITY_CUE_TOPIC;

  std::string node_name = "calibration";
  ros::init(argc, argv, node_name.c_str());
  ros::NodeHandle n;

  ros::Subscriber wind_direction_sub =
    n.subscribe(wind_cue_sub_topic.c_str(), 1000, windUpdateCallback);

  ros::Subscriber intensity_cue_sub =
    n.subscribe(intensity_cue_sub_topic.c_str(), 1000, intensityUpdateCallback);

  ros::Publisher calibration_notify =
    n.advertise<std_msgs::String>(bb_util::defs::CALIBRATION_NOTIFY_TOPIC,1000);

  while(ros::ok()){
    ros::spinOnce();
    n.setParam(bb_util::params::CALIBRATION_INTENSITY_OFFSET, intensity_offset);
    n.setParam(bb_util::params::CALIBRATION_WIND_OFFSET, wind_offset);
    std_msgs::String msg;
    msg.data = "";

    // Delay seems to be required for ros plumbing
    ros::Duration(0.1).sleep();

    calibration_notify.publish(msg);
    ROS_INFO("New calibration data stored; press enter to run again. To\n"
             "exit, use CTRL-C. To store calibration data, remember to use\n"
             "$: rosparam dump > params.txt.\n");
    std::cin.get(); // Wait for either enter or interrupt
  }

  return 0;
}
