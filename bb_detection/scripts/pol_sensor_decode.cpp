/**
   @file pol_sensor_decode.cpp
   @brief Decode the signal from the SkyCompass sensor
*/

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Twist.h>

#include <sstream>
#include <cmath>
#include <iostream>
#include <chrono>
#include <thread>
#include <cstdint>

#include "bb_util/bb_util.h"
#include "bb_util/argparse.h"
#include "bb_util/locomotion_cmd.h"

// Eigen linear algebra library: https://eigen.tuxfamily.com
#include "bb_util/Eigen/Eigen"

#define N_POL_OPS 4

argparse::ArgumentParser parser("Parser");
rosbag::Bag bag; // Rosbag for recording
ros::Publisher cmd_publisher;

std::vector<int> global_sensor_data; // Current sensor data (sign-extended)
double global_yaw; // Current yaw (from odom)
float angular_distance = 0;
bool yaw_data_received = false;
bool pol_data_received = false;

std::vector<std::vector<int>> pol_op_responses;

inline double radians(double degrees){return degrees*bb_util::defs::PI / 180;}
inline double degrees(double radians){return 180*radians/bb_util::defs::PI;}

bool initParser(argparse::ArgumentParser &parser, int argc, char **argv){
  parser.add_argument()
    .names({"-t", "--time"})
    .description("Duration (seconds) of bookends. If not specified, "
                 "bookends (pre and post-rotation sensor data recording) "
                 "are disabled")
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

//
// POL-OP callbacks; brittle but simple and quick.
//
void polCallback0(const std_msgs::Int32MultiArray::ConstPtr& pol_msg){
  pol_op_responses[0] = pol_msg.data;
}
void polCallback1(const std_msgs::Int32MultiArray::ConstPtr& pol_msg){
  pol_op_responses[1] = pol_msg.data;
}
void polCallback2(const std_msgs::Int32MultiArray::ConstPtr& pol_msg){
  pol_op_responses[2] = pol_msg.data;
}
void polCallback3(const std_msgs::Int32MultiArray::ConstPtr& pol_msg){
  pol_op_responses[3] = pol_msg.data;
}

int main(int argc, char **argv){
  if (!initParser(parser, argc, argv)) return -1;
  if (parser.exists("help")){
    parser.print_help();
    return 0;
  }

  // Initialise all pol_op datastructure
  for (int i = 0; i < N_POL_OPS; i++){
    std::vector<int> pd_responses = {0,0,0,0};
    pol_op_responses.push_back(pd_responses);
  }

  std::vector<double> sensor_angles = {0,90,180,270};
  double sensor_elevation = 45;

  // ROS init
  ros::init(argc, argv, node_name.c_str());
  ros::NodeHandle n;
  ros::Subscriber po0_sub = n.subscribe("pol_op_0", 1000, polCallback0);
  ros::Subscriber po1_sub = n.subscribe("pol_op_1", 1000, polCallback1);
  ros::Subscriber po2_sub = n.subscribe("pol_op_2", 1000, polCallback2);
  ros::Subscriber po0_sub = n.subscribe("pol_op_3", 1000, polCallback3);

  while (ros::ok()){
    ros::spinOnce(); // Get new polarisation data

    //
    // Decode...
    //

  }

  ROS_INFO("Exiting");
  return 0;
}
