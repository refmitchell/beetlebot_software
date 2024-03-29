/**
   \file pol_op_recording.cpp
   \brief Record from a single zenith-facing pol op unit.

   Rotate the robot on the spot and record the polarisation opponent
   response in the zenith alongside the yaw information.

   \warning The working status of this code is unknown.
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

argparse::ArgumentParser parser("Parser");
rosbag::Bag bag; // Rosbag for recording
ros::Publisher cmd_publisher;

std::vector<int> global_sensor_data; // Current sensor data (sign-extended)
double global_yaw; // Current yaw (from odom)
float angular_distance = 0;
bool yaw_data_received = false;
bool pol_data_received = false;

std::string pol_sub_topic = "pol_op_0";
std::string odom_sub_topic = "yaw";
std::string node_name = "pol_op_pd_test_recorder";

/**
   Clip values less than 0 to be 0.
   \param value The value to be clipped
   \return 0 if value less than 0, otherwise the original value.
*/
double clip(double value){
  return value >= 0 ? value : 0;
}


/**
   Construct and publish a command velocity command. 
   \param linear The goal linear velocity
   \param angular The goal angular velocity
*/
void command_velocity(float linear, float angular){
 geometry_msgs::Twist msg;

  msg.linear.x = linear;
  msg.linear.y = 0.0;
  msg.linear.z = 0.0;

  msg.angular.x = 0.0;
  msg.angular.y = 0.0;
  msg.angular.z = angular;

  cmd_publisher.publish(msg);
}


/**
   Initialise the argument parser
   \param parser The argument parser
   \param argc The argument count
   \param argv The array of argument values from the command line
   \return true on success
*/
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

/**
   Yaw topic callback.
   \param msg The yaw message (note, from bb_util yaw, not odometry)
*/
void yawCallback(const std_msgs::Float64::ConstPtr& yaw_msg){
  // Conversion from Quaternion to RPY, lifted from ROS forums.
  global_yaw = yaw_msg->data; // Update local memory

  yaw_data_received = true;
}

/**
   Polarisation opponent callback for readings from the unit.
   \param msg The photodiode readings from the unit.
*/
void polCallback(const std_msgs::Int32MultiArray::ConstPtr& pol_msg){
  global_sensor_data = pol_msg->data; // Update local memory

  pol_data_received = true;
}

int main(int argc, char **argv){
  // Parser initialisation
  if (!initParser(parser, argc, argv)) return -1;

  // Parser decode
  if (parser.exists("help")){
    parser.print_help();
    return 0;
  }

  int bookend =
    parser.exists("time") ?
    parser.get<float>("time") :
    0;

  auto bookend_time = std::chrono::seconds(bookend);

  // ROS init
  ros::init(argc, argv, node_name.c_str());
  ros::NodeHandle n;
  ros::Subscriber pol_sub =
    n.subscribe(pol_sub_topic,
                1000,
                polCallback);
  ros::Subscriber odom_sub =
    n.subscribe(odom_sub_topic,
                1000,
                yawCallback);
  cmd_publisher = n.advertise<geometry_msgs::Twist>("cmd_vel", 1000);

  // Wait for sensor data
  while(ros::ok() && !(yaw_data_received && pol_data_received)){
    ros::spinOnce();
    ROS_INFO("Awaiting pol/odom data. Start the sensor nodes.");
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }
  ROS_INFO("Data available.");
  ROS_INFO("Moving to start position.");

  // Move to zero on odometry
  double goal_yaw = 0;
  double error = global_yaw - goal_yaw;
  ros::Rate rate(50);
  while(ros::ok() && abs(error) > 0.011){
    ros::spinOnce();
    ROS_INFO("%lf", error);
    command_velocity(0, -0.7*error);
    error = global_yaw - goal_yaw;
    if (abs(error) <= 0.011){
      command_velocity(0, 0);
      ROS_INFO("stop sent");
    }
    rate.sleep();
  }

  // Pre-rotation wait
  bag.open("pol_op_recording.bag", rosbag::bagmode::Write);
  auto start_time = std::chrono::system_clock::now();
  auto current_time = std::chrono::system_clock::now();
  std::chrono::duration<double> diff =  current_time - start_time;
  double corrected_yaw =
    global_yaw >= 0 ?
    global_yaw :
    2*bb_util::defs::PI + global_yaw;
  ros::Rate ten_hz(10);

  while (ros::ok() && diff < bookend_time){
    ros::spinOnce();
    corrected_yaw =
      global_yaw >= 0 ?
      global_yaw :
      2*bb_util::defs::PI + global_yaw;

    std_msgs::Int32MultiArray pol_msg;
    std_msgs::Float64 yaw_msg;
    pol_msg.data = global_sensor_data;
    yaw_msg.data = global_yaw;
    bag.write(odom_sub_topic.c_str(), ros::Time::now(), yaw_msg);
    bag.write(pol_sub_topic.c_str(), ros::Time::now(), pol_msg);
    current_time = std::chrono::system_clock::now();
    diff =  current_time - start_time;
    ten_hz.sleep();
  }

  corrected_yaw =
    global_yaw >= 0 ?
    global_yaw :
    2*bb_util::defs::PI + global_yaw;
  double last_measure = corrected_yaw;
  double traverse = 0;


  while(ros::ok() && traverse <= 2*bb_util::defs::PI){
    ros::spinOnce();
    command_velocity(0, 0.2);
    corrected_yaw =
      global_yaw >= 0 ?
      global_yaw :
      2*bb_util::defs::PI + global_yaw;

    traverse = traverse + clip(corrected_yaw - last_measure);
    ROS_INFO("LM: %lf, CY: %lf, Traverse: %lf", last_measure, corrected_yaw, traverse);
    last_measure = corrected_yaw;

    std_msgs::Int32MultiArray pol_msg;
    std_msgs::Float64 yaw_msg;
    pol_msg.data = global_sensor_data;
    yaw_msg.data = global_yaw;
    bag.write(odom_sub_topic.c_str(), ros::Time::now(), yaw_msg);
    bag.write(pol_sub_topic.c_str(), ros::Time::now(), pol_msg);
    if (traverse >= 2*bb_util::defs::PI){
      command_velocity(0, 0);
      ROS_INFO("stop sent");
    }
    ten_hz.sleep();
  }

  // Post rotation wait.
  start_time = std::chrono::system_clock::now();
  current_time = std::chrono::system_clock::now();
  diff =  current_time - start_time;
  corrected_yaw =
    global_yaw >= 0 ?
    global_yaw :
    2*bb_util::defs::PI + global_yaw;
  while (ros::ok() && diff < bookend_time){
    ros::spinOnce();
    corrected_yaw =
      global_yaw >= 0 ?
      global_yaw :
      2*bb_util::defs::PI + global_yaw;

    std_msgs::Int32MultiArray pol_msg;
    std_msgs::Float64 yaw_msg;
    pol_msg.data = global_sensor_data;
    yaw_msg.data = global_yaw;
    bag.write(odom_sub_topic.c_str(), ros::Time::now(), yaw_msg);
    bag.write(pol_sub_topic.c_str(), ros::Time::now(), pol_msg);
    current_time = std::chrono::system_clock::now();
    diff =  current_time - start_time;
    ten_hz.sleep();
  }

  // Close bag and exit.
  bag.close();
  ROS_INFO("Exiting");
  return 0;
}
