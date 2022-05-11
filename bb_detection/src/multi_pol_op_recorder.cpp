/**

*/

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/opencv.hpp>

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
std::vector<int> global_sensor_data1; // Current sensor data (sign-extended)
std::vector<int> global_sensor_data2; // Current sensor data (sign-extended)
std::vector<int> global_sensor_data3; // Current sensor data (sign-extended)
std::vector<int> global_sensor_data4; // Current sensor data (sign-extended)
std::vector<int> global_sensor_data5; // Current sensor data (sign-extended)
std::vector<int> global_sensor_data6; // Current sensor data (sign-extended)
std::vector<int> global_sensor_data7; // Current sensor data (sign-extended)

sensor_msgs::Image sm_frame;

double global_yaw; // Current yaw (from odom)
nav_msgs::Odometry global_odom_msg;
float angular_distance = 0;
bool yaw_data_received = false;
bool pol_data_received = false;

std::string pol_sub_topic = "pol_op_0";
std::string pol_sub_topic1 = "pol_op_1";
std::string pol_sub_topic2 = "pol_op_2";
std::string pol_sub_topic3 = "pol_op_3";
std::string pol_sub_topic4 = "pol_op_4";
std::string pol_sub_topic5 = "pol_op_5";
std::string pol_sub_topic6 = "pol_op_6";
std::string pol_sub_topic7 = "pol_op_7";

std::string odom_sub_topic = "yaw";
std::string cam_sub_topic = "frames";
std::string full_odom_sub_topic = "odom";
std::string node_name = "pol_op_pd_test_recorder";

double clip(double value){
  return value >= 0 ? value : 0;
}

void write_all_to_bag(rosbag::Bag& bag){
    std_msgs::Int32MultiArray pol_msg;
    std_msgs::Int32MultiArray pol_msg1;
    std_msgs::Int32MultiArray pol_msg2;
    std_msgs::Int32MultiArray pol_msg3;
    std_msgs::Int32MultiArray pol_msg4;
    std_msgs::Int32MultiArray pol_msg5;
    std_msgs::Int32MultiArray pol_msg6;
    std_msgs::Int32MultiArray pol_msg7;
    std_msgs::Float64 yaw_msg;
    nav_msgs::Odometry full_odom_msg;
    pol_msg.data = global_sensor_data;
    pol_msg1.data = global_sensor_data1;
    pol_msg2.data = global_sensor_data2;
    pol_msg3.data = global_sensor_data3;
    pol_msg4.data = global_sensor_data4;
    pol_msg5.data = global_sensor_data5;
    pol_msg6.data = global_sensor_data6;
    pol_msg7.data = global_sensor_data7;
    yaw_msg.data = global_yaw;

    // Write yaw info
    bag.write(odom_sub_topic.c_str(), ros::Time::now(), yaw_msg);
    bag.write(full_odom_sub_topic.c_str(), ros::Time::now(), global_odom_msg);

    // Write all pol-op data
    bag.write(pol_sub_topic.c_str(), ros::Time::now(), pol_msg);
    bag.write(pol_sub_topic1.c_str(), ros::Time::now(), pol_msg1);
    bag.write(pol_sub_topic2.c_str(), ros::Time::now(), pol_msg2);
    bag.write(pol_sub_topic3.c_str(), ros::Time::now(), pol_msg3);
    bag.write(pol_sub_topic4.c_str(), ros::Time::now(), pol_msg4);
    bag.write(pol_sub_topic5.c_str(), ros::Time::now(), pol_msg5);
    bag.write(pol_sub_topic6.c_str(), ros::Time::now(), pol_msg6);
    bag.write(pol_sub_topic7.c_str(), ros::Time::now(), pol_msg7);

    // Write image
    bag.write(cam_sub_topic.c_str(), ros::Time::now(), sm_frame);
}

//Publish a velocity command.
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

void imageCallback(const sensor_msgs::ImageConstPtr& msg){
  sm_frame = *msg;
}

void yawCallback(const std_msgs::Float64::ConstPtr& yaw_msg){
  // Conversion from Quaternion to RPY, lifted from ROS forums.
  global_yaw = yaw_msg->data; // Update local memory

  yaw_data_received = true;
}

void odometryCallback(const nav_msgs::Odometry::ConstPtr& odom_msg){
  global_odom_msg = *odom_msg;
}

void polCallback(const std_msgs::Int32MultiArray::ConstPtr& pol_msg){
  global_sensor_data = pol_msg->data; // Update local memory

  pol_data_received = true;
}

void polCallback1(const std_msgs::Int32MultiArray::ConstPtr& pol_msg){
  global_sensor_data1 = pol_msg->data; // Update local memory
  pol_data_received = true;
}

void polCallback2(const std_msgs::Int32MultiArray::ConstPtr& pol_msg){
  global_sensor_data2 = pol_msg->data; // Update local memory
  pol_data_received = true;
}

void polCallback3(const std_msgs::Int32MultiArray::ConstPtr& pol_msg){
  global_sensor_data3 = pol_msg->data; // Update local memory
  pol_data_received = true;
}

void polCallback4(const std_msgs::Int32MultiArray::ConstPtr& pol_msg){
  global_sensor_data4 = pol_msg->data; // Update local memory

  pol_data_received = true;
}

void polCallback5(const std_msgs::Int32MultiArray::ConstPtr& pol_msg){
  global_sensor_data5 = pol_msg->data; // Update local memory
  pol_data_received = true;
}

void polCallback6(const std_msgs::Int32MultiArray::ConstPtr& pol_msg){
  global_sensor_data6 = pol_msg->data; // Update local memory
  pol_data_received = true;
}

void polCallback7(const std_msgs::Int32MultiArray::ConstPtr& pol_msg){
  global_sensor_data7 = pol_msg->data; // Update local memory
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
  image_transport::ImageTransport it(n);
  ros::Subscriber pol_sub =
    n.subscribe(pol_sub_topic,
                1000,
                polCallback);
  ros::Subscriber pol_sub1 =
    n.subscribe(pol_sub_topic1,
                1000,
                polCallback1);
  ros::Subscriber pol_sub2 =
    n.subscribe(pol_sub_topic2,
                1000,
                polCallback2);
  ros::Subscriber pol_sub3 =
    n.subscribe(pol_sub_topic3,
                1000,
                polCallback3);
  ros::Subscriber pol_sub4 =
    n.subscribe(pol_sub_topic4,
                1000,
                polCallback4);
  ros::Subscriber pol_sub5 =
    n.subscribe(pol_sub_topic5,
                1000,
                polCallback5);
  ros::Subscriber pol_sub6 =
    n.subscribe(pol_sub_topic6,
                1000,
                polCallback6);
  ros::Subscriber pol_sub7 =
    n.subscribe(pol_sub_topic7,
                1000,
                polCallback7);
  ros::Subscriber odom_sub =
    n.subscribe(odom_sub_topic,
                1000,
                yawCallback);
  ros::Subscriber full_odom_sub =
    n.subscribe(full_odom_sub_topic,
                1000,
                odometryCallback);
  image_transport::Subscriber cam_sub =
    it.subscribe(cam_sub_topic,
                 1000,
                 imageCallback);

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

    write_all_to_bag(bag);
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

    write_all_to_bag(bag);
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

    write_all_to_bag(bag);
    current_time = std::chrono::system_clock::now();
    diff =  current_time - start_time;
    ten_hz.sleep();
  }

  // Close bag and exit.
  bag.close();
  ROS_INFO("Exiting");
  return 0;
}
