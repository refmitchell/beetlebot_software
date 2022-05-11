/**
@file offline_pol_sensor_recorder.cpp
@brief read recording, compute solar vector and record result.
*/

#include <iostream>
#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Float64.h>
#include <rosbag/bag.h>


std::vector<double> global_solar_vector;
double global_yaw;
bool yaw_received = false;
bool solvec_received = false;

void solCallback(const std_msgs::Float64MultiArray::ConstPtr& msg){
  global_solar_vector = msg->data;
  solvec_received = true;
}

void yawCallback(const std_msgs::Float64::ConstPtr& msg){
  global_yaw = msg->data;
  yaw_received = true;
}

int main(int argc, char** argv){
  ros::init(argc, argv, "offline_sensor_recorder");
  ros::NodeHandle n;
  ros::Subscriber sol_sub =
    n.subscribe("solar_vector", 1000, solCallback);
  ros::Subscriber yaw_sub =
    n.subscribe("yaw", 1000, yawCallback);

  while(ros::ok() && !(solvec_received || yaw_received)){
    ros::spinOnce();
    ROS_INFO("Waiting for yaw and solar_vector channels to be active.");
    ROS_INFO("Yaw: %d, sol: %d", solvec_received, yaw_received);
  }

  ROS_INFO("Data received");

  rosbag::Bag bag;
  bag.open("sol_vec_recording.bag", rosbag::bagmode::Write);
  ros::Rate ten_hz(10);
  while(ros::ok()){
    ros::spinOnce();
    std_msgs::Float64MultiArray sol_vec_msg;
    std_msgs::Float64 yaw_msg;
    sol_vec_msg.data = global_solar_vector;
    yaw_msg.data = global_yaw;
    bag.write("solar_vector", ros::Time::now(), sol_vec_msg);
    bag.write("yaw", ros::Time::now(), yaw_msg);
    ten_hz.sleep();
  }

  bag.close();
  ROS_INFO("Exiting.");

  return 0;
}
