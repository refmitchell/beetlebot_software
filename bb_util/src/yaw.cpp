/**
   \file yaw.cpp
   \brief Utility node to extract and print yaw information
 */

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>
#include <tf/transform_datatypes.h>
#include <std_msgs/Float64.h>

ros::Publisher yaw_pub;

void odomCallback(const nav_msgs::Odometry::ConstPtr& odom_msg){
  // Conversion from Quaternion to RPY, lifted from ROS forums.
  tf::Quaternion quat;
  tf::quaternionMsgToTF(odom_msg->pose.pose.orientation, quat);

  double roll, pitch, yaw;
  tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
  std_msgs::Float64 msg;
  msg.data = yaw;
  yaw_pub.publish(msg);
  ROS_INFO("Yaw: %lf", yaw);
}

int main(int argc, char **argv){
  ros::init(argc, argv, "yaw_node");
  ros::NodeHandle n;
  ros::Subscriber odom_sub = n.subscribe("/odom", 1000, odomCallback);

  yaw_pub = n.advertise<std_msgs::Float64>("yaw", 1000);

  while(ros::ok()){
    ros::spinOnce();
  }
  return 0;
}
