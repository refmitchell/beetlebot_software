/**
   @file yaw.cpp
   @brief Utility node to extract and print yaw information

   Primarily used for calibration purposes but may be used to simply
   broadcast yaw on its own topic to save unpicking the quaternion at
   least twice.
 */

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>
#include <tf/transform_datatypes.h>

void odomCallback(const nav_msgs::Odometry::ConstPtr& odom_msg){
  // Conversion from Quaternion to RPY, lifted from ROS forums.
  tf::Quaternion quat;
  tf::quaternionMsgToTF(odom_msg->pose.pose.orientation, quat);

  double roll, pitch, yaw;
  tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
  ROS_INFO("Yaw: %lf", yaw);
}

int main(int argc, char **argv){
  ros::init(argc, argv, "yaw_node");
  ros::NodeHandle n;
  ros::Subscriber odom_sub = n.subscribe("/odom", 1000, odomCallback);

  while(ros::ok()){
    ros::spinOnce();
  }
  return 0;
}
