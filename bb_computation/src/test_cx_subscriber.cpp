
/*
 test_cx_subscriber.cpp
 Subscribe to a single cue vector and feed it into the CX, let the
 CX drive steering.

 Test module current state: subscribe to cue and feed into VMCX, testing
 VMCX functionality.
*/

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>
#include <tf/transform_datatypes.h>
#include "std_msgs/String.h"


#include "bb_computation/icm.h"
#include "bb_computation/cue_vector.h"
#include "bb_computation/cx_model.hpp"
#include "bb_computation/vmcx_model.hpp"

#include "bb_util/velocity.h"
#include "bb_util/cx_activity.h"


#define GOAL_ANGLE -2.0
#define TRAVERSE_TIME 30



// Init CX
//VMCX cx;
CentralComplex cx;

// Try to init VMCX;
//VMCX vmcx;

// Broadcast CX status
ros::Publisher pub;

// Send movement commands to the update_velocity service
ros::ServiceClient client;

// Weird wee function that can help clean the linear velocity component
// of the odometry information. There's a little noise which could cause
// issues and it's more straightforward for now just to remove it.
// This is by no means required. Didn't use it for the yaw because I
// don't think there's any advantage to cleaning that up.
inline double clean_velocity(double lin_vel, int factor=100){
  if (factor == 0) return 0;
  return ((double) ((int) (lin_vel * factor))) / factor;
}

//
// Publish for graphing
//
void cx_status_publish(std::vector<std::vector<double>> &status){
  bb_util::cx_activity msg;

  msg.tl2 = status[0];
  msg.cl1 = status[1];
  msg.tb1 = status[2];
  msg.cpu4 = status[3];
  msg.mem = status[4];
  msg.cpu1 = status[5];

  pub.publish(msg);
}

//
// Cue callback
//
void cvCallback(const bb_computation::cue_vector::ConstPtr& cue_msg){
  ROS_INFO("Cue info: (R: %f, T: %f)",
           cue_msg->magnitude,
           cue_msg->theta);
  //
  // Msg unpack
  //
  float current = cue_msg->theta;
  float error = GOAL_ANGLE - current;
  float correction_velocity = -0.09*error;

  // Get motor output from CX
  double CXMotor = cx.unimodal_monolithic_CX(current, 10);

  // Get status
  std::vector<std::vector<double>> cx_status;
  cx.get_status(cx_status);
  cx_status_publish(cx_status);

  //
  // Velocity update
  //
  bb_util::velocity msg;
  msg.request.angular = correction_velocity;
  msg.request.linear = 0.0;
  client.call(msg);
}

void odomCallback(const nav_msgs::Odometry::ConstPtr& odom_msg){
  // odom.twist.linear.x // Linear velocity or forward speed
  // odom.pose.orientation.z // Heading - quaternion comes into play

  // Conversion from Quaternion to RPY, lifted from ROS forums.
  tf::Quaternion quat;
  tf::quaternionMsgToTF(odom_msg->pose.pose.orientation, quat);

  double roll, pitch, yaw;
  tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);

  double linear_velocity = clean_velocity(odom_msg->twist.twist.linear.x);
  double CXMotor = cx.unimodal_monolithic_CX(yaw, linear_velocity);

  ROS_INFO("[CXMOTOR] : [%f]", CXMotor);

  // Get status
  std::vector<std::vector<double>> cx_status;
  cx.get_status(cx_status);
  cx_status_publish(cx_status);

  // Update velocity
  bb_util::velocity msg;

  ///////////////////////////////////////////////////////////
  // DONT RUN THIS ON THE ROBOT; YOU WILL BREAK THE MOTORS //
  ///////////////////////////////////////////////////////////
  msg.request.angular = (CXMotor < 0) ? 0.5 : -0.5;
  msg.request.linear = 0.05;
  client.call(msg);
  ///////////////////////////////////////////////////////////
  // DONT RUN THIS ON THE ROBOT; YOU WILL BREAK THE MOTORS //
  ///////////////////////////////////////////////////////////
}

int main(int argc, char **argv){
  ros::init(argc, argv, "test_cx_subscriber");
  ros::NodeHandle n;

  // ROS networking
  // Get cue - Sub to visual cues
  //ros::Subscriber sub = n.subscribe("visual_cue", 1000, cvCallback);

  // Sub to odometry for internal compass info
  ros::Subscriber sub = n.subscribe("odom", 1000, odomCallback);

  // Request velocity changes from velocity service
  client = n.serviceClient<bb_util::velocity>("update_velocity");

  // Set up publisher for CX
  pub = n.advertise<bb_util::cx_activity>("cx_status", 1);

  ROS_INFO("Running...");

  ros::spin();

  return 0;
}
