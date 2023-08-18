
/**
   \file test_vmcx_subscriber.cpp
   \brief Node which runs the vector memory model from Le Moel et al. (2019).

   Node to test the functionality of the Le Moel et al. (2019) vector memory
   model.

   The node currently tracks direction by using the raw beetlebot IMU but
   this could easily be switched to use a bb_util::Cue. 

   To use, start the node at the 'home' location and drive it manually to
   a 'feeder' location (e.g. using turtlebot_teleop_key). You then need to
   send vm_command service request (defined in srv/vm_command.srv) with
   the "store" command. This will store the current PI status in the vector
   memory. The robot constantly publishes motor commands, to engage the motors
   start bb_locomotion cmd_vel_service. The robot should now home (it will loop
   when it gets back to where it thinks home is). If you then send another
   service request, this time with the "activate" command, the vector memory
   neuron will be activated and the robot should navigate back out to the
   'feeder' location. 
*/

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>
#include <tf/transform_datatypes.h>
#include "std_msgs/String.h"

#include "bb_computation/vm_command.h"
#include "bb_computation/cx_model.hpp"
#include "bb_computation/vmcx_model.hpp"

#include "bb_util/velocity.h"
#include "bb_util/vmcx_activity.h"
#include "bb_util/cue_msg.h"

#define GOAL_ANGLE -2.0
#define TRAVERSE_TIME 30

// Init CX
VMCX cx;

// Broadcast CX status
ros::Publisher pub;

// Send movement commands to the update_velocity service
ros::ServiceClient client;

// Server to accept VM commands because it seems like the most
// direct solution at the moment.
ros::Subscriber vm_sub;

// Clean the linear velocity prior to passing it into the CX
inline double clean_velocity(double lin_vel, int factor=100){
  if (factor == 0) return 0;
  return ((double) ((int) (lin_vel * factor))) / factor;
}

//
// Publish for graphing
//
void cx_status_publish(std::vector<std::vector<double>> &status){
  bb_util::vmcx_activity msg;

  msg.tl2 = status[0];
  msg.cl1 = status[1];
  msg.tb1 = status[2];
  msg.cpu4 = status[3];
  msg.mem = status[4];
  msg.cpu1 = status[5];
  msg.vm = status[6];
  msg.active = status[7];

  pub.publish(msg);
}

//
// Cue callback
//
void cvCallback(const bb_util::cue_msg::ConstPtr& cue_msg){
  ROS_INFO("Cue info: (R: %f, T: %f)",
           cue_msg->contrast,
           cue_msg->theta);
  //
  // Msg unpack
  //
  float current = cue_msg->theta;
  float error = GOAL_ANGLE - current;
  float correction_velocity = -0.09*error;

  // Get motor output from CX
  double CXMotor = cx.unimodal_VMCX(current, 10);

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
  double CXMotor = cx.unimodal_VMCX(yaw, linear_velocity);

  //ROS_INFO("[CXMOTOR] : [%f]", CXMotor);

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
  msg.request.linear = 0.2;
  client.call(msg);
  ///////////////////////////////////////////////////////////
  // DONT RUN THIS ON THE ROBOT; YOU WILL BREAK THE MOTORS //
  ///////////////////////////////////////////////////////////
}

// Really quick and dirty method to manage the VM status
// bool manage_vm(bb_computation::vm_command::Request& req,
//                bb_computation::vm_command::Response& res){
void manage_vm_callback(const std_msgs::String::ConstPtr& msg){
  std::string cmd = msg->data;
  ROS_INFO("%s", cmd.c_str());
  if (!cmd.compare("toggle")){
    cx.toggle_vm(); // Will print the current state
  } else if (!cmd.compare("store")){
    cx.store_vm();
  } else if (!cmd.compare("activate")){
    cx.activate_vm();
  } else if (!cmd.compare("deactivate")){
    cx.deactivate_vm();
  } else if (!cmd.compare("clear")){
    cx.clear_vm();
  } else {
    ROS_INFO("Command %s not recognised", cmd.c_str());
  }
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

  // Set up VM Server
  vm_sub = n.subscribe("vm_management", 1000, manage_vm_callback);

  // Set up publisher for CX
  pub = n.advertise<bb_util::vmcx_activity>("vmcx_status", 1);

  ROS_INFO("Running...");

  ros::spin();

  return 0;
}
