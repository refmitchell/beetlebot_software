
/*
 test_vmcx_subscriber.cpp
 Want to test VMCX functionality; this requires a little flexibility in
 interaction. It should basically be the same as the test_cx_subscriber,
 using the internal odometry as the "cue" but this node will have an
 additional server component which allows an experimenter to iteract with 
 the VM (store, toggle, enable, disable...)
*/

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>
#include <tf/transform_datatypes.h>
#include "std_msgs/String.h"


#include "computation/icm.h"
#include "computation/cue_vector.h"
#include "computation/vm_command.h"
#include "computation/cx_model.hpp"
#include "computation/vmcx_model.hpp"
#include "computation/mmcx_model.hpp"

#include "db_util/velocity.h"
#include "db_util/vmcx_activity.h"


#define GOAL_ANGLE -2.0
#define TRAVERSE_TIME 30

// Init CX
//VMCX cx;
MMCX cx;

// Broadcast CX status
ros::Publisher pub;

// Send movement commands to the update_velocity service
ros::ServiceClient client;

// Server to accept VM commands because it seems like the most
// direct solution at the moment.
ros::ServiceServer vm_server;

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
  db_util::vmcx_activity msg;

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
void cvCallback(const computation::cue_vector::ConstPtr& cue_msg){
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
  double CXMotor = cx.unimodal_VMCX(current, 10);

  // Get status
  std::vector<std::vector<double>> cx_status;
  cx.get_status(cx_status);
  cx_status_publish(cx_status);

  //
  // Velocity update
  //
  db_util::velocity msg;
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

  ROS_INFO("[CXMOTOR] : [%f]", CXMotor);

  // Get status
  std::vector<std::vector<double>> cx_status;
  cx.get_status(cx_status);
  cx_status_publish(cx_status);

  // Update velocity
  db_util::velocity msg;

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
bool manage_vm(computation::vm_command::Request& req,
               computation::vm_command::Response& res){
  std::string cmd = req.command;
  ROS_INFO("%s", cmd.c_str());
  if (!cmd.compare("toggle")){
    cx.toggle_vm();
  } else if (!cmd.compare("store")){
    cx.store_vm();
  } else if (!cmd.compare("activate")){
    cx.activate_vm();
  } else if (!cmd.compare("deactivate")){
    cx.deactivate_vm();
  }
  return true;
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
  client = n.serviceClient<db_util::velocity>("update_velocity");

  // Set up VM Server
  vm_server = n.advertiseService("vm_management", manage_vm);

  // Set up publisher for CX
  pub = n.advertise<db_util::vmcx_activity>("vmcx_status", 1);

  ROS_INFO("Running...");

  ros::spin();

  return 0;
}
