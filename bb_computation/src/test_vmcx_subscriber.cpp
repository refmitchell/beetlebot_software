
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
#include "bb_util/argparse.h"

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

// Global linear velocity
double velocity = 0;

argparse::ArgumentParser parser("parser");

// Clean the linear velocity prior to passing it into the CX
inline double clean_velocity(double lin_vel, int factor=100){
  if (factor == 0) return 0;
  return ((double) ((int) (lin_vel * factor))) / factor;
}

/**
   Initialise the argument parser
   \param parser The ArgumentParser object
   \param argc The command-line argument count
   \param argv The command-line argument values
   \return true on success
*/
bool initParser(argparse::ArgumentParser &parser, int argc, char **argv){
  parser.add_argument()
    .names({"-s", "--subscribe"})
    .description("Set the subscription topic, must be of type bb_util/cue_msg")
    .required(true);
  parser.add_argument()
    .names({"-p", "--publish"})
    .description("Override the VMCX status publisher.")
    .required(false);

  parser.enable_help();

  auto err = parser.parse(argc, const_cast<const char**>(argv));
  if(err){
    std::stringstream ss;
    ss << err;
    ROS_ERROR("argparse error: %s", ss.str().c_str());
    return false;
  }

  return true;
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
void cueCallback(const bb_util::cue_msg::ConstPtr& cue_msg){
  // ROS_INFO("Cue info: (R: %f, T: %f)",
  //          cue_msg->contrast,
  //          cue_msg->theta);
  //
  // Msg unpack
  //
  float current_angle = cue_msg->theta;
  double CXMotor = cx.unimodal_VMCX(current_angle, velocity);

  // Get status
  std::vector<std::vector<double>> cx_status;
  cx.get_status(cx_status);
  cx_status_publish(cx_status);


  //
  // Velocity update
  //
  bb_util::velocity msg;

  // If CXMotor small enough, then allow forward movement,
  // else set linear velocity to zero and turn on the spot.
  // Threshold of CXMotor < 1 arbitrarily chosen.
  double angular = 0.3;
  double linear = 0.1;
  msg.request.linear = std::abs(CXMotor) < 0.5 ? linear : 0;
  msg.request.angular = 0;
  
  if (CXMotor != 0) { msg.request.angular = (CXMotor > 0) ? angular : -angular; }

  client.call(msg);
}

/**
   Callback for odometry information, specifically linear velocity.
   Note that this uses perceived linear velocity, not the intended
   velocity.
   \param odom_msg The odometry message.
*/
void odomCallback(const nav_msgs::Odometry::ConstPtr& odom_msg){
  double linear_velocity = clean_velocity(odom_msg->twist.twist.linear.x);
  velocity = linear_velocity;
}


// Really quick and dirty method to manage the VM status
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
  if (!initParser(parser, argc, argv)) return -1;
  if (parser.exists("help")){
    parser.print_help();
    return -1;
  }

  // Set pub and sub topics
  std::string sub_topic = parser.get<std::string>("subscribe");

  // ROS networking

  // Get cue - Sub to visual cues
  ros::Subscriber sub = n.subscribe(sub_topic.c_str(), 1000, cueCallback);
  ros::Subscriber odom_sub = n.subscribe("odom", 1000, odomCallback);

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
