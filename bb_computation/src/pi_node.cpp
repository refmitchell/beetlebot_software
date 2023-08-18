
/*
 \file pi_node.cpp
 \brief Host a CX model subscibed to a single cue topic.

 This node hosts a CentralComplex model and allows an operator to run a
 path integration experiment.

 Start this node (make sure the cmd_vel_service is not running) to start
 the CentralComplex. Drive the robot manually (using turtlebot3_teleop_key
 or any other suitable method) to a target location. Stop teleoperation.
 Start the bb_locomotion cmd_vel_service to allow this node to communicate
 with the motors on the turtlebot. The robot will then home.

 If you actually want to run path integration experiments then there
 may be ways to streamline this process but as a proof of prinicple
 this was sufficient.

 \warning If the bb_locomotion cmd_vel_service is running then starting
 this node will cause the robot to move!
*/

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>
#include <tf/transform_datatypes.h>
#include "std_msgs/String.h"

#include "bb_computation/icm.h"
#include "bb_computation/cx_model.hpp"
#include "bb_computation/vmcx_model.hpp"

#include "bb_util/velocity.h"
#include "bb_util/cx_activity.h"
#include "bb_util/argparse.h"
#include "bb_util/cue.hpp"
#include "bb_util/cue_msg.h"

#define DEBUG 0

// Init CX
CentralComplex cx;

// Broadcast CX status
ros::Publisher pub;

// Send movement commands to the update_velocity service
ros::ServiceClient client;

argparse::ArgumentParser parser("parser");

double velocity = 0;

/**
   Clean up received linear velocity. This function will trim
   the linear velocity to N decimal places where N is the order
   of magnitude of the factor.

   \param lin_vel The raw linear velocity
   \param factor The cleaning factor to use
   \return The cleaned value
*/
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
    .description("Override the CX status publisher.")
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


/**
   Publish the CX status so that it can be plotted by bb_graphics/CX_graphing.
   \param status The current CX status (can be fetched from the CentralComplex class).
*/

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

/**
   Cue received callback. Receive current compass information
   from the given cue, pass it to the CX and generate a motor 
   command. The motor command requests will always be sent.

   \warning If bb_locomotion cmd_vel_service is running, then starting
   this node will cause the robot to move.

   \param cue_msg A bb_util::cue_msg
*/
void cueCallback(const bb_util::cue_msg::ConstPtr& cue_msg){
  ROS_INFO("Cue info: (R: %f, T: %f)",
           cue_msg->contrast,
           cue_msg->theta);
  //
  // Msg unpack
  //
  float current_angle = cue_msg->theta;

  // Get motor output from CX
  double CXMotor = cx.unimodal_monolithic_CX(current_angle, velocity);

  // Get status
  std::vector<std::vector<double>> cx_status;
  cx.get_status(cx_status);
  cx_status_publish(cx_status);

#if DEBUG
  ROS_INFO("CX_MOTOR: %lf", CXMotor);
#endif
  
  //
  // Velocity update
  //
  bb_util::velocity msg;

  // If CXMotor small enough, then allow forward movement,
  // else set linear velocity to zero and turn on the spot.
  // Threshold of CXMotor < 1 arbitrarily chosen.
  double angular = 0.7;
  double linear = 0.1;
  msg.request.linear = std::abs(CXMotor) < 1 ? linear : 0;
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

int main(int argc, char **argv){
  if (!initParser(parser, argc, argv)) return -1;
  if (parser.exists("help")){
    parser.print_help();
    return -1;
  }

  // Set pub and sub topics
  std::string sub_topic = parser.get<std::string>("subscribe");
  std::string cx_pub =
    parser.exists("publish") ?
    parser.get<std::string>("publish") :
    bb_util::defs::CX_STATUS; //"cx_status"

  // Init ROS node
  ros::init(argc, argv, "PI");
  ros::NodeHandle n;

  // ROS networking
  // Get cue - Sub to visual cues
  ros::Subscriber cue_sub = n.subscribe(sub_topic.c_str(), 1000, cueCallback);

  // Sub to odometry for internal compass info
  ros::Subscriber odom_sub = n.subscribe("odom", 1000, odomCallback);

  // Request velocity changes from velocity service "update_velocity"
  client = n.serviceClient<bb_util::velocity>(bb_util::defs::UPDATE_VELOCITY);

  // Set up publisher for CX
  pub = n.advertise<bb_util::cx_activity>(cx_pub.c_str(), 1);

  ROS_INFO("Running...");

  ros::spin();

  return 0;
}
