
/*
 pi_node.cpp
 Subscribe to a single cue topic which is fed into the CX to allow it to
 control steering
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

#define GOAL_ANGLE -2.0
#define TRAVERSE_TIME 30

// Init CX
CentralComplex cx;

// Broadcast CX status
ros::Publisher pub;

// Send movement commands to the update_velocity service
ros::ServiceClient client;

argparse::ArgumentParser parser("parser");

double velocity = 0;

inline double clean_velocity(double lin_vel, int factor=100){
  if (factor == 0) return 0;
  return ((double) ((int) (lin_vel * factor))) / factor;
}

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
void cueCallback(const bb_util::cue_msg::ConstPtr& cue_msg){
  ROS_INFO("Cue info: (R: %f, T: %f)",
           cue_msg->reliability,
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

  ROS_INFO("CX_MOTOR: %lf", CXMotor);
  //
  // Velocity update
  //
  bb_util::velocity msg;

  // Do not move unless CXMotor sufficiently small
  // msg.request.linear = 0.02;
  // if (std::abs(CXMotor) < 0.002){
  msg.request.linear = 0.1;
  // }

  double angular = 0.7;
  msg.request.angular = 0;
  if (CXMotor != 0) { msg.request.angular = (CXMotor > 0) ? angular : -angular; }


  client.call(msg);

}

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

  std::string sub_topic = parser.get<std::string>("subscribe");
  std::string cx_pub =
    parser.exists("publish") ?
    parser.get<std::string>("publish") :
    "cx_status";

  ros::init(argc, argv, "PI");
  ros::NodeHandle n;

  // ROS networking
  // Get cue - Sub to visual cues
  ros::Subscriber cue_sub = n.subscribe(sub_topic.c_str(), 1000, cueCallback);

  // Sub to odometry for internal compass info
  ros::Subscriber odom_sub = n.subscribe("odom", 1000, odomCallback);

  // Request velocity changes from velocity service
  client = n.serviceClient<bb_util::velocity>("update_velocity");

  // Set up publisher for CX
  pub = n.advertise<bb_util::cx_activity>("cx_status", 1);

  ROS_INFO("Running...");

  ros::spin();

  return 0;
}
