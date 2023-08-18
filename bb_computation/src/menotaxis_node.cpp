
/*
 \file menotaxis_node.cpp
 \brief Perform menotaxis using a single cue and CX for steering.

 This node will subscribe to a specified bb_util::cue_msg and use it to
 provide directional input to the CX. Before movement begins, a random direction
 is selected and passed to the central complex along with some speed input.

 This generates a 'home vector' pointing opposite to the random direction. During
 'homing', no speed information is given to the CX so this home vector never
 shrinks. This is a little hacky but functionally provides menotaxis.

 \warning If the bb_locomotion cmd_vel_service is running, starting
 this node will cause the robot to move!
*/

#include <ros/ros.h>
#include "bb_computation/cx_model.hpp"

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

double goal_angle = 0;

/**
   Initialise the argument parser
   \param parser The parser object
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

   \note No linear velocity is passed to the CentralComplex so it will
   never 'reach home'. The point of this node is to provide continuous
   movement in a given direction.

   \warning If bb_locomotion cmd_vel_service is running, then starting
   this node will cause the robot to move.

   \param cue_msg A bb_util::cue_msg
*/
void cueCallback(const bb_util::cue_msg::ConstPtr& cue_msg){
  float current = cue_msg->theta;

  // Update CX angle but do not provide speed input (not homing).
  double CXMotor = cx.unimodal_monolithic_CX(current, 0);

#if DEBUG
  ROS_INFO("CXMotor: %lf", CXMotor);
#endif

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
  double angular = 0.7;
  double linear = 0.1;  
  msg.request.linear = std::abs(CXMotor) < 1 ? linear : 0;
  msg.request.angular = 0;
  
  if (CXMotor != 0) { msg.request.angular = (CXMotor > 0) ? angular : -angular; }

  client.call(msg);
}

int main(int argc, char **argv){
  if (!initParser(parser, argc, argv)) return -1;
  if(parser.exists("help")){
    parser.print_help();
    return 0;
  }

  // Set pub and sub topics
  std::string sub_topic = parser.get<std::string>("subscribe");
  std::string cx_pub =
    parser.exists("publish") ?
    parser.get<std::string>("publish") :
    "cx_status";

  // Init ROS node, subs, pubs, and service clients.
  ros::init(argc, argv, "menotaxis");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe(sub_topic.c_str(), 1000, cueCallback);
  pub = n.advertise<bb_util::cx_activity>(cx_pub.c_str(), 1);
  client = n.serviceClient<bb_util::velocity>(bb_util::defs::UPDATE_VELOCITY);

  // Choose random angle between 0 and 2pi
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_real_distribution<> dis(0.0, 2*bb_util::defs::PI);

  // Inject displacement into the CX
  cx.unimodal_monolithic_CX(dis(gen), 10);
  while(ros::ok()){
    ros::spinOnce();
  }

  return 0;
}
