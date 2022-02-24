/**
   @file presets.cpp
   @brief Provides preset movement sequences which can be called via a service.

   Preset movement sequences which operate via feedback control. The service
   blocks until movement commands are complete. Can be used to send one-shot
   movement commands.
*/

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

#include "locomotion/velocity.h"

#include "bb_util/bb_util.h"
#include "bb_util/velocity.h"
#include "bb_util/locomotion_cmd.h"

#define ROS_LINK_DELAY = 0.3

ros::Subscriber odom_sub;
ros::Publisher cmd_publisher;

double global_yaw = 0; // Initialisation, will be overwritten.

//Publish a velocity command.
void command_velocity(float linear, float angular){
 geometry_msgs::Twist msg;

  msg.linear.x = linear;
  msg.linear.y = 0.0;
  msg.linear.z = 0.0;

  msg.angular.x = 0.0;
  msg.angular.y = 0.0;
  msg.angular.z = angular;

  cmd_publisher.publish(msg);
}

void rotate(){
  double last_measure = global_yaw;
  double traverse = 0;

  command_velocity(0, 0.1);

  // Move in a slow circle back to current position.
  while(traverse < 2*bb_util::defs::PI){
    traverse = traverse + (global_yaw - last_measure);
    last_measure = global_yaw;
  }
}

void move_to_target(double goal_yaw){
  double error = global_yaw - goal_yaw;
  while(error > 0.01){
    command_velocity(0, 0.9*error);
    error = global_yaw - goal_yaw;
  }
}

bool loc_node_callback(bb_util::locomotion_cmd::Request& req,
                       bb_util::locomotion_cmd::Response& res
                        ){
  const int opcode = req.opcode;

  switch(opcode){
  case bb_util::loc_node_commands::driving::ALL_STOP:
    command_velocity(0,0);
    res.success = true;
    res.response = bb_util::responses::COMPLETE;
  case bb_util::loc_node_commands::driving::ZERO:
    move_to_target(0);
    res.success = true;
    res.response = bb_util::responses::COMPLETE;
  case bb_util::loc_node_commands::driving::RGT:
    command_velocity(0,0.2);
    res.success = true;
    res.response = bb_util::responses::COMPLETE;
  case bb_util::loc_node_commands::driving::FWD:
    command_velocity(0.15, 0);
    res.success = true;
    res.response = bb_util::responses::COMPLETE;
  case bb_util::loc_node_commands::driving::BWD:
    command_velocity(-0.15, 0);
    res.success = true;
    res.response = bb_util::responses::COMPLETE;
  case bb_util::loc_node_commands::driving::LFT:
    command_velocity(0,-0.2);
    res.success = true;
    res.response = bb_util::responses::COMPLETE;
  default:
    command_velocity(0,0); // Do nothing
  }

  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "cmd_vel_service");
  ros::NodeHandle n;

  cmd_publisher = n.advertise<geometry_msgs::Twist>("cmd_vel", 10);

  ros::ServiceServer sequence_Service =
    n.advertiseService("preset_movement", loc_node_callback);

  // Need a small delay to let everything link up at initialisation
  ros::Duration(0.3).sleep();

  ros::spin();

  return 0;
}
