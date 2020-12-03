#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

#include "locomotion/velocity.h"

#include "db_util/db_util.h"
#include "db_util/velocity.h"
#include "db_util/locomotion_cmd.h"

#define ROS_LINK_DELAY = 0.3 //Found through trial and error?

// Don't judge me pls
ros::Publisher cmd_publisher;

// Lock velocities in place; we may want to lock linear velocity while
// updating angular.
namespace node_settings {
  bool linear_lock = false;
  bool angular_lock = false;

  inline std::string bool2string(bool b){ return b ? "true" : "false"; }
}

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

bool cmd_callback (db_util::velocity::Request& req,
                   db_util::velocity::Response& res
                   )
{
  ROS_INFO("I heard: angular - %f, linear - %f", req.linear, req.angular);
  command_velocity(req.linear, req.angular);

  // geometry_msgs::Twist msg;

  // msg.linear.x = req.linear;
  // msg.linear.y = 0.0;
  // msg.linear.z = 0.0;

  // msg.angular.x = 0.0;
  // msg.angular.y = 0.0;
  // msg.angular.z = req.angular;

  // cmd_publisher.publish(msg);

  return true;
}

bool loc_node_callback(db_util::locomotion_cmd::Request& req,
                       db_util::locomotion_cmd::Response& res
                        )
{
  const int opcode = req.opcode;

  using namespace db_util::loc_node_commands;

  switch(opcode){
  case driving::ALL_STOP:
    command_velocity(0,0);
    res.response = "Stopping all motors.";
  case driving::FWD:
    command_velocity(0.15, 0);
    res.response = "Linear velocity set to: 0.15.";
  case driving::BWD:
    command_velocity(-0.15, 0);
    res.response = "Linear velocity set to: -0.15.";
  case driving::LFT:
    command_velocity(0,-0.08);
    res.response = "Angular velocity set to: -0.08.";
  case driving::RGT:
    command_velocity(0,0.08);
    res.response = "Angular velocity set to: 0.08.";
  case control::LIN_LOCK_TOGGLE:
    node_settings::linear_lock = !(node_settings::linear_lock);
    res.response =
      "Linear lock set to: " +
      node_settings::bool2string(node_settings::linear_lock);
  case control::ANG_LOCK_TOGGLE:
    node_settings::angular_lock = !(node_settings::angular_lock);
    res.response =
      "Angular lock set to: " +
      node_settings::bool2string(node_settings::angular_lock);
  default:
    command_velocity(0,0);
  }


  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "cmd_vel_service");
  ros::NodeHandle n;

  cmd_publisher = n.advertise<geometry_msgs::Twist>("cmd_vel", 10);

  ros::ServiceServer cmd_service =
    n.advertiseService("update_velocity", cmd_callback);

  // ros::ServiceServer motor_cmd_service =
  //   n.advertiseService("motor_cmd_priority", motor_cmd_callback);

  // Need a small delay to let everything link up at initialisation
  ros::Duration(0.3).sleep();

  ros::spin();

  return 0;
}
