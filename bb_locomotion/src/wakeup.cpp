/*
 * Locomotion code example for learning and documentation purposes.
 *
 * Execute a predetermined movement sequence using the cmd_vel topic.
 * Movements are executed using the geometry_msgs/Twist message type.
 * We only care about the linear x and angular z components of velocity,
 * The others do not apply. Linear velocity seems more sensitive.
 */

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

#include "bb_locomotion/velocity.h"

void send_velocity_msg(float linear, float angular, ros::ServiceClient& client){
  bb_locomotion::velocity msg;
  msg.request.linear = linear;
  msg.request.angular = angular;
  msg.request.priority = 0;
  client.call(msg);
}

int wakeup(ros::ServiceClient& client){
  ROS_INFO("Running wakeup sequence...");

  ROS_INFO("Turn attempt, 0.08 angular velocity");
  send_velocity_msg(0.0, 0.08, client);
  ros::Duration(3).sleep();

  ROS_INFO("Halting");
  send_velocity_msg(0.0, 0.0,  client);

  ROS_INFO("Sequence complete.");
}


int main(int argc, char **argv)
{
  // ROS init
  ros::init(argc, argv, "wakeup_node");
  ros::NodeHandle n;

  ros::ServiceClient client =
    n.serviceClient<bb_locomotion::velocity>("update_velocity");

  wakeup(client);

  return 0;
}
