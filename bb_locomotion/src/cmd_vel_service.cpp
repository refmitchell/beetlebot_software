/**
   \file cmd_vel_service.cpp
   \brief A node which links the beetlebot software to the turtlebot cmd_vel topic.

   This node provides a link from the computational models in bb_computation to
   the motors on the turtlebot. This allows computational models to output motor 
   commands without worrying how these will be interpreted.
*/

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

#include "bb_util/bb_util.h"
#include "bb_util/velocity.h"
#include "bb_util/locomotion_cmd.h"

ros::Publisher cmd_publisher;

/**
   Helper function which abstracts message formation.
   \param linear the desired linear velocity
   \param angular the desired angular velocity
*/
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


/**
   The ROS service callback.
   \param req The request portion of the srv format
   \param res The response portion of the srv format
   \return true on success
*/
bool cmd_callback (bb_util::velocity::Request& req,
                   bb_util::velocity::Response& res
                   )
{
  ROS_INFO("I heard: angular - %f, linear - %f", req.angular, req.linear);
  command_velocity(req.linear, req.angular);

  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "cmd_vel_service");
  ros::NodeHandle n;

  cmd_publisher = n.advertise<geometry_msgs::Twist>("cmd_vel", 10);

  ros::ServiceServer cmd_service =
    n.advertiseService("update_velocity", cmd_callback);

  // Need a small delay to let everything link up at initialisation
  ros::Duration(0.3).sleep();

  ros::spin();

  return 0;
}
