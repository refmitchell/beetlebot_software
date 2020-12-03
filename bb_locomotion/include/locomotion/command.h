#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

#include "locomotion/velocity.h"



class Command {
private:
  // Members
  static ros::Publisher cmd_publisher;
  static ros::ServiceServer cmd_service;

  static float linear_velocity;
  static float angular_velocity;
  static int cmd_priority;

  static bool initialised;

  // Methods
  static geometry_msgs::Twist geometry_msg();

  // Private ctor; singleton class
  Command(){initialised = false;};

public:
  static bool initialise(ros::NodeHandle n);
  static void set_velocity(locomotion::velocity::Request& req,
                           locomotion::velocity::Response& res);
  static void clear_cmd_priority();
};
