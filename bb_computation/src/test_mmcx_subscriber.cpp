
/**
   \file test_mmcx_subscriber.cpp
   \brief Test host node for the MMCX cue integration model.

   A host node which was used to test the very early implementation
   of the cue integration model given by MMCX. This model is no
   longer up-to-date and should not be used but is included in the
   codebase for completeness. This file is also included so that the
   interaction between this node and the cue manager can be seen.

   \note This node is not currently built but can be enabled by 
   modifying CMakeLists.txt.

   \warning If bb_locomotion cmd_vel_service is running then starting
   this node will cause the robot to move. The current working status
   of this node is unknown.
*/

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>
#include <tf/transform_datatypes.h>
#include "std_msgs/String.h"

#include "bb_computation/mmcx_model.hpp"

#include "bb_util/velocity.h"
#include "bb_util/cue_list.h"
#include "bb_util/cue_msg.h"
#include "bb_util/cue.hpp"
#include "bb_util/vector.hpp"
#include "bb_util/vec2d_msg.h"
#include "bb_util/encoding_status.h"


#define GOAL_ANGLE -2.0
#define TRAVERSE_TIME 30

// Init CX
MMCX mmcx = MMCX(2);

// Broadcast CX status
ros::Publisher pub;

// Send movement commands to the update_velocity service
ros::ServiceClient client;

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
   Cue list callback. Receive current compass information from
   multiple cues, formatted by the cue manager. Pass these to the MMCX
   and generate a motor command. The motor command requests will
   always be sent.

   \warning If bb_locomotion cmd_vel_service is running, then starting
   this node will cause the robot to move.

   \param cue_msg A bb_util::cue_list representing a list of bb_util::cue_msgs.
*/
void cue_list_callback(const bb_util::cue_list::ConstPtr& cue_list){
  std::vector<bb_util::cue_msg> cues_from_msg = cue_list->cues;
  std::vector<bb_util::Cue> cues;

  for (int i = 0; i < cues_from_msg.size(); ++i){
    cues.push_back(bb_util::Cue::toCue(cues_from_msg[i]));}

  double CXMotor = mmcx.input(cues, 0);

  // For now, only update the angular velocity.
  bb_util::velocity vel_msg;
  vel_msg.request.angular = (CXMotor < 0) ? 0.5 : -0.5;
  vel_msg.request.linear = 0;
  client.call(vel_msg);

  // Retrieve cue encodings. This allows the vector encoded by each TL and the CL1
  // population to be published and plotted by bb_graphics. 
  std::vector<bb_util::Vec2D> encodings;
  bb_util::encoding_status encoding_msg = mmcx.get_cue_encoding(encodings);
  pub.publish(encoding_msg);
}

int main(int argc, char **argv){
  ros::init(argc, argv, "test_mmcx_subscriber");
  ros::NodeHandle n;

  // ROS networking

  // Get cue - Sub to visual cues
  ros::Subscriber sub = n.subscribe("cue_list", 1000, cue_list_callback);

  // Request velocity changes from velocity service
  client = n.serviceClient<bb_util::velocity>(bb_util::defs::UPDATE_VELOCITY);

  // Set up publisher for CX
  pub = n.advertise<bb_util::encoding_status>(bb_util::defs::MMCX_ENCODING, 1);

  ROS_INFO("Running...");

  ros::spin();

  return 0;
}
