/**
   \file cue_calibration.cpp
   \brief Calibrate detection nodes relative to the IMU

   Used to calibrate the different supported cues. Functionally this aligns 
   the receptive fields of the TL neurons downstream. Unlike most nodes this is
   designed to run for one cycle then terminate; figure out the calibration
   parmeters, store them in the parameter server, then exit.

   \note This system was implemented prior to the neural model from Mitchell
   et al. (2023). The plastic connections between the R and E-PG neurons in that
   model effectively fill this role.

   \note
   If you want to add any cues to this calibration system then you need to add a
   callback and subscriber for the cue.

   \warning The calibration system was implemented quickly and not tested thoroughly.
   While it should work, there may be problems. 
 */

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>
#include <tf/transform_datatypes.h>
#include <std_msgs/String.h>
#include <iostream>

#include "bb_util/bb_util.h"
#include "bb_util/cue.hpp"
#include "bb_util/argparse.h"

argparse::ArgumentParser parser("Parser");

double global_yaw = 0;
double intensity_offset = 0;
double wind_offset = 0;
std::map<std::string, double> readings;

/**
   Initialise the argument parser.  
   \param parser The argument parser
   \param argc Argument count
   \param argv Argument values passed on the command line
   \return true on success
 */
bool initParser(argparse::ArgumentParser &parser, int argc, char **argv){
  parser.add_argument()
    .names({"-i", "--int_sub"})
    .description("Set the subscription topic for the light intensity cue."
                 "Should be whatever topic the intensity cue detector is "
                 "publishing to."
                 )
    .required(false);

  parser.add_argument()
    .names({"-w", "--wind_sub"})
    .description("Set the wind cue subscription topic.")
    .required(false);

  parser.enable_help();

  auto err = parser.parse(argc, const_cast<const char**>(argv));
  if (err){
    std::stringstream ss;
    ss << err;
    ROS_ERROR("argparse error: %s", ss.str().c_str());
    return false;
  }

  return true;
}

/**
   Callback for IMU information. Extracts the yaw from the turtlebot
   odometry message.
   \param odom_msg The odometry message
*/
void odomCallback(const nav_msgs::Odometry::ConstPtr& odom_msg){
  tf::Quaternion quat;
  tf::quaternionMsgToTF(odom_msg->pose.pose.orientation, quat);

  double roll, pitch, yaw;
  tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
  global_yaw = yaw;
}

/**
   Callback for intensity cue information
   \param msg The bb_util::cue_msg
*/
void intensityUpdateCallback(const bb_util::cue_msg::ConstPtr& msg){
  bb_util::cue_msg cue_msg = *msg;
  bb_util::Cue cue = bb_util::Cue::toCue(cue_msg);

  // If we do not have a reading for this cue, store one.
  if(!readings.count(cue.getType())) readings[cue.getType()] = cue.getTheta();

  // Apply offset based on first reading
  intensity_offset = global_yaw + readings[cue.getType()];//cue.getTheta();
  ROS_INFO("Yaw, reading, offset: %lf, %lf, %lf",
           global_yaw,
           readings[cue.getType()],
           intensity_offset);
}


/**
   Callback for wind information
   \param msg The bb_util::cue_msg
*/
void windUpdateCallback(const bb_util::cue_msg::ConstPtr& msg){
  bb_util::cue_msg cue_msg = *msg;
  bb_util::Cue cue = bb_util::Cue::toCue(cue_msg);

  // If we do not have a reading for this cue, store one.
  if(!readings.count(cue.getType())) readings[cue.getType()] = cue.getTheta();

  // Apply offset based on first reading
  wind_offset = global_yaw + readings[cue.getType()];//cue.getTheta();
}

int main(int argc, char**argv){
  // Parser initialisation
  if (!initParser(parser, argc, argv)) return -1;

  // Parser decode
  if (parser.exists("help")){
    parser.print_help();
    return 0;
  }
  
  // Determine the detection nodes for the various cues
  std::string wind_cue_sub_topic =
    parser.exists("wind_sub") ?
    parser.get<std::string>("wind_sub") :
    bb_util::defs::WIND_CUE_TOPIC;

  std::string intensity_cue_sub_topic =
    parser.exists("int_sub") ?
    parser.get<std::string>("int_sub") :
    bb_util::defs::INTENSITY_CUE_TOPIC;

  std::string node_name = "calibration";
  ros::init(argc, argv, node_name.c_str());
  ros::NodeHandle n;

  ros::Subscriber wind_direction_sub =
    n.subscribe(wind_cue_sub_topic.c_str(), 1000, windUpdateCallback);

  ros::Subscriber intensity_cue_sub =
    n.subscribe(intensity_cue_sub_topic.c_str(), 1000, intensityUpdateCallback);

  ros::Subscriber odom_sub =
    n.subscribe("/odom", 1000, odomCallback);

  ros::Publisher calibration_notify =
    n.advertise<std_msgs::String>(bb_util::defs::CALIBRATION_NOTIFY_TOPIC,1000);


  ros::Duration(0.5).sleep();

  //Clear existing calibration offsets
  n.setParam(bb_util::params::CALIBRATION_INTENSITY_OFFSET, 0);
  n.setParam(bb_util::params::CALIBRATION_WIND_OFFSET, 0);
  std_msgs::String notify_clear;
  notify_clear.data = "";
  calibration_notify.publish(notify_clear);

  ROS_INFO("Once calibration information has been set, the node will continue"
           " to update it every five seconds. Once cues are calibrated, this node"
           " can be stopped using CTRL-C. Calibration offsets are stored on the"
           " parameter server for as long as roscore is running.");
  
  while(ros::ok()){
    ros::spinOnce();
    n.setParam(bb_util::params::CALIBRATION_INTENSITY_OFFSET, intensity_offset);
    n.setParam(bb_util::params::CALIBRATION_WIND_OFFSET, wind_offset);
    std_msgs::String msg;
    msg.data = "";
    calibration_notify.publish(msg);
    ROS_INFO("IO: %lf | WO: %lf", intensity_offset, wind_offset);
    ros::Duration(5).sleep();
  }

  return 0;
}
