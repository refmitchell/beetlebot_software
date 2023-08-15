/**
   \file dummy_cue.cpp
   \brief Provdes dummy input signals which can be used by the robot.
   \author Robert Mitchell

   Designed for testing purposes. Cues can be synthesised with a
   magnitude, direction, and "sensitivity". This node is designed
   to be simple and general. They are anonymised such that any number
   can run (so long as they publish to different topics).
 */


#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>

#include <sstream>

#include "bb_util/bb_util.h"
#include "bb_util/cue.hpp"
#include "bb_util/cue_msg.h"
#include "bb_util/argparse.h"

argparse::ArgumentParser parser = argparse::ArgumentParser("Parser");
double current_yaw = 0;
std::string *type_p;
double calibration_offset = 0;

ros::NodeHandle *nhp;

/**
   Initialises the argument parser defined in bb_util/argparse.h
   \param parser The argparse::ArgumentParser object.
   \param argc Passthrough argc from the command line.
   \param argv Passthrough argv from the command line.
   \return true on success
*/
bool initParser(argparse::ArgumentParser &parser, int argc, char **argv){
  parser.add_argument()
    .names({"-t", "--type"})
    .description("Set the dummy cue type")
    .required(false);

  parser.add_argument()
    .names({"-s", "--sensitivity"})
    .description("Set the agent's sensitivity to this cue.")
    .required(false);

  parser.add_argument()
    .names({"-m", "--magnitude"})
    .description("Set the \"strength\" of the dummy cue.")
    .required(true);

  parser.add_argument()
    .names({"-a", "--angle"})
    .description("Set the angle for this cue (in degrees).")
    .required(true);

  parser.add_argument()
    .names({"-p","--pub"})
    .description("Override the publication topic.")
    .required(false);

  parser.add_argument()
    .names({"--noupdate"})
    .description("Set flag to disable updates w.r.t. odometry.")
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
   Calibration callback. If a message is received on the calibration 
   topic ('calibration_notify'), then the calibration offsets will be
   updated. \b If the message type is `wind` or `intensity` then the
   offsets will be stored in the parameter server. Otherwise they
   will just be applied locally.

   \param msg The incoming ROS message

   \note The actual message content is ignored.
*/
void calibrationNotifyCallback(const std_msgs::String& msg){
  // Sensor calibration has been updated, re-read from parameter server
  std::string rosparam_id;
  if (*type_p == "wind_cue") {
    rosparam_id = bb_util::params::CALIBRATION_WIND_OFFSET;
  } else if (*type_p == "intensity_cue") {
    rosparam_id = bb_util::params::CALIBRATION_INTENSITY_OFFSET;
  } else {
    // Fail but let the node keep running
    ROS_ERROR("Calibration not supported for cue of type %s", type_p->c_str());
    return;
  }

  double current = calibration_offset;
  nhp->param(rosparam_id,
             calibration_offset,
             current);
}

/**
   Yaw message callback. Expects a message which expresses the yaw of the agent
   in radians.
   \param msg The ROS message.
   \note There is a node which provides exactly this function for the turtlebot.
   If the turtlebot core is running then, 
   \verbatim
   $ rosrun bb_util yaw
   \endverbatim
   Will generate the required messages.
*/
void yawCallback(const std_msgs::Float64& msg){
  current_yaw = msg.data;
}

/**
   Main loop
   \param argc Number of arguments
   \param argv The array of argument values.
   \note These parameters are passed through to the 3rd party argument 
   parser provided by bb_util/argparse.h.
*/
int main(int argc, char **argv){
  // Initialise the parser
  if (!initParser(parser, argc, argv)) return -1;

  if (parser.exists("help")){
    parser.print_help();
    return 0;
  }

  //
  // Parser decode
  //
  std::string type =
    parser.exists("type") ?
    parser.get<std::string>("type") :
    "dummy_cue";

  type_p = &type;

  double sensitivity =
    parser.exists("sensitivity") ?
    parser.get<double>("sensitivity") :
    1;

  double magnitude = parser.get<double>("magnitude");
  double angle = parser.get<double>("angle");
  angle = angle * (3.14159/180); // Deg to radian conversion: a * pi/180

  bool no_update =
    parser.exists("noupdate") ?
    true :
    false;

  // Cue object defined by the arguments
  bb_util::Cue cue(type, sensitivity, magnitude, angle);

  std::stringstream name;
  std::stringstream topic;

  name << "dummy_node";
  topic << type;

  std::string namestring = name.str();
  std::string topicstring =
    parser.exists("pub") ?
    parser.get<std::string>("pub") :
    topic.str();

  //
  // ROS initialisation and admin
  //
  ros::init(argc, argv, name.str(), ros::init_options::AnonymousName);
  ros::NodeHandle n;
  nhp = &n;
  ros::Publisher pub = n.advertise<bb_util::cue_msg>(topicstring, 1000);
  ros::Subscriber yaw_sub = n.subscribe("yaw", 1000, yawCallback);
  ros::Subscriber calibration_notify =
    n.subscribe(bb_util::defs::CALIBRATION_NOTIFY_TOPIC,
                1000,
                calibrationNotifyCallback);

  ROS_INFO("\nDummy cue specification:"
           "Angle: %lf\nMagnitude: %lf\nSensitivity: %lf\nTopic: %s\n",
           angle,
           magnitude,
           sensitivity,
           topicstring.c_str()
           );

  ros::Rate r(10);  // Publish the cue message at 10hz

  //
  // If you want to read pre-existing calibration information from an
  // active parameter server, then uncomment this code and recompile
  // the node.
  //
  // if (type == "wind_cue" || type == "intensity_cue"){
  //   auto param_id = type == "wind_cue" ?
  //     bb_util::params::CALIBRATION_WIND_OFFSET :
  //     bb_util::params::CALIBRATION_INTENSITY_OFFSET;
  //   calibration_offset = nhp->getParam(param_id,calibration_offset);
  // }
 
  while(ros::ok()){
    ros::spinOnce();
    // Update position w.r.t. robot odom
    double pub_angle = no_update ? angle : angle - current_yaw;
    pub_angle = pub_angle - calibration_offset; // Include calibration
    bb_util::Cue cue(type, sensitivity, magnitude, pub_angle);
    pub.publish(bb_util::Cue::toMsg(cue));
    r.sleep();
  }

  return 0;
}
