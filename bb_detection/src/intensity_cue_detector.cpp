/**
   @file mono_directional_prompt_node.cpp
   @brief Compute single image-wide cue from cropped image.

   Available cues are currently brightest vector and centroid vector.
   Current relationship with IntensityCueManager and Cue classes is overly
   convoluted and should be simplified in the future.
*/

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/String.h>

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <sstream>
#include <cmath>
#include <iostream>


#include "bb_detection/argparse.h"
#include "bb_detection/cv_cue.hpp"

#include "bb_util/bb_util.h"
#include "bb_util/cue.hpp"

//Global, bad but should be safe in this case
argparse::ArgumentParser parser("Parser");

image_transport::ImageTransport *it; /**< ROS image transport layer*/
image_transport::Subscriber sub; /**< Image subscription node. */
ros::Publisher pub; /**< Publisher, publishing cue information. */

ros::NodeHandle *nhp;
double calibration_offset = 0;

bool initParser(argparse::ArgumentParser &parser, int argc, char **argv){
  //Global, all bb_computation nodes should have these options.
  parser.add_argument()
    .names({"-v", "--video"})
    .description("Enable video output for this node.")
    .required(false);

  parser.add_argument()
    .names({"-s", "--subscribe"})
    .description("Set the subscription topic.")
    .required(true);

  parser.add_argument()
    .names({"-p", "--publish"})
    .description("Set the publication topic.")
    .required(false);

  parser.add_argument()
    .names({"-n", "--name"})
    .description("Set the node name.")
    .required(true);

  // Node-specific
  parser.add_argument()
    .names({"-m", "--method"})
    .description("Define a method for determining a directional prompt.")
    .required(true);

  parser.enable_help();

  auto err = parser.parse(argc, const_cast<const char**>(argv));
  if (err) {
    std::stringstream ss;
    ss << err;
    ROS_ERROR("argparse error: %s", ss.str().c_str());
    return false;
  }

  return true;
}

// Calibration update callback
void calibrationNotifyCallback(const std_msgs::String::ConstPtr& msg){
  double current = calibration_offset;
  nhp->param(bb_util::params::CALIBRATION_INTENSITY_OFFSET,
             calibration_offset,
             current);
  ROS_INFO("CB: %lf", calibration_offset);
}

// BV Callback
void brightestVectorCallback(const sensor_msgs::ImageConstPtr& msg){
  cv::Mat frame;

  try{
    frame = cv_bridge::toCvCopy(msg, "mono8")->image;
  } catch (cv_bridge::Exception &e){
    ROS_ERROR("cv_bridge error: %s", e.what());
    ROS_INFO("This node should be subscribed to a "
             "topic producing grayscale (mono8) images");
  }

  double minVal=0, maxVal=0;
  cv::Point minLoc, maxLoc;

  //Find brightest point and mark  - Note if this isn't working well
  //try enabling gaussian smoothing.
  cv::minMaxLoc(frame, &maxVal, &maxVal, &minLoc, &maxLoc);

  const cv::Mat& frame_ref = frame;
  bb_detection::CueFromCV cv_cue(maxLoc, frame_ref);

  if (parser.exists("video")){
    ROS_INFO("Cue, (direction, magnitude): (%lf, %lf)",
             cv_cue.direction(),
             cv_cue.strength());

    // Mark brightest point on the frame for printing
    cv::Mat& colour_frame = cv_cue.drawCueVectorOnFrame(frame);
    bb_util::vision::imshow("Brightest point", colour_frame);
  }

  pub.publish(bb_util::Cue::toMsg(cv_cue.toSystemCue()));
}

// CV Callback
void centroidVectorCallback(const sensor_msgs::ImageConstPtr& msg){
  cv::Mat frame;
  try{
    frame = cv_bridge::toCvCopy(msg, "mono8")->image;
  } catch (cv_bridge::Exception &e){
    ROS_ERROR("cv_bridge error: %s", e.what());
    ROS_INFO("This node should be subscribed to a "
             "topic producing grayscale (mono8) images");
  }

  // I originally wanted to unwrap this and manually compute everything
  // Ramsey uses cv::moments() to compute this but I'm unsure of the validity of
  // this. I'll use it here to test it.
  // He also applies a spectral trasform to the image that I'm unsure of
  // spectrum = (np.round(((bgr[:,:,1] + bgr[:,:,2]) / bgr.sum(2).astype(float)) * 255.)).astype('uint8')
  // Best I can tell this is taking an average of the blue and green channels
  // and reducing this to a single "grey" channel.
  cv::Moments frame_moments = cv::moments(frame);

  cv::Point centre_of_mass(
                           frame_moments.m10 / frame_moments.m00, // CoM x-coord
                           frame_moments.m01 / frame_moments.m00  // CoM y-coord
                           );

  // Because I can't be bothered fighting with CMake
  const cv::Mat& frame_ref = frame;
  bb_detection::CueFromCV cv_cue(centre_of_mass, frame_ref);

  if (parser.exists("video")){
    cv::Mat &colour_frame = cv_cue.drawCueVectorOnFrame(frame);
    bb_util::vision::imshow("Centroid Vector", colour_frame);
  }

  pub.publish(bb_util::Cue::toMsg(cv_cue.toSystemCue()));
}

int main(int argc, char **argv){
  // Parser initialisation
  if (!initParser(parser, argc, argv)) return -1;

  // Parser decode
  if (parser.exists("help")){
    parser.print_help();
    return 0;
  }

  // If parser option defined, use it, else default.
  // publish to
  std::string pub_topic =
    parser.exists("publish") ?
    parser.get<std::string>("publish") :
    bb_util::defs::INTENSITY_CUE_TOPIC;

  // subscribe to
  std::string sub_topic = parser.get<std::string>("subscribe");

  // name
  std::string node_name = parser.get<std::string>("name");

  // method
  std::string method = parser.get<std::string>("method");

  ROS_INFO("Node information: ");
  ROS_INFO("Node name: %s", node_name.c_str());
  ROS_INFO("Subscribing to: %s", sub_topic.c_str());
  ROS_INFO("Publishing to: %s", pub_topic.c_str());
  ROS_INFO("Cue: %s", method.c_str());

  // ROS init
  ros::init(argc, argv, node_name.c_str());
  ros::NodeHandle n;
  nhp = &n; // Init global pointer for callback usage.

  // Check for calibration data
  n.param<double>(bb_util::params::CALIBRATION_INTENSITY_OFFSET,
                  calibration_offset,
                  -1);

  void (*imageCallback) (const sensor_msgs::ImageConstPtr& msg);

 // Determine which callback function this class instance should use.
  if (method == "bv") {
    imageCallback = &brightestVectorCallback;
  } else if (method == "cv") {
    imageCallback = &centroidVectorCallback;
  } else {
    ROS_ERROR("Unrecognised cue type %s", method.c_str());
    ROS_INFO("Supported methods: bv, cv");
    exit(-1);
  }

  // Plumb up the ROS subscriber and publisher
  it = new image_transport::ImageTransport(n);
  pub = n.advertise<bb_util::cue_msg>(pub_topic, 1000);
  sub = it->subscribe(sub_topic, 1, imageCallback);

  ros::Subscriber calibration_sub =
    n.subscribe(bb_util::defs::CALIBRATION_NOTIFY_TOPIC,
                1000,
                calibrationNotifyCallback);

  ros::spin();

  return 0;
}
