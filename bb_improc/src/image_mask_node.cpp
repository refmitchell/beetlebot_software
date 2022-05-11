#define DEBUG 1

/**
   @file image_mask_node.cpp
   @brief Applies a circular mask to an RGB image.

   This is really meant to be first in the pipeline. Image capture is part
   of the robot-based software; the mask is applied to provide a circular
   "view-port". The only reason it is not applied by default is for flexibility.
*/


#include <cmath>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <sstream>

#include "bb_improc/bb_improc.hpp"

#include "bb_util/argparse.h"
#include "bb_util/bb_util.h"

//Global, bad but should be safe in this case
argparse::ArgumentParser parser("Parser");

bool initParser(argparse::ArgumentParser &parser, int argc, char **argv){
  //Global, all image_processing nodes should have these options.
  parser.add_argument()
    .names({"-v", "--video"})
    .description("Enable video output for this node.")
    .required(false);

  parser.add_argument()
    .names({"-s", "--subscribe"})
    .description("Override the subscription topic.")
    .required(false);

  parser.add_argument()
    .names({"-p", "--publish"})
    .description("Override the publication topic.")
    .required(false);

  parser.add_argument()
    .names({"-n", "--name"})
    .description("Override the node name.")
    .required(false);

  parser.add_argument()
    .names({"-r", "--viewport_radius"})
    .description("Set the radius of the circular viewport at the centre of the frame.")
    .required(false);
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

void ImageProcessingLink::imageCallback(const sensor_msgs::ImageConstPtr& msg){
  cv::Mat raw_frame = cv_bridge::toCvCopy(msg, "bgr8")->image;

  // MAX_RADIUS is determined by the frame resolution captured on the robot.
  // At the moment that resolution is 96x96; this can be configured by changing
  // raw_capture.cpp in the capture package on the turtlebot.
  int MAX_RADIUS = floor(raw_frame.rows/2);

  int radius =
    parser.exists("viewport_radius") ?
    parser.get<int>("viewport_radius")
    : MAX_RADIUS;

  // Check radius does not exceed max
  if (radius > MAX_RADIUS) {
    ROS_FATAL(
              "Viewport maximum radius is %d, please correct the radius.",
              MAX_RADIUS
              );
    exit(-1);
  }


  // Crop frame to a square with edge length 2*r
  int edge_length = radius * 2;

  // Bounding box top left x and y coordinates
  int roi_x = (raw_frame.cols / 2) - radius;
  int roi_y = (raw_frame.rows / 2) - radius;

  cv::Rect roi(roi_x, roi_y, edge_length, edge_length);
  cv::Mat frame = raw_frame(roi);


  // Set centre for circular mask
  cv::Point centre;
  centre.x = floor(frame.cols / 2);
  centre.y = floor(frame.rows / 2);

  // Define circular mask.
  cv::Mat mask;
  mask = cv::Mat::zeros(frame.rows, frame.cols, frame.type());
  circle(mask, centre, radius, cv::Scalar(255,255,255), -1);

  // Apply the binary mask.
  cv::bitwise_and(frame, mask, frame);

  // Image display
  if (parser.exists("video")){
    bb_util::vision::imshow("Image Mask Output", frame, cv::Size(1000,1000));
  }

  // Translate into ROS format.
  sensor_msgs::ImagePtr repub =
    cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
  this->pub.publish(repub);

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
    "masked_frames";

  // subscribe to
  std::string sub_topic =
    parser.exists("subscribe") ?
    parser.get<std::string>("subscribe") :
    "frames";

  // name
  std::string node_name =
    parser.exists("name") ?
    parser.get<std::string>("name") :
    "image_mask_node";

  ROS_INFO(" = IPL configuration = ");
  ROS_INFO("Node name: %s", node_name.c_str());
  ROS_INFO("Subscribing to: %s", sub_topic.c_str());
  ROS_INFO("Publishing to: %s", pub_topic.c_str());

  // ROS setup and IPL construction
  ros::init(argc, argv, node_name.c_str());
  ros::NodeHandle n;

  ImageProcessingLink ipl(n, sub_topic, pub_topic);

  ros::spin();

  return 0;
}
