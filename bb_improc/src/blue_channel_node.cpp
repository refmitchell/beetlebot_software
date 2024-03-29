/**
   \file blue_channel_node.cpp
   \brief Extracts and publishes the blue channel from an rgb image.
*/

#include <cmath>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "bb_improc/bb_improc.hpp"
#include "bb_util/bb_util.h"
#include "bb_util/argparse.h"

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
  cv::Mat frame = cv_bridge::toCvCopy(msg, "bgr8")->image;
  cv::Mat channels[3];
  split(frame,channels);

  // Debug imshow
  if (parser.exists("video")){
    bb_util::vision::imshow("Blue channel", channels[0]);
  }

  // Translate into ROS format.
  sensor_msgs::ImagePtr repub =
    cv_bridge::CvImage(std_msgs::Header(), "mono8", channels[0]).toImageMsg();
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
    "blue_frames";

  // subscribe to
  std::string sub_topic =
    parser.exists("subscribe") ?
    parser.get<std::string>("subscribe") :
    "blurred_frames";

  // name
  std::string node_name =
    parser.exists("name") ?
    parser.get<std::string>("name") :
    "blue_channel_node";

  ROS_INFO(" = IPL configuration = ");
  ROS_INFO("Node name: %s", node_name.c_str());
  ROS_INFO("Subscribing to: %s", sub_topic.c_str());
  ROS_INFO("Publishing to: %s", pub_topic.c_str());

  ros::init(argc, argv, node_name.c_str());
  ros::NodeHandle n;

  ImageProcessingLink ipl(n, sub_topic, pub_topic);

  ros::spin();

  return 0;
}
