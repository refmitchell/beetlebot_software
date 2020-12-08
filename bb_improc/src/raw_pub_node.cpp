#define DEBUG 0

/**
   @file raw_pub_node.cpp
   @brief To be removed.

   Unsure as to whether or not there is another capture example backed up
   anywhere so I've left this in the package as a reference. Just in case...
*/

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "bb_improc/argparse.h"



int main(int argc, char **argv){
  // ROS Node init
  ros::init(argc, argv, "raw_pub_node");

  // Instantiate node handle
  ros::NodeHandle n;

  // Set up image transport publisher
  image_transport::ImageTransport it(n);
  image_transport::Publisher pub = it.advertise("frames", 1);

  // Start OpenCV video capture
  cv::VideoCapture cap;
  if(!cap.open(0)) {
    ROS_ERROR("OpenCV capture failed to open.");
    return -1;
  }

  // Capture convert and publish images
  int count = 0;
  while(ros::ok()){
    // Print some useful info
    std::stringstream info;
    info << "Publishing frame: " << count++;
    ROS_INFO("%s", info.str().c_str());

    // Get OpenCV frame
    cv::Mat frame;

    cap >> frame;
    if (frame.empty()){
      ROS_INFO("Camera frame empty, exiting.");
      return 0;
    }

    #if DEBUG
    // If we're in debug mode allow the node to create a
    // video window.
    cv::imshow("Publisher - Video", frame);
    if (cv::waitKey(10) == 27) break;
    #endif

    // Convert to sensor_msgs/Image
    sensor_msgs::ImagePtr img_msg =
      cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();

    // Publish our camera image - want to publish them as frequently as we can.
    pub.publish(img_msg);
  }

  return 0;
}
