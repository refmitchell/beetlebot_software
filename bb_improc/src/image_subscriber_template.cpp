#define DEBUG 1

/**
   @file image_subscriber_template.cpp
   @brief Minimum working example of an image subscriber.

   Retained for reference, not in active use.
*/



#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

void imageCallback(const sensor_msgs::ImageConstPtr& msg){
  cv::imshow("Subscriber - Video", cv_bridge::toCvShare(msg)->image);
  cv::waitKey(10);
  // try {
  //   cv::imshow("Subscriber - Video", cv_bridge::toCvShare(msg)->image);
  //   cv::waitKey(10);
  // } catch(cv_bridge::Exception& e) {
  //   ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  // }
}

int main(int argc, char **argv){
  ros::init(argc, argv, "image_subscriber_template");
  ros::NodeHandle n;
  image_transport::ImageTransport it(n);
  cv::namedWindow("Subscriber - Video");
  cv::startWindowThread();
  image_transport::Subscriber sub =
    it.subscribe("gray_frames", 1, imageCallback);
  ros::spin();
  cv::destroyWindow("Subscriber - Video");
  return 0;
}
