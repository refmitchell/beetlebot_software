#pragma once

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <opencv2/opencv.hpp>

#include "db_util/db_util.h"
#include "computation/cue.h"

#include "computation/cue_vector.h"


/*
  IntensityCueManager provides utilities for receiving a mono8 frame,
  computing a Cue from it using the method defined in constructing it,
  then publishing the Cue information to a pre-determined topic.
*/

class IntensityCueManager {
private:
  Cue cue();
  image_transport::ImageTransport *it;
  image_transport::Subscriber sub;
  ros::Publisher pub;
  bool video;

  // Callback function pointer
  void (IntensityCueManager::*imageCallback) (const sensor_msgs::ImageConstPtr& msg);

public:
  //Ctor
  IntensityCueManager(ros::NodeHandle n,
               const std::string sub_topic,
               const std::string pub_topic,
               const std::string method,
               const bool video
             );

  ~IntensityCueManager();

  //IMAGE CALLBACK FUNCTIONS
  void brightestVectorCallback(const sensor_msgs::ImageConstPtr& msg);
  void centroidVectorCallback(const sensor_msgs::ImageConstPtr& msg);


};
