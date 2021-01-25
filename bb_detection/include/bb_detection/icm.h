#pragma once

/**
   @file icm.h
   @brief Provides the IntensityCueManager class.
*/

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <opencv2/opencv.hpp>


/**
  @brief ICM class wraps the process of computing an intensity cue from an image.
  IntensityCueManager provides utilities for receiving a mono8 frame,
  computing a Cue from it using the method defined in constructing it,
  then publishing the Cue information to a pre-determined topic.
*/
class IntensityCueManager {
private:
  Cue cue();  /**< The computed Cue */
  image_transport::ImageTransport *it; /**< ROS image transport layer*/
  image_transport::Subscriber sub; /**< Image subscription node. */
  ros::Publisher pub; /**< Publisher, publishing cue information. */
  bool video;

  /**
     Cue computation callback function pointer; to allow multiple cue computation
     methods to be used within the same class implementation.

     @param msg The image message received from the visual processing pipeline.
  */
  void (IntensityCueManager::*imageCallback) (const sensor_msgs::ImageConstPtr& msg);

public:
  /**
     Constructor.
     @param n The ROS node handle required to initialise the pub/sub
            infrastructure.
     @param sub_topic The ROS topic to which the manager should subscribe.
     @param pub_topic The ROS topic to which the manager will publish cue info.
     @param method The method the manager should use to compute the cue from the
            image.
     @param video Indicates whether we want to display the video frames on screen.
   */
  IntensityCueManager(ros::NodeHandle n,
               const std::string sub_topic,
               const std::string pub_topic,
               const std::string method,
               const bool video
             );

  /**
      Destructor. Explicitly required as the image transport must be initialised
      new and deleted.
  */
  ~IntensityCueManager();

  /**
      Image callback implementation; compute the cue using the Brightest Vector
      method.

      @param msg The image message from which we should compute the cue.
   */
  void brightestVectorCallback(const sensor_msgs::ImageConstPtr& msg);

  /**
     Image callback implementation; compute the cue using the Centroid Vector
     method.

     @param msg The image message from which we should compute the cue.
  */
  void centroidVectorCallback(const sensor_msgs::ImageConstPtr& msg);
};
