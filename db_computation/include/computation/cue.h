#pragma once

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>

#include <opencv2/opencv.hpp>

#include "computation/cue_vector.h"

class Cue {
private:
  // This is just used to display the frame in colour
  // as it allows us to draw the cue vector in red.
  cv::Mat colour_frame;

  //Cue coordinates within the frame
  double frame_x;
  double frame_y;

  //Cue coordinates
  double x;
  double y;

  //Polar coordinates
  double r;
  double theta;

  //Image centre
  double centre_x;
  double centre_y;

  //
  // Make sure the internal cue representations are internally consistent
  //
  void updateInternalRepresentation();

public:
  //Ctor
  Cue(cv::Point location, const cv::Mat& frame_ref);

  //
  // Draw the cue vector on an openCV frame
  //
  cv::Mat& drawCueVectorOnFrame(cv::Mat &frame);

  //
  // Update the coordinates of the cue.
  // Arguments are expected to be with respect to the image
  //
  void updateFrameCoordinates(double frame_x_coord, double frame_y_coord);

  //
  // Update the coordinates of the cue.
  // Arguments are expected to be with respect to the image
  //
  void updateFrameCoordinates(cv::Point location);

  //
  // Getters
  //
  double strength();
  double direction();

  // Translate to a ROS message.
  computation::cue_vector toMessage();
};
