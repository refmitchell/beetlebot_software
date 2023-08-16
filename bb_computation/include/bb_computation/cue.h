#pragma once

/**
   \file cue.h
   \brief Provides a Cue display class.

   Provides a wrapper class which allows a cue vector to be
   overlayed on an OpenCV matrix for testing/display purposes.

   \todo Update the name of this class to be more reflective of its purpose.
         Would suggest "CueDisplay", it may be more prudent to remove this
         entirely and add more comprehensive visualisation utilities.
 */

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>

#include <opencv2/opencv.hpp>

#include "bb_computation/cue_vector.h"

/**
   \brief Provides a wrapper class which allows a cue vector to be
   overlayed on an OpenCV matrix for testing/display purposes.

   \warning This class is very poorly named and clashes with 
*/
class Cue {
private:
  // This is just used to display the frame in colour
  // as it allows us to draw the cue vector in red.
  cv::Mat colour_frame; /**< The cv::Mat onto which we want to project the */

  //Cue coordinates within the frame
  double frame_x; /**< X coordinate within the frame */
  double frame_y; /**< Y coordinate within the frame */

  //Cue coordinates
  double x; /**< X translated from polar representation */
  double y; /**< Y translated from polar representation */

  //Polar coordinates
  double r; /**< Magnitude*/
  double theta; /**< Azimuth */

  //Image centre
  double centre_x; /**< Image centre X */
  double centre_y; /**< Image centre Y*/

  /** Ensure all cue coordinate information is internally consistent. */
  void updateInternalRepresentation();

public:
  //Ctor

  /**
     Constructor
     \param location cv::Point denoting the position of the cue
     \param frame_ref cv::Mat used to determine display Mat properties
   */
  Cue(cv::Point location, const cv::Mat& frame_ref);

  /**
      Draw the cue vector on an openCV frame.
      \param frame The cv::frame on which the cue will be drawn.
   */
  cv::Mat& drawCueVectorOnFrame(cv::Mat &frame);

  /**
     Update the cue coordinates with respect to the frame.
     \param frame_x_coord The new x coordinate.
     \param frame_y_coord The new y coordinate.
  */
  void updateFrameCoordinates(double frame_x_coord, double frame_y_coord);

  /**
     Update the cue coordinates with respect to the frame.
     \param location cv::Point defining the new x/y position of the cue.
   */
  void updateFrameCoordinates(cv::Point location);

  /** Get the strength (polar magnitude) of the cue. */
  double strength();

  /** Get the direction (polar angle) of the cue. */
  double direction();

  /** Translate to a custom ROS message format. */
  bb_computation::cue_vector toMessage();
};
