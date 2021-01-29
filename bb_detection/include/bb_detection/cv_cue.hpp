#pragma once

/**
   @file cv_cue.h
   @brief Provides a CueFromCV display class.

   Provides a wrapper class which provides the functionality which
   allows cue detection from an openCV frame context, display, and
   translation to a standard bb_util::Cue.
*/

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>

#include <opencv2/opencv.hpp>

#include "bb_util/cue.hpp"
#include "bb_util/vector.hpp"

/**
   Wraps display functionality in a class for convenience.

   @todo Update the name of this class to be more reflective of its purpose.
   Would suggest "CueFromCVDisplay", it may be more prudent to remove this
   entirely and add more comprehensive visualisation utilities.
*/

namespace bb_detection{
  class CueFromCV {
  private:
    // This is just used to display the frame in colour
    // as it allows us to draw the cue vector in red.
    cv::Mat colour_frame; /**< The cv::Mat onto which we want to project the */

    //CueFromCV coordinates within the frame
    double frame_x; /**< X coordinate within the frame */
    double frame_y; /**< Y coordinate within the frame */

    //CueFromCV coordinates
    double x; /**< X translated from polar representation */
    double y; /**< Y translated from polar representation */

    //Polar coordinates
    double r; /**< Magnitude*/
    double theta; /**< Azimuth */

    //Image centre
    double centre_x; /**< Image centre X */
    double centre_y; /**< Image centre Y*/

    double max_cue_vector_length; /**< The maximum length a cue vector could be.

    /** Ensure all cue coordinate information is internally consistent. */
    void updateInternalRepresentation();

  public:
    //Ctor

    /**
       Constructor
       @param location cv::Point denoting the position of the cue
       @param frame_ref cv::Mat used to determine display Mat properties
    */
    CueFromCV(cv::Point location, const cv::Mat& frame_ref);

    /**
       Draw the cue vector on an openCV frame.
       @param frame The cv::frame on which the cue will be drawn.
    */
    cv::Mat& drawCueVectorOnFrame(cv::Mat &frame);

    /**
       Update the cue coordinates with respect to the frame.
       @param frame_x_coord The new x coordinate.
       @param frame_y_coord The new y coordinate.
    */
    void updateFrameCoordinates(double frame_x_coord, double frame_y_coord);

    /**
       Update the cue coordinates with respect to the frame.
       @param location cv::Point defining the new x/y position of the cue.
    */
    void updateFrameCoordinates(cv::Point location);

    /** Get the strength (polar magnitude) of the cue. */
    double strength();

    /** Get the direction (polar angle) of the cue. */
    double direction();

    /** Translate to system standard Cue format */
    bb_util::Cue toSystemCue();
    bb_util::Cue toSystemCue(double sensitivity);

  };
}
