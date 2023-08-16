#include "bb_detection/cv_cue.hpp"

/**
   \file cue.cpp
   \brief Provides an implementation of the Cue display class.
*/

namespace bb_detection{
  /**
     \brief Constructor
     \param location The location of the cue within the frame.
     \param frame_ref A reference for the image frame onto which to draw the cue vector.
   */
  CueFromCV::CueFromCV(cv::Point location, const cv::Mat& frame_ref) {
    // Use const frame reference to determine the frame
    // centre without modifying the frame itself.
    double frame_centre_x = frame_ref.cols / 2;
    double frame_centre_y = frame_ref.rows / 2;
    frame_x = location.x;
    frame_y = location.y;
    centre_x = frame_centre_x;
    centre_y = frame_centre_y;

    // Eucliden distance from the centre-left of the frame to the centre.
    max_cue_vector_length =
      sqrt(centre_x*centre_x);

    this->updateInternalRepresentation();
  }

  /* Private */
  /**
     Ensure the internal cue representation stays consistent
   */
  void CueFromCV::updateInternalRepresentation(){
    x = frame_x - centre_x;
    y = frame_y - centre_y;
    r = sqrt(x*x + y*y);
    theta = atan2(y, x);
  }

  /* Public */

  /**
     Draw the cue vector on an openCV frame
     \param frame The frame reference onto which to draw the cue vector
     \return A colour image with the vector drawn in red.
  */
  cv::Mat& CueFromCV::drawCueVectorOnFrame(cv::Mat &frame){
    cv::Mat &colour = colour_frame;
    cv::cvtColor(frame, colour, cv::COLOR_GRAY2RGB);
    cv::Point origin(centre_x, centre_y);
    cv::Point location(frame_x, frame_y);
    cv::arrowedLine(colour, origin, location, cv::Scalar(0,0,255));
    return colour;
  }

  /**
   Update the coordinates of the cue.

   \param x The x coordinate from the left of the frame
   \param y The y coordinate from the top of the frame
  */
  void CueFromCV::updateFrameCoordinates(double frame_x_coord, double frame_y_coord){
    frame_x = frame_x_coord;
    frame_y = frame_y_coord;

    // Transform coordinates to central axes, compute polar coordinates.
    this->updateInternalRepresentation();
  }

  /**
     Update the coordinates of the cue using a cv::Point.
     \param location A cv::Point with the x and y coordinates of the cue.
  */
  void CueFromCV::updateFrameCoordinates(cv::Point location){
    frame_x = location.x;
    frame_y = location.y;

    // Transform coordinates to central axes, compute polar coordinates.
    this->updateInternalRepresentation();
  }

  /** Get the magnitude of the cue vector. */
  double CueFromCV::strength(){ return r; }
  /** 
      Get the direction of the cue vector. 
      \warning The angular convention here is not known. From the code
      it appears to be the standard anti-clockwise from the x-axis but
      if this is important to your application, you should double check.
  */
  double CueFromCV::direction(){ return theta; }


  /**
     Translate this OpenCV cue into a bb_util::Cue
     \return An equivalent bb_util::Cue
   */
  bb_util::Cue CueFromCV::toSystemCue(){
    std::string type = "intensity";
    double sensitivity = 1;
    double contrast = this->r / max_cue_vector_length; // Scale between 0:1
    contrast = contrast > 1.1 ? 0 : contrast;

    // Cue theta is expected to be in radians.
    return bb_util::Cue(type,
                        sensitivity,
                        contrast,
                        this->theta
                        );
  }

  /** 
      Translate this OpenCV cue into a bb_util::Cue but define
      the sensitivity.
      \param sensitivity The sensitivity to this cue
      \return An equivalent bb_util::Cue
   */
  bb_util::Cue CueFromCV::toSystemCue(double sensitivity){
    std::string type = "intensity";
    double contrast = this->r / max_cue_vector_length; // Scale between 0:1
    contrast = contrast > 1.1 ? 0 : contrast;
    
    // Cue theta is expected to be in radians.
    return bb_util::Cue(type,
                        sensitivity,
                        contrast,
                        this->theta
                        );
  }


}
