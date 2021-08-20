#include "bb_detection/cv_cue.hpp"

/**
   @file cue.cpp
   @brief Provides an implementation of the Cue display class.
*/

  //
  // Cue class implementation
  //

  //
  // Ctor
  //
namespace bb_detection{
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

  //
  // Private method
  //
  void CueFromCV::updateInternalRepresentation(){
    x = frame_x - centre_x;
    y = frame_y - centre_y;
    r = sqrt(x*x + y*y);
    theta = atan2(y, x);
  }

  //
  // Public methods
  //

  //
  // Draw the cue vector on an openCV frame
  //
  cv::Mat& CueFromCV::drawCueVectorOnFrame(cv::Mat &frame){
    cv::Mat &colour = colour_frame;
    cv::cvtColor(frame, colour, cv::COLOR_GRAY2RGB);
    cv::Point origin(centre_x, centre_y);
    cv::Point location(frame_x, frame_y);
    cv::arrowedLine(colour, origin, location, cv::Scalar(0,0,255));
    return colour;
  }

  //
  // Update the coordinates of the cue.
  // Arguments are expected to be with respect to the image
  //
  void CueFromCV::updateFrameCoordinates(double frame_x_coord, double frame_y_coord){
    frame_x = frame_x_coord;
    frame_y = frame_y_coord;

    // Transform coordinates to central axes, compute polar coordinates.
    this->updateInternalRepresentation();
  }

  //
  // Update the coordinates of the cue.
  // Arguments are expected to be with respect to the image
  //
  void CueFromCV::updateFrameCoordinates(cv::Point location){
    frame_x = location.x;
    frame_y = location.y;

    // Transform coordinates to central axes, compute polar coordinates.
    this->updateInternalRepresentation();
  }

  //
  // Getters; only publically accessible features should be the strength and
  // the direction.
  //
  double CueFromCV::strength(){ return r; }
  double CueFromCV::direction(){ return theta; }

  //
  // Translate into the system standard cue representation
  //
  bb_util::Cue CueFromCV::toSystemCue(){
    std::string type = "intensity";
    double sensitivity = 1;
    double reliability = this->r / max_cue_vector_length; // Scale between 0:1
    reliability = reliability > 1.1 ? 0 : reliability;

    // Cue theta is expected to be in radians.
    return bb_util::Cue(type,
                        sensitivity,
                        reliability,
                        this->theta
                        );
  }

  bb_util::Cue CueFromCV::toSystemCue(double sensitivity){
    std::string type = "light";
    double reliability = this->r / max_cue_vector_length; // Scale between 0:1
    reliability = reliability > 1.1 ? 0 : reliability;
    
    // Cue theta is expected to be in radians.
    return bb_util::Cue(type,
                        sensitivity,
                        reliability,
                        this->theta
                        );
  }


}
