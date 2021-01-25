#include "bb_detection/cv_cue.h"

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
CueFromCV::CueFromCVFromCV(cv::Point location, const cv::Mat& frame_ref) {
  // Use const frame reference to determine the frame
  // centre without modifying the frame itself.
  double frame_centre_x = frame_ref.cols / 2;
  double frame_centre_y = frame_ref.rows / 2;
  frame_x = location.x;
  frame_y = location.y;
  centre_x = frame_centre_x;
  centre_y = frame_centre_y;

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
cv::Mat& CueFromCV::drawCueFromCVVectorOnFrame(cv::Mat &frame){
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
// Translate the cue into a ROS message
//
bb_computation::cue_vector CueFromCV::toMessage(){
  bb_computation::cue_vector msg;
  msg.magnitude = this->strength();
  msg.theta = this->direction();
  return msg;
}

