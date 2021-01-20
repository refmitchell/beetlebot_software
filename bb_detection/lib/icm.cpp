#include "bb_detection/icm.h"
#include "std_msgs/String.h" //DEBUG ONLY

/**
   @file icm.cpp
   @brief Implements the IntensityCueManager class.
*/
IntensityCueManager::IntensityCueManager(ros::NodeHandle n,
                                        const std::string sub_topic,
                                        const std::string pub_topic,
                                        const std::string method,
                                        const bool video
                                        ){
  this->video = video; // Display boolean

  // Determine which callback function this class instance should use.
  if (method == "bv") {
    imageCallback = &IntensityCueManager::brightestVectorCallback;
  } else if (method == "cv") {
    imageCallback = &IntensityCueManager::centroidVectorCallback;
  } else {
    ROS_ERROR("Unrecognised cue type %s", method.c_str());
    ROS_INFO("Supported reference methods: bv, cv");
    exit(-1);
  }

  // Plumb up the ROS subscriber and publisher
  it = new image_transport::ImageTransport(n);
  pub = n.advertise<bb_computation::cue_vector>(pub_topic, 1000);
  sub =
    it->subscribe(sub_topic, 1, IntensityCueManager::imageCallback, this);

}

//Dtor
IntensityCueManager::~IntensityCueManager(){
  delete it;
}


//
// IMAGE CALLBACK FUNCTIONS
//

//
// Compute a cue as the vector from the centre of the image to the brightest
// point in the image.
//
void IntensityCueManager::brightestVectorCallback(
                                const sensor_msgs::ImageConstPtr& msg
                                                  ){
  cv::Mat frame;

  try{
    frame = cv_bridge::toCvCopy(msg, "mono8")->image;
  } catch (cv_bridge::Exception &e){
    ROS_ERROR("cv_bridge error: %s", e.what());
    ROS_INFO("This node should be subscribed to a "
             "topic producing grayscale (mono8) images");
  }

  double minVal=0, maxVal=0;
  cv::Point minLoc, maxLoc;

  //Find brightest point and mark  - Note if this isn't working well
  //try enabling gaussian smoothing.
  cv::minMaxLoc(frame, &maxVal, &maxVal, &minLoc, &maxLoc);

  const cv::Mat& frame_ref = frame;
  Cue cue(maxLoc, frame_ref);

  if (this->video){
    ROS_INFO("Cue, (direction, magnitude): (%lf, %lf)",
             cue.direction(),
             cue.strength());

    // Mark brightest point on the frame for printing
    cv::Mat& colour_frame = cue.drawCueVectorOnFrame(frame);
    bb_util::vision::imshow("Brightest point", colour_frame);
  }

  this->pub.publish(cue.toMessage());
}

void IntensityCueManager::centroidVectorCallback(
                                const sensor_msgs::ImageConstPtr& msg
                                                 ){
  cv::Mat frame;
  try{
    frame = cv_bridge::toCvCopy(msg, "mono8")->image;
  } catch (cv_bridge::Exception &e){
    ROS_ERROR("cv_bridge error: %s", e.what());
    ROS_INFO("This node should be subscribed to a "
             "topic producing grayscale (mono8) images");
  }

  // I originally wanted to unwrap this and manually compute everything
  // Ramsey uses cv::moments() to compute this but I'm unsure of the validity of
  // this. I'll use it here to test it.
  // He also applys a spectral trasform to the image that I'm unsure of
  // spectrum = (np.round(((bgr[:,:,1] + bgr[:,:,2]) / bgr.sum(2).astype(float)) * 255.)).astype('uint8')
  // Best I can tell this is taking an average of the blue and green channels
  // and reducing this to a single "grey" channel.
  cv::Moments frame_moments = cv::moments(frame);

  cv::Point centre_of_mass(
                           frame_moments.m10 / frame_moments.m00, // CoM x-coord
                           frame_moments.m01 / frame_moments.m00  // CoM y-coord
                           );

  // Because I can't be bothered fighting with CMake
  const cv::Mat& frame_ref = frame;
  Cue cv_cue(centre_of_mass, frame_ref);

  if (this->video){
    cv::Mat &colour_frame = cv_cue.drawCueVectorOnFrame(frame);
    bb_util::vision::imshow("Centroid Vector", colour_frame);
  }

  this->pub.publish(cv_cue.toMessage());
}
