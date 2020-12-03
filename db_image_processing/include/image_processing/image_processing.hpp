#pragma once

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>

/*
  ImageProcessingLink describes the pub/sub setup required by all
  nodes in the image processing pipeline. The imageCallback method
  must be defined per-node and must call the publisher.
*/

class ImageProcessingLink {
private:
  image_transport::ImageTransport *it;
  image_transport::Publisher pub;
  image_transport::Subscriber sub;

  // TODO: could be nice to wrap this so the publisher is always called.
  // void imageCallbackWrapper(const sensor_msgs::ImageConstPtr& msg){
  //   imageCallback(msg);
  // }
public:
 void imageCallback(const sensor_msgs::ImageConstPtr& msg);

  ImageProcessingLink(
                      ros::NodeHandle n,
                      const std::string subscription,
                      const std::string publication
                      ){
    it = new image_transport::ImageTransport(n);
    pub = it->advertise(publication, 1);
    sub =
      it->subscribe(subscription, 1, &ImageProcessingLink::imageCallback, this);
  }

  ~ImageProcessingLink(){
    delete it;
  }
};
