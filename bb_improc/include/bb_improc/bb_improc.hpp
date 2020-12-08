#pragma once

/**
   @file bb_improc.hpp
   @brief Provides internal utilities required by  multiple improc nodes.

   This header can be used to provide any classes or utilities required by the
   image processing package.
 */

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>

/**
  @brief A wrapper class for the pub/sub "link" required for the image
         processing pipeline design.

  The image processing pipeline is designed to be highly modular and flexible.
  Nodes can (to a degree) be moved about in their position in the chain so long
  as they subscribe to the correct image type (mono, rgb etc.) This wrapper class
  makes this chaining possible, allowing each node to function as a publisher and
  subscriber with common properties.
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
  /**
     All image processing nodes will receive an image so they must have this
     callback; each node may define its own implementation, however.
  */
  void imageCallback(const sensor_msgs::ImageConstPtr& msg);

  /**
     Constructor.
     @param n The ros NodeHandle required to initialise the publisher and 
              subscriber.
     @param subscription The name of the topic we will listen to.
     @param publication The name of the topic we wil publish to.
   */
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

  /**
     Explicit destructor required to ensure the image transport object gets 
     deleted.
   */
  ~ImageProcessingLink(){
    delete it;
  }
};
