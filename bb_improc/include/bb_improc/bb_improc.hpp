#pragma once

/**
   \file bb_improc.hpp
   \brief Provides the specification for the ImageProcessingLink class.

   \remark This was originally intended to provide overall utilities
   for the improc class but these weren't really needed. As a result
   this provides just one class. It could be renamed.
 */

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>

/**
  \brief A wrapper class for the pub/sub "link" required for the image
         processing pipeline design.

  The image processing pipeline is designed to be modular and flexible.
  Nodes can (to a degree) be moved about in their position in the chain so long
  as they subscribe to the correct image type (mono, rgb etc.) This wrapper class
  makes this chaining possible, allowing each node to function as a publisher and
  subscriber with common properties.

  \remark In hindsight this is overengineered. In large part I think this class exists
  to avoid making global publishers and subscribers within the nodes, but given the
  simplicity of the nodes, global variables seem much more acceptable.
*/
class ImageProcessingLink {
private:
  image_transport::ImageTransport *it;
  image_transport::Publisher pub;
  image_transport::Subscriber sub;

public:
  /**
     All image processing nodes will receive an image so they must have this
     callback. Each node defines its own implementation.
  */
  void imageCallback(const sensor_msgs::ImageConstPtr& msg);

  /**
     Constructor.
     \param n The ros NodeHandle required to initialise the publisher and 
              subscriber.
     \param subscription The name of the topic we will listen to.
     \param publication The name of the topic we wil publish to.
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
