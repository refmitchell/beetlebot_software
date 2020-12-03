#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

static const std::string OPENCV_WINDOW = "Image window";

class ImageConverter{
  ros::NodeHandle nh;
  image_transport::ImageTransport it;
  image_transport::Subscriber image_sub;
  image_transport::Publisher image_pub;

public:
  ImageConverter() : it(nh){
    image_sub = it.subscribe("/camera/image_raw",
                             1,
                             &ImageConverter::imageCb,
                             this);
    image_pub = it.advertise("/image_converter/output_video", 1);
    cv::namedWindow(OPENCV_WINDOW);
  }

  ~ImageConverter(){
    cv::destroyWindow(OPENCV_WINDOW);
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg){
    cv_bridge::CvImagePtr cv_ptr;
    try{
      cv_ptr = cv_bridge::toCvCopy(msg,
                                             sensor_msgs::image_encodings::BGR8);
    }catch (cv_bridge::Exception& e){
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
  }
};

int main(int argc, char **argv){
  //
  // RaspiCam capture code: assumes the camera is being read as the default
  // webcam. Use uncomment-region below.
  //

  // // Open default system camera.
  // cv::VideoCapture cap;
  // if (!cap.open(0)) return 0;

  // //
  // // Read OpenCV Frame from the camera stream.
  // //
  // while(1){
  //   cv::Mat frame;
  //   cap >> frame;
  //   if (frame.empty()) break;
  //   cv::imshow("Video", frame);
  //   if(cv::waitKey(10) == 27) break;
  // }
  //

  ros::init(argc, argv, "image_converter");
  ImageConverter ic;
  ros::spin();

  return 0;
}
