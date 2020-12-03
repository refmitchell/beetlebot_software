#include <iostream>

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "bb_util/bb_util.h"

void bb_util::vision::imshow(const std::string window_title, const cv::Mat& frame){
  cv::Mat resized;
  cv::resize(frame, resized, cv::Size(640, 640));
  cv::imshow(window_title, resized);
  cv::waitKey(10);
}

