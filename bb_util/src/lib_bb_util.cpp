/**
   \file lib_bb_util.cpp
   \brief Provides implementations of any functions declared in bb_util.h.
*/

#include <iostream>

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "bb_util/bb_util.h"

/**
   Wrapper for cv imshow which allows image frames to be resized.
   Saves on code duplication in bb_improc.

   \param window_title The title of the OpenCV window.
   \param frame The OpenCV matrix to show.
   \param size The desired size of the image display.
*/
void bb_util::vision::imshow(const std::string window_title,
                             const cv::Mat& frame,
                             const cv::Size size){
  cv::Mat resized;
  cv::resize(frame, resized, size);
  cv::imshow(window_title, resized);
  cv::waitKey(10);
}

