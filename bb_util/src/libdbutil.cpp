#include <iostream>
#include <vector>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "bb_util/bb_util.h"

void bb_util::vision::imshow(const std::string window_title,
                             const cv::Mat& frame,
                             const cv::Size size){
  cv::Mat resized;
  cv::resize(frame, resized, size);
  //cv::resize(frame, resized, cv::Size(640, 640));
  cv::imshow(window_title, resized);
  cv::waitKey(10);
}


std::vector<double> bb_util::func::linspace(double start,
                                            double end,
                                            int division,
                                            bool endpoint=true){

  std::vector<double> result(0);

  if (division <= 0) return result; // Return empty vector
  division = endpoint ? (division - 1) : division;

  int limit = endpoint ? division : (division - 1);
  double interval = (end - start) / division;
  double current = start + interval;

  result.push_back(start);

  for (int i = 0; i < limit; i++){
    result.push_back(current);
    current = current + interval;
  }

  return result;
}


