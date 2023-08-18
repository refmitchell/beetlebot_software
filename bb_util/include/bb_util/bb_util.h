/**
   \file bb_util.h
   \brief Provides basic global declarations and definitions for the beetlebot.

   Some of the definitions here are still in use so this file is very much required.

   \note Historically there were many definitions (e.g. things like
   movement commands) which it was deemed useful to have defined
   glboally. Most of these were never used or fully developed (which
   is a shame) but the build process is still such that anything
   defined here (or in the associated src/lib_bb_util.cpp) will be
   available to all other packages in beetlebot_software.
   Essentially, while I didn't get to flesh this out, it's still a
   useful space to have in the codebase. Basic structure and some
   unused examples have therefore been preserved.
*/

#pragma once

#include <iostream>

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

/**
   \brief Utility namespace which is subdivided by function.
*/
namespace bb_util {
  /** \brief Contains any global functions pertaining to vision. */
  namespace vision {
    void imshow(const std::string window_title,
                const cv::Mat& frame,
                const cv::Size size = cv::Size(640,640));
  }

  /** 
      \brief Definitions which are shared across different nodes. 

      \note This seems (and is) a little over-engineered but it does
      allow compile-time checking of topic names which can help greatly
      when trying to figure out why one node isn't talking to 
      another. Unfortunately, due to the flexibility of most nodes in
      setting publishers and subscribers at run-time, the utility of these
      global definitions is quite limited.
  */
  namespace defs {
    // Topic/service name definitions
    // These could be factored out into a 'topics' namespace
    const std::string UPDATE_VELOCITY = "update_velocity";
    const std::string CX_STATUS = "cx_status";
    const std::string VEC_MEM_STATUS = "vmcx_status";
    const std::string MMCX_ENCODING = "mmcx_encoding_list";
    const std::string CALIBRATION_NOTIFY_TOPIC = "calibration_notify";
    const std::string INTENSITY_CUE_TOPIC = "intensity_cue";
    const std::string WIND_CUE_TOPIC = "wind_cue";    

    // Available cue types
    const std::string WIND = "wind";
    const std::string INTENSITY = "intensity";

    // Not provided by C++ 
    const double PI = 3.14159265359;
  }

  /** \brief Names for interacting with the parameter server. */
  namespace params {
    // Calibration parameters for sensors; frames of reference must be aligned.
    const std::string CALIBRATION_WIND_OFFSET = "/calibration/wind_offset";
    const std::string CALIBRATION_INTENSITY_OFFSET =
      "/calibration/intensity_offset";
  }
}

