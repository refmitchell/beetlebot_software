#pragma once

#include <iostream>

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

namespace bb_util {
  namespace vision {
    void imshow(const std::string, const cv::Mat&);
  }

  // Definitions which are shared amongst nodes. Constness should
  // be applied as appropriate.
  namespace defs {
    // Velocity update service should always advertise the
    // same name.
    const std::string UPDATE_VEL_SRV = "update_velocity";
    const std::string LOC_CMD_SRV = "loc_node_command";

    // Topic for broadcasting the vector memory status (all layers)
    const std::string VEC_MEM_STATUS = "vmcx_status";

    // Available cue types, should reduce to strings, we only
    // want to ensure they're sensible when set, after that they
    // won't be modified.
    const std::string WIND = "wind";
    const std::string INTENSITY = "intensity";

    const double PI = 3.14159265359;
  }

  // More permenant node name definitions for formal nodes
  namespace nodes{
    const std::string VM_MENOTAXIS = "vm_menotaxis";
  }

  //Locomotion node command opcodes. Some of these are for movement,
  //others are functional.
  namespace loc_node_commands {
    namespace driving {
      const int ALL_STOP = 0;
      const int FWD = 1;
      const int BWD = 2;
      const int RGT = 3;
      const int LFT = 4;
    }

    namespace control {
      const int LIN_LOCK_TOGGLE = -1;
      const int ANG_LOCK_TOGGLE = -2;
    }
  }
}

