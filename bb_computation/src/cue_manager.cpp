/**
   \file cue_manager.cpp
   \brief ROS Node for wrangling cues from sensory systems into a usable format.

   This node will subscribe to a set of cue detection nodes to receive
   the angle and contrast of each available cue. The contrasts will
   then be compared and their relative weights set and this information
   is compiled into a single data structure.

   \remark This node provides a level of abstraction between cues and
   computational models whereby the computational model (and
   associated node) can be completely agnostic as to cue types and can
   instead just work on a vector of Cues.  This means that models can
   be programmed so as to flexibly adapt to the number of available
   cues on initialisation.

   \remark This node was designed to work with an older conception of
   the multimodal cue integration presented in Mitchell et al. (2023)
   (provided by the MMCX class).  This was a simple extension of the
   Stone et al. (2017) path integration model (CentralComplex) which
   used multiple populations of TL neurons. Flexible construction of
   this model is trivial, the plastic model from Mitchell et al. (2023)
   may not be so straightforward and has not been attempted.

   \node This node is included for completeness but was not in use
   by the end of the project.
*/

#include <ros/ros.h>

#include <vector>

#include "bb_util/bb_util.h"
#include "bb_util/cue.hpp"
#include "bb_util/cue_msg.h"
#include "bb_util/cue_list.h"

#define DEBUG 0

std::vector<ros::Subscriber> subs; // List of subscriptions
ros::Publisher pub; // Publishing topic

// Map to store an entry for each cue's current state
std::vector<bb_util::Cue> cues;

/**
 * bb_util::Cue update callback.
 */
void cue_callback(const bb_util::cue_msg::ConstPtr& msg){
  bb_util::cue_msg cue_msg = *msg;
  bb_util::Cue cue = bb_util::Cue::toCue(cue_msg);
  double contrast_sum = 0;
  int goal_idx = -1;

  // Search for the correct element and sum the contrasts
  // of each cue.
  for (int i = 0; i < cues.size(); i++){
    if (goal_idx == -1 && cues[i] == cue) goal_idx = i;
    contrast_sum = contrast_sum + cues[i].getContrast();
  }

  // Normalise the reliabilities to get the weight.
  cue.setRelativeWeight(cue.getContrast() / contrast_sum);
  
  if (goal_idx == -1){
    ROS_FATAL("Cue type [%s] not found by manager. Supported"
              " types are \"wind\" and \"intensity\".",
              cue.getType().c_str());
    ROS_WARN("If you are using a dummy_cue, override the type"
             " to match one of the supported types. Exiting.");
    exit(-1);
  }
  
  cues[goal_idx] = cue; // Update the cue in the list
}

/**
 * Helper: Translate std::vector<bb_util::Cue> to bb_util::cue_list
 */
inline bb_util::cue_list cue_list_to_msg_list(std::vector<bb_util::Cue> cue_list){
  std::vector<bb_util::cue_msg> msg_list;

  for (int i = 0; i < cue_list.size(); i++)
    msg_list.push_back(bb_util::Cue::toMsg(cue_list[i]));

  bb_util::cue_list cue_msg_list;
  cue_msg_list.cues = msg_list;
  return cue_msg_list;
}

//Debug
std::string cue_list_to_string(std::vector<bb_util::Cue> cue_list){
  std::stringstream ss = std::stringstream("");

  ss << std::endl;
  ss << "=START=======================================" << std::endl;

  for (int i = 0; i < cue_list.size(); ++i){
    std::string type = cue_list[i].getType();
    double contrast = cue_list[i].getContrast();
    double theta = cue_list[i].getTheta();

    ss << type << ": " << theta << ", " << contrast << std::endl;
  }

  ss << "=END=========================================" << std::endl;

  return ss.str();
}

int main(int argc, char **argv){
  ros::init(argc, argv, "cue_manager");
  ros::NodeHandle n;

  // Defined for ease, would be better to have this user-defined somewhere
  // else. E.g.a config file.
  std::vector<std::string> cue_topics = {
    bb_util::defs::WIND_CUE_TOPIC,
    bb_util::defs::INTENSITY_CUE_TOPIC
  };

  std::vector<std::string> cue_types = {
    "wind",
    "intensity"
  };

  //
  // Initialisation: Set up subscribers for each cue type
  // and set up the cue list so the callback has something
  // to find.
  //
  for (int i = 0; i < cue_topics.size(); i++){
    // Temporary while testing with dummy cues, would be
    // better to have a less brittle solution.
    std::string topic_name = cue_topics[i];
    subs.push_back(n.subscribe(topic_name, 1000, cue_callback));

    bb_util::Cue cue = bb_util::Cue(cue_types[i], 0, 0, 0);
    cues.push_back(cue);
  }

  // set up a publisher to advertise a list of cues
  pub = n.advertise<bb_util::cue_list>("cue_list", 1);

  // Retrieve updated cue list and broadcast to the network
  while(ros::ok()){
    // Check callbacks.
    ros::spinOnce();

    #if DEBUG
    ROS_INFO("%s", cue_list_to_string(cues).c_str());
    #endif
    
    // Translate to bb_util::cue_list and publish.
    pub.publish(cue_list_to_msg_list(cues));
  }

  return 0;
}
