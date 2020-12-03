/*
  Included for the purposes of testing, not final.
*/

#include <ros/ros.h>

#include <vector>
#include <map>

#include "bb_util/cue.hpp"
#include "bb_util/cue_msg.h"
#include "bb_util/cue_list.h"

std::vector<ros::Subscriber> subs;
ros::Publisher pub;

// Map to store an entry for each cue's current state
std::vector<bb_util::Cue> cues;

/**
 * bb_util::Cue update callback.
 */
void cue_callback(const bb_util::cue_msg::ConstPtr& msg){
  bb_util::cue_msg cue_msg = *msg;
  bb_util::Cue cue = bb_util::Cue::toCue(cue_msg);

  // Search for the correct element and sum the reliabilities
  // of each cue.
  double reliability_sum = 0;
  int goal_idx = -1;
  for (int i = 0; i < cues.size(); i++){
    if (goal_idx == -1 && cues[i] == cue) goal_idx = i;
    reliability_sum = reliability_sum + cues[i].getReliability();
  }

  // Normalise the reliabilities to get the weight.
  cue.setRelativeWeight(cue.getReliability() / reliability_sum);
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

int main(int argc, char **argv){
  ros::init(argc, argv, "cue_manager");
  ros::NodeHandle n;

  // Defined for ease, would be better to have this user-defined somewhere
  // else. E.g.a config file.
  std::vector<std::string> cue_nodes = { "dummy_wind_node", "dummy_light_node" };

  // Set up subscribers for each cue node
  for (int i = 0; i < cue_nodes.size(); i++)
    subs.push_back(n.subscribe(cue_nodes[i], 1000, cue_callback));

  // Set up a publisher to advertise a list of cues
  pub = n.advertise<bb_util::cue_list>("cue_list", 1);

  // Retrieve updated cue list and broadcast to the network
  while(ros::ok()){
    // Check callbacks.
    ros::spinOnce();

    // Translate to bb_util::cue_list and publish.
    pub.publish(cue_list_to_msg_list(cues));
  }

  return 0;
}
