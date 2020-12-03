#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include "computation/argparse.h"
//#include "computation/cx_model.hpp"

//Global, bad but should be safe in this case
argparse::ArgumentParser parser("Parser");

bool initParser(argparse::ArgumentParser &parser, int argc, char **argv){
  //Global, all computation nodes should have these options.
  // parser.add_argument()
  //   .names({"-v", "--video"})
  //   .description("Enable video output for this node.")
  //   .required(false);

  parser.add_argument()
    .names({"-s", "--subscribe"})
    .description("Set the subscription topic.")
    .required(true);

  parser.add_argument()
    .names({"-p", "--publish"})
    .description("Set the publication topic.")
    .required(false);

  parser.add_argument()
    .names({"-n", "--name"})
    .description("Set the node name.")
    .required(false);

  parser.enable_help();

  auto err = parser.parse(argc, const_cast<const char**>(argv));
  if (err) {
    std::stringstream ss;
    ss << err;
    ROS_ERROR("argparse error: %s", ss.str().c_str());
    return false;
  }

  return true;
}

void cue_callback(const computation::cue_vector::ConstPtr& cue_msg){

}

int main(int argc, char **argv){
  //
  // Init parser
  //
  if (!initParser(parser, argc, argv)) return -1;

  if (parser.exists("help")){
    parser.print_help();
    return 0;
  }

  //
  // Decode params
  //
  std::string sub_topic =  parser.get<std::string>("subscribe");
  std::string pub_topic =
    parser.exists("publish") ? parser.get<std::string>("publish") : "cx_output";
  std::string node_name =
    parser.exists("name") ? parser.get<std::string>("name") : "cx_node";

  //
  // Node setup
  //
  ros::init(argc, argv, node_name);
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe(sub_topic, 1000, cue_callback);

  ROS_INFO("CX Running...");

  return 0;
}
