/**
   @file pol_op_recording.cpp
   @brief Specialised node which breaks the modular conventions of the codebase.

   This node runs a preset sequence of movements governed by user
   timing for the purpose of testing different photodiodes for the
   polarisation opponent units. I've tried to separate out the locomotion
   code to bb_locomotion but it's not so neat.

   Sequence of events:
   - Wait for data from pol_op and odometry to become available
   - Send request to bb_locomotion to 'zero' the robot, wait for completion.
   - Start recording sensor and odom data using rosbag
   - Wait T seconds (T is user specified in seconds)
   - Send rotation request to bb_locomotion
   - Send stop request once the robot has traversed 360deg
   - Record for T seconds after completeion
   - Stop recording and exit.
*/

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <std_msgs/UInt32MultiArray.h>
#include <std_msgs/Float64.h>

#include <sstream>
#include <cmath>
#include <iostream>
#include <chrono>
#include <thread>
#include <cstdint>

#include "bb_util/bb_util.h"
#include "bb_util/argparse.h"
#include "bb_util/locomotion_cmd.h"

argparse::ArgumentParser parser("Parser");
rosbag::Bag bag; // Rosbag for recording
std::vector<uint32_t> global_sensor_data; // Current sensor data
double global_yaw; // Current yaw (from odom)
float angular_distance = 0;
bool yaw_data_received = false;
bool pol_data_received = false;
std::string pol_sub_topic = "pol_op_data";
std::string odom_sub_topic = "yaw";
std::string node_name = "pol_op_pd_test_recorder";

bool initParser(argparse::ArgumentParser &parser, int argc, char **argv){
  parser.add_argument()
    .names({"-t", "--time"})
    .description("Duration (seconds) of bookends. If not specified, "
                 "bookends (pre and post-rotation sensor data recording) "
                 "are disabled")
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

void yawCallback(const std_msgs::Float64::ConstPtr& yaw_msg){
  // Conversion from Quaternion to RPY, lifted from ROS forums.
  global_yaw = yaw_msg->data; // Update local memory
  bag.write(odom_sub_topic.c_str(), ros::Time::now(), yaw_msg);
  yaw_data_received = true;
}

void polCallback(const std_msgs::UInt32MultiArray::ConstPtr& pol_msg){
  global_sensor_data = pol_msg->data; // Update local memory
  bag.write(pol_sub_topic.c_str(), ros::Time::now(), pol_msg);
  pol_data_received = true;
}

int main(int argc, char **argv){
  // Parser initialisation
  if (!initParser(parser, argc, argv)) return -1;

  // Parser decode
  if (parser.exists("help")){
    parser.print_help();
    return 0;
  }

  int bookend =
    parser.exists("time") ?
    parser.get<float>("time") :
    0;

  auto bookend_time = std::chrono::seconds(bookend);

  // ROS init
  ros::init(argc, argv, node_name.c_str());
  ros::NodeHandle n;

  // Want to update rosbag given data from either topic
  // as we want to know how these change w.r.t. eachother.
  ros::Subscriber pol_sub =
    n.subscribe(pol_sub_topic,
                1000,
                polCallback);
  ros::Subscriber odom_sub =
    n.subscribe(odom_sub_topic,
                1000,
                yawCallback);

  // Connect to the locomotion node.
  ros::ServiceClient client =
    n.serviceClient<bb_util::locomotion_cmd>("preset_movement");

  // Spin until we start receiving data
  while(ros::ok() && !(yaw_data_received && pol_data_received)){
    ros::spinOnce();
    ROS_INFO("Awaiting pol/odom data. Start the sensor nodes.");
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }

  ROS_INFO("Data available.");
  ROS_INFO("Moving to start position.");

  bb_util::locomotion_cmd zero_cmd;
  zero_cmd.request.opcode = bb_util::loc_node_commands::driving::ZERO;

  if (!client.call(zero_cmd)){
    ROS_ERROR("Failed to call preset_movement service. Make sure it's started.");
    return 1;
  }

  // Start recording
  ROS_INFO("Beginning recording.");
  bag.open("pol_op_recording.bag", rosbag::bagmode::Write);

  auto start_time = std::chrono::system_clock::now();
  auto end_time = std::chrono::system_clock::now(); // rotation end
  double last_measure = global_yaw;
  double traverse = 0;
  bool rotation_started = false;
  bool rotation_complete = false;
  while(ros::ok()){
    ros::spinOnce(); // Get any new messages



    // Rotation
    auto current_time = std::chrono::system_clock::now();

    std::chrono::duration<double, std::milli> diff = current_time - start_time;

    // Management, this is awful...
    // If initial wait is complete and the rotation isn't complete
    if ((diff > bookend_time) && !rotation_complete){
      if (!(rotation_started)){
        // Start rotation if it hasn't started
        bb_util::locomotion_cmd rgt;
        rgt.request.opcode = bb_util::loc_node_commands::driving::RGT;
        client.call(rgt);
        rotation_started = true;
      } else if (traverse >= 2*bb_util::defs::PI) {
        // Check if rotation is complete and send stop signal
        rotation_complete = true;
        bb_util::locomotion_cmd all_stop;
        all_stop.request.opcode = bb_util::loc_node_commands::driving::ALL_STOP;
        client.call(all_stop);
        end_time = std::chrono::system_clock::now();
      } else { // Rotation has started and isn't complete, update traverse.
        traverse = traverse + (global_yaw - last_measure);
        last_measure = global_yaw;
      }
    } else {
      diff = current_time - end_time;
      if (diff > bookend_time){
        bag.close(); // End recording
        break; // Break out of the loop, end node.
      }
    }
  }



  return 0;
}
