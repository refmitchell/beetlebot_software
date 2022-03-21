/**
   @file pol_sensor_decode.cpp
   @brief Decode the signal from the SkyCompass sensor
*/

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Twist.h>

#include <sstream>
#include <cmath>
#include <iostream>
#include <chrono>
#include <thread>
#include <cstdint>

#include "bb_util/bb_util.h"
#include "bb_util/argparse.h"
#include "bb_util/locomotion_cmd.h"

// Eigen linear algebra library: https://eigen.tuxfamily.com
#include "bb_util/Eigen/Eigen"

#define N_POL_OPS 4

argparse::ArgumentParser parser("Parser");
rosbag::Bag bag; // Rosbag for recording
ros::Publisher cmd_publisher;

std::vector<int> global_sensor_data; // Current sensor data (sign-extended)
double global_yaw; // Current yaw (from odom)
float angular_distance = 0;
bool yaw_data_received = false;
bool pol_data_received = false;

std::vector<std::vector<int>> pol_op_responses;

inline double radians(double degrees){return degrees*bb_util::defs::PI / 180;}
inline double degrees(double radians){return 180*radians/bb_util::defs::PI;}

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

//
// POL-OP callbacks; brittle but simple and quick.
//
void polCallback0(const std_msgs::Int32MultiArray::ConstPtr& pol_msg){
  pol_op_responses[0] = pol_msg->data;
}
void polCallback1(const std_msgs::Int32MultiArray::ConstPtr& pol_msg){
  pol_op_responses[1] = pol_msg->data;
}
void polCallback2(const std_msgs::Int32MultiArray::ConstPtr& pol_msg){
  pol_op_responses[2] = pol_msg->data;
}
void polCallback3(const std_msgs::Int32MultiArray::ConstPtr& pol_msg){
  pol_op_responses[3] = pol_msg->data;
}

Eigen::Matrix<double, 3, N_POL_OPS> compute_E(){
  /*
    Procedure to compute the 3xN_POL_OPS matrix E from Han et
    al. (2017), "Design and calibration of a novel bio-inspired
    pixelated polarized light compass".
  */

  // Step 1: compute the angle of polarisation and turn these into unit
  // vectors (polarisation e-vectors in context).
  std::vector<Eigen::Matrix<double, 3, 1>> e_vectors_cart;
  for (int i = 0; i < N_POL_OPS; i++){
    double Q = pol_op_responses[i][2] - pol_op_responses[i][3];
    double U = pol_op_responses[i][1] - pol_op_responses[i][0];
    double angle = atan2(U, Q) / 2;

    Eigen::Matrix<double,3,1> PE;
    PE <<
      cos(angle),
      sin(angle),
      1;
    e_vectors_cart.push_back(PE);
  }

  // Step 2: translate these e-vectors to the observer coordinate frame.
  // Sensor azimuths
  //  double alphas[N_POL_OPS] = {0, radians(90), radians(180), radians(270)};
  double alphas[N_POL_OPS] = {radians(180), radians(90), 0, radians(270)};
  // Sensor zenith angle. I think this is meant to be inclination but it could
  // also be elevation. As we use 45deg it doesn't matter.
  double gamma = radians(45);
  // Second component of translation matrix C computation. Same for all sensors
  // so define once.
  Eigen::Matrix3d c2;
  c2 <<
      cos(gamma), 0, sin(gamma),
      0, 1, 0,
      -sin(gamma), 0, cos(gamma);

  Eigen::Matrix<double, 3, N_POL_OPS> E;
  for (int i = 0; i < N_POL_OPS; i++){
    Eigen::Matrix3d c1;
    Eigen::Matrix3d C;
    double alpha = alphas[i];
    c1 <<
      cos(alpha), -sin(alpha), 0,
      sin(alpha), cos(alpha), 0,
      0, 0, 1;

    C = c1 * c2; // Eq. (7), Han et al. (2017)
    Eigen::Matrix<double, 3, 1> e = C*e_vectors_cart[i]; // Eq. (8), Han et al. (2017)
    E.col(i) = e;
  }

  return E;
}

Eigen::Matrix<double, 3, 1> find_solar_vector(Eigen::Matrix<double, 3, N_POL_OPS> E){
  /*
    According to Han et al. (2017), the solar vector (3D vector
    pointing to the sun's position is given by the vector
    corresponding to the smallest eigenvalue of EE' (' denotes
    transpose).
   */
  Eigen::Matrix<double, 3, 3> EET = E*E.transpose();
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 3, 3>> es(EET);

  // Eigen::SelfAdjointEigenSolver will return eigenvectors in order of
  // increasing eigenvalue, i.e. es.eigenvectors().col(0) is the eigenvector
  // corresponding to the smallest eigenvalue.
  Eigen::Matrix<double, 3, 1> solar_vector = es.eigenvectors().col(0);
  return solar_vector;
}

int main(int argc, char **argv){
  if (!initParser(parser, argc, argv)) return -1;
  if (parser.exists("help")){
    parser.print_help();
    return 0;
  }

  // Initialise all pol_op datastructure
  for (int i = 0; i < N_POL_OPS; i++){
    std::vector<int> pd_responses = {0,0,0,0};
    pol_op_responses.push_back(pd_responses);
  }

  std::vector<double> sensor_angles = {0,90,180,270};
  double sensor_elevation = 45;

  // ROS init
  ros::init(argc, argv, "pol_sensor_decoder");
  ros::NodeHandle n;
  ros::Subscriber po0_sub = n.subscribe("pol_op_0", 1000, polCallback0);
  ros::Subscriber po1_sub = n.subscribe("pol_op_1", 1000, polCallback1);
  ros::Subscriber po2_sub = n.subscribe("pol_op_2", 1000, polCallback2);
  ros::Subscriber po3_sub = n.subscribe("pol_op_3", 1000, polCallback3);
  ros::Publisher sol_vec_pub =
    n.advertise<std_msgs::Float64MultiArray>("solar_vector", 1000);

  ros::Rate ten_hz(10);
  while (ros::ok()){
    ros::spinOnce(); // Get new polarisation data
    Eigen::Matrix<double, 3, 1> s = find_solar_vector(compute_E());
    std::vector<double> s_std_vec;
    for (int i = 0; i < 3; i++) s_std_vec.push_back(s(i, 0));
    std_msgs::Float64MultiArray s_msg;
    s_msg.data = s_std_vec;
    sol_vec_pub.publish(s_msg);
    ten_hz.sleep();
  }

  ROS_INFO("Exiting");
  return 0;
}
