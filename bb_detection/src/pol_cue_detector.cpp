/**
   \file pol_cue_detector.cpp
   \brief Decode pol-op readings, format into a cue then publish.

   On-line decode of the prototype polarisation sensor from
   Gkanias et al. (2023) using the decoding routine from
   Gkanias et al. (2019).

   \warning The working status of this node is unknown but it should
   be functioning properly. It would last have been used in the
   field in November 2022 and I remember it working.
*/

#include <ros/ros.h>
#include <std_msgs/Int32MultiArray.h>
#include <iostream>
#include <cmath>
#include <sstream>
#include <complex>

#include "bb_util/bb_util.h"
#include "bb_util/cue.hpp"
#include "bb_util/cue_msg.h"
#include "bb_util/argparse.h"

#define N_POL_OPS 8
#define LOG(x) std::cout << x << std::endl;

argparse::ArgumentParser parser("Parser");

// Node not currently linked to calibration routine.
// TODO: link this up with the calibration routine
double calibration_offset = 0;
ros::NodeHandle *nhp;

// One vector per pol_op unit - non-configurable
std::vector<std::vector<int>> pol_op_responses(N_POL_OPS);
std::string pol_sub_topic_0 = "pol_op_0";
std::string pol_sub_topic_1 = "pol_op_1";
std::string pol_sub_topic_2 = "pol_op_2";
std::string pol_sub_topic_3 = "pol_op_3";
std::string pol_sub_topic_4 = "pol_op_4";
std::string pol_sub_topic_5 = "pol_op_5";
std::string pol_sub_topic_6 = "pol_op_6";
std::string pol_sub_topic_7 = "pol_op_7";

std::string node_name = "pol_cue_detector";

int n_sol_neurons = 8; //Default, configurable
std::vector<double> pol_prefs;
std::vector<double> sol_prefs;

bb_util::Cue pol_cue = bb_util::Cue("pol", 1, 0, 0);

double (*activation) (int input);

inline double radians(double degrees){return degrees*bb_util::defs::PI / 180;}
inline double degrees(double radians){return 180*radians/bb_util::defs::PI;}


/**
   Initialise the argument parser.
   \param parser The parser
   \param argc The argument count from the command line
   \param argv The parameter value array from the command line
   \return true on success
*/
bool initParser(argparse::ArgumentParser &parser, int argc, char **argv){
  parser.add_argument()
    .names({"-a", "--activation"})
    .description("[If using the bio inspired sensor decode] "
                 "The activation function used for the POL neurons.");

  parser.add_argument()
    .names({"-n_sol", "--n_sol_neurons"})
    .description("[If using the bio inspired sensor decode] "
                 "The number of SOL neurons in use (default = 8).");

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

/** 
    Polarisation opponent callbacks for unit 0 
    \param pol_msg The photodiode readings from the unit.
*/
void polCallback0(const std_msgs::Int32MultiArray::ConstPtr& pol_msg){
  pol_op_responses[0] = pol_msg->data; // Update local memory
}

/** 
    Polarisation opponent callbacks for unit 1
    \param pol_msg The photodiode readings from the unit.
*/
void polCallback1(const std_msgs::Int32MultiArray::ConstPtr& pol_msg){
  pol_op_responses[1] = pol_msg->data; // Update local memory
}

/** 
    Polarisation opponent callbacks for unit 2
    \param pol_msg The photodiode readings from the unit.
*/
void polCallback2(const std_msgs::Int32MultiArray::ConstPtr& pol_msg){
  pol_op_responses[2] = pol_msg->data; // Update local memory
}

/** 
    Polarisation opponent callbacks for unit 3
    \param pol_msg The photodiode readings from the unit.
*/
void polCallback3(const std_msgs::Int32MultiArray::ConstPtr& pol_msg){
  pol_op_responses[3] = pol_msg->data; // Update local memory
}

/** 
    Polarisation opponent callbacks for unit 4
    \param pol_msg The photodiode readings from the unit.
*/
void polCallback4(const std_msgs::Int32MultiArray::ConstPtr& pol_msg){
  pol_op_responses[4] = pol_msg->data; // Update local memory
}

/** 
    Polarisation opponent callbacks for unit 5
    \param pol_msg The photodiode readings from the unit.
*/
void polCallback5(const std_msgs::Int32MultiArray::ConstPtr& pol_msg){
  pol_op_responses[5] = pol_msg->data; // Update local memory
}

/** 
    Polarisation opponent callbacks for unit 6
    \param pol_msg The photodiode readings from the unit.
*/
void polCallback6(const std_msgs::Int32MultiArray::ConstPtr& pol_msg){
  pol_op_responses[6] = pol_msg->data; // Update local memory
}

/** 
    Polarisation opponent callbacks for unit 7
    \param pol_msg The photodiode readings from the unit.
*/
void polCallback7(const std_msgs::Int32MultiArray::ConstPtr& pol_msg){
  pol_op_responses[7] = pol_msg->data; // Update local memory
}

/**
   Log activation function for the pol neurons.
   \param x The input to the neuron
   \return ln(x)
*/
double log_activation(int x){
  return std::log((double) x);
}

/**
   Square root activation function for the pol neurons.
   \param x The input to the neuron
   \return The square root of x.
*/
double sqrt_activation(int x){
  return std::sqrt(x);
}

/**
   Decode the photodiode information using the method from Gkanias et al. (2019)
   \return The angle (phi) and certainty (tau) of the sensor reading (see Gkanias et al. (2019)).
*/
std::pair<double, double> bioInspiredDecode(){
  // Extract the relevant photodiodes and take their absolute response
  std::vector<double> pol_neuron_responses(N_POL_OPS);

  for (int i = 0; i < N_POL_OPS; i++){
    int s_vert = std::abs(pol_op_responses[i][2]);
    int s_horiz = std::abs(pol_op_responses[i][3]);
    ROS_INFO("(%d, %d)", s_vert, s_horiz);
    double r_vert = activation(s_vert);
    double r_horiz = activation(s_horiz);
    double r_op = r_horiz - r_vert;
    double r_po = r_horiz + r_vert;

    if (r_po == 0) r_po = NAN;

    pol_neuron_responses[i] = r_op / r_po;
  }

  double pol_sol_ratio = (n_sol_neurons / N_POL_OPS);
  std::complex<double> R(0, 0); // Complex accumulator for R
  for (int z = 0; z < n_sol_neurons; z++){
    double r_sol_z = 0; // Accumulator for r_sol[zeta]
    for (int j = 0; j < N_POL_OPS; j++){
      double alpha_j = pol_prefs[j] - bb_util::defs::PI/2;
      r_sol_z +=
        pol_sol_ratio *
        std::sin(alpha_j - sol_prefs[z]) *
        pol_neuron_responses[j];
    }
    R +=
      r_sol_z *
      std::exp(std::complex<double>(0,-1) *
               2.0*bb_util::defs::PI *
               (z - 1.0) / (double) n_sol_neurons);
  }

  double a = R.real();
  double b = R.imag();
  double phi = std::atan2(-b, a);
  double tau = std::sqrt(a*a + b*b);
  phi = -phi;

  return std::pair<double, double>(phi,tau);
}

int main(int argc, char **argv){
  if (!initParser(parser, argc, argv)) return -1;

  if (parser.exists("help")){
    parser.print_help();
    return 0;
  }

  std::string method = "bio";

  std::string activation_function =
    parser.exists("activation") ?
    parser.get<std::string>("activation") :
    "log";

  //Globally defined
  n_sol_neurons =
    parser.exists("n_sol_neurons") ?
    parser.get<int>("n_sol_neurons") :
    8;

  std::string pub_topic = "pol_cue";

  // Compute sol neuron preferred angles
  double sol_interval = 2*bb_util::defs::PI / n_sol_neurons;
  sol_prefs = std::vector<double>(n_sol_neurons);
  sol_prefs[0] = 0;
  for (int i = 1; i < n_sol_neurons; i++) {
    sol_prefs[i] = sol_prefs[i - 1] + sol_interval;
  }

  pol_prefs = {0, 90, 180, 270, 45, 135, 225, 315};
  for (int i=0; i < N_POL_OPS; i++){
    pol_prefs[i] = radians(pol_prefs[i]);
  }

  std::pair<double, double> (*decodeSensor) ();

  // Determine which callback function to use
  if (method == "bio") {
    decodeSensor = &bioInspiredDecode;
  } else if (method == "eigen") {
    /*
      Stub, I had originally planned to implement multiple decoding methods
      which used the available data but had trouble getting the eigenvector
      method (Han et al., 2017) to work correctly. In any case, this technique 
      can still be used to assign the correct callback function to the pointer
      if alternative decoding methods are to be trialled.
     */
  } else {
    ROS_ERROR("Unrecognised decoding method %s", method.c_str());
    exit(-1);
  }

  // Determine which activation function to use
  if (activation_function == "log") {
    activation = &log_activation;
  } else if (activation_function == "sqrt") {
    activation = &sqrt_activation;
  } else {
    ROS_ERROR("Unrecognised activation function for POL neurons %s",
              method.c_str());
    ROS_INFO("Supported activation functions: log [default], sqrt");
    exit(-1);
  }

  // ROS init
  ros::init(argc, argv, node_name.c_str());
  ros::NodeHandle n;
  ros::Subscriber pol_sub_0 =
    n.subscribe(pol_sub_topic_0,
                1000,
                polCallback0);
  ros::Subscriber pol_sub_1 =
    n.subscribe(pol_sub_topic_1,
                1000,
                polCallback1);
  ros::Subscriber pol_sub_2 =
    n.subscribe(pol_sub_topic_2,
                1000,
                polCallback2);
  ros::Subscriber pol_sub_3 =
    n.subscribe(pol_sub_topic_3,
                1000,
                polCallback3);
  ros::Subscriber pol_sub_4 =
    n.subscribe(pol_sub_topic_4,
                1000,
                polCallback4);
  ros::Subscriber pol_sub_5 =
    n.subscribe(pol_sub_topic_5,
                1000,
                polCallback5);
  ros::Subscriber pol_sub_6 =
    n.subscribe(pol_sub_topic_6,
                1000,
                polCallback6);
  ros::Subscriber pol_sub_7 =
    n.subscribe(pol_sub_topic_7,
                1000,
                polCallback7);

  ros::Publisher cue_pub =
    n.advertise<bb_util::cue_msg>(pub_topic.c_str(), 1000);


  for (int i = 0; i < N_POL_OPS; i++){
    // Fill with zeros until sensor readings are available.
    pol_op_responses[i] = std::vector<int>(4, 0);
  }

  ros::Rate ten_hz(10);
  while(ros::ok()){
    ros::spinOnce();
    std::pair<double, double> output = decodeSensor();

    double phi = output.first;
    double tau = output.second;
    pol_cue.setAzimuth(phi);
    pol_cue.setContrast(tau);
    cue_pub.publish(bb_util::Cue::toMsg(pol_cue));
    ROS_INFO("\n%s", pol_cue.toString().c_str());
    ten_hz.sleep();
  }

}

