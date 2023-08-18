#ifndef MMCX_CORE
#define MMCX_CORE

/**
   \file mmcx_model.hpp
   \brief [M]ulti[M]odal[C]entralComple[X]. 

   An old implementation of multimodal cue integration in the Central Complex.

   The implementation given here is a version of the Stone et
   al. (2017) cue integration model with a configurable number of TL
   populations. These populations all input to the same CL1 population
   giving a multimodal integration in the CL1 neurons. The relative
   weight of different cues is managed by scaling the synaptic
   connections between the respective TL and CL1 populations.
      
   \warning While the underlying idea (multiple TL populations
   implementing cue integration in CL1 neurons) is still valid, this
   implementation is incredibly simplistic compared to the work provided by
   Mitchell et al. (2023). This implementation should not be used and
   the more recent model should serve as the basis for any future implementation.

   \b References 

   Mitchell et al. (2023) - A model of cue integration as vector
   summation in the insect brain.
*/

#include <iostream>
#include <random>
#include <math.h>
#include <exception>
#include <vector>
#include <string>
#include <cassert>

// Eigen linear algebra library: https://eigen.tuxfamily.org
#include "bb_util/Eigen/Eigen"
#include "bb_util/cue.hpp"
#include "bb_util/vector.hpp"
#include "bb_util/encoding_status.h"
#include "bb_util/vec2d_msg.h"

// Useful to have defined at compile time
#define CX_N_TL2 16
#define CX_N_CL1 16
#define CX_N_TB1 8
#define CX_N_CPU4 16
#define CX_N_CPU1 16

#define MAT_LOG(x) std::cout << std::endl << x << std::endl;
#define LOG(x) std::cout << "cx: " << x << std::endl;

/**
   Outdated implementation, see mmcx_model.hpp. 

   This class implements the Stone et al. (2017) central complex model
   (see CentralComplex) but with a configurable number TL2 neuron
   populations instead of one. The documentation is intentionally
   light, just to allow anyone cuerious to understand how this fit
   together with the rest of the system. If something is not documented
   here, it should be documented in the CentralComplex documentation.
   This class was intended to work with the cue_manager node give by
   cue_manager.cpp.

   \note While this class was intended to work with any number of cues
   (TL2 populations), I think I only ever tested two.

   \warning The working status of this class is unknown. It \b should \b not be used
   and is included for reference \b only. A better multimodal cue integration model
   based on the Central Complex is provided by extended_ring_model.RingModel in 
   scripts/extended_ring_model.py
*/
class MMCX {
protected:
  int n_tl_layers = 1;
  int n_tl2 = CX_N_TL2;
  int n_cl1 = CX_N_CL1;
  int n_tb1 = CX_N_TB1;
  int n_cpu4 = CX_N_CPU4;
  int n_cpu1 = CX_N_CPU1;
  double tb1_tb1_weight = 1;

  double noise = 0.0;

  double tl2_slope = 6.8;
  double tl2_bias = 3.0;
  Eigen::Matrix<double, CX_N_TL2, 1> tl2_prefs;

  double cl1_slope = 3.0;
  double cl1_bias = -0.5;

  double tb1_slope = 5.0;
  double tb1_bias = 0;

  double cpu4_slope = 5.0;
  double cpu4_bias = 2.5;

  double cpu4_mem_gain = 0.0005;
  double cpu4_mem_loss = 0.00026;

  double cpu1_slope = 6.0;
  double cpu1_bias = 2.0;

  /** Vector of TL weight matrices */
  std::vector<Eigen::Matrix<double, CX_N_CL1, CX_N_TL2>> W_TL_CL1;

  // Standard anatomical matrices
  Eigen::Matrix<double, CX_N_TB1, CX_N_CL1> W_CL1_TB1;
  Eigen::Matrix<double, CX_N_TB1, CX_N_TB1> W_TB1_TB1;
  Eigen::Matrix<double, CX_N_CPU1, CX_N_TB1> W_TB1_CPU1;
  Eigen::Matrix<double, CX_N_CPU4, CX_N_TB1> W_TB1_CPU4;
  Eigen::Matrix<double, CX_N_CPU1, CX_N_CPU4> W_CPU4_CPU1;
  Eigen::Matrix<double, 1, CX_N_CPU1> W_CPU1_motor;


  /** Vector with TL population responses for each TL population */
  std::vector<Eigen::Matrix<double, CX_N_TL2, 1>> TL;

  // Standard population matrices
  // Eigen::Matrix<double, CX_N_TL2, 1> TL2;
  Eigen::Matrix<double, CX_N_CL1, 1> CL1;
  Eigen::Matrix<double, CX_N_TB1, 1> TB1;
  Eigen::Matrix<double, CX_N_CPU4, 1> MEM;
  Eigen::Matrix<double, CX_N_CPU4, 1> CPU4;
  Eigen::Matrix<double, CX_N_CPU1, 1> CPU1;

  //
  // Utility members
  //

  /** Used to store the current weights for display purposes. */
  std::vector<double> TL_CL1_display_mag;

  void (*sigmoid) (Eigen::Ref<Eigen::MatrixXd>, double, double, double);
  static void noisySigmoid (Eigen::Ref<Eigen::MatrixXd> v,
                            double slope,
                            double bias,
                            double noise);
  static void noiselessSigmoid (Eigen::Ref<Eigen::MatrixXd>,
                                double,
                                double,
                                double);

  double dot(Eigen::Ref<Eigen::MatrixXd> a, Eigen::Ref<Eigen::MatrixXd> b);

  // Generate intra TB1 weights.
  inline Eigen::Matrix<double, CX_N_TB1, CX_N_TB1> gen_tb1_tb1_weights(double w){
    double sinusoid[CX_N_TB1] =
      {0, 0.14644661, 0.5, 0.85355339, 1., 0.85355339, 0.5, 0.14644661};

    Eigen::Matrix<double, CX_N_TB1, CX_N_TB1> W;

    for (int i = 0; i < CX_N_TB1; i++) {
      for (int j = 0; j < CX_N_TB1; j++) {
        int  idx = (j - i) % (CX_N_TB1);
        idx = idx > 0 ? idx : idx + CX_N_TB1;
        W(i, j) = sinusoid[idx]*w;
      }
    }

    return W;
  }

public:
  /**
     Constructor
     \param n_cues The number of cues and hence the number of TL populations to generate.
   */
  MMCX(int n_cues=1){
    //
    // TL neuron preferred directions (receptive fields).
    //
    tl2_prefs <<
      0,
      0.78539816,
      1.57079633,
      2.35619449,
      3.14159265,
      3.92699082,
      4.71238898,
      5.49778714,
      0,
      0.78539816,
      1.57079633,
      2.35619449,
      3.14159265,
      3.92699082,
      4.71238898,
      5.49778714;

    //
    // Initialise weights from TL to CL1: W_TL_CL1
    //
    n_tl_layers = n_cues;

    // Use even weightings for initialisation
    double init_weight = 1 / (double) n_tl_layers;
    for (int i = 0; i < n_tl_layers; i++){
      Eigen::Matrix<double, CX_N_CL1, CX_N_TL2> tl;

      // Compute the weight matrix and add to the weight matrix colletion.
      tl << Eigen::Matrix<double, CX_N_CL1, CX_N_TL2>::Identity();
      tl *= -init_weight;
      tl.colwise().reverse();
      W_TL_CL1.push_back(tl);
      TL_CL1_display_mag.push_back(init_weight);
    }

    for (int i = 0; i < n_tl_layers; i++){
      Eigen::Matrix<double, CX_N_TL2, 1> tl;
      tl.setConstant(0);
      TL.push_back(tl);
    }

    // Weights from CL1 to TB1: W_CL1_TB1
    W_CL1_TB1 <<
      Eigen::Matrix<double, CX_N_TB1, CX_N_TB1>::Identity(),
      Eigen::Matrix<double, CX_N_TB1, CX_N_TB1>::Identity();

    // Intra-TB1 weights: W_TB1_TB1
    W_TB1_TB1 << gen_tb1_tb1_weights(tb1_tb1_weight);

    // Weights from TB1 to CPU1: W_TB1_CPU1
    W_TB1_CPU1 <<
      Eigen::Matrix<double, CX_N_TB1, CX_N_TB1>::Identity(),
      Eigen::Matrix<double, CX_N_TB1, CX_N_TB1>::Identity();

    // Weights from TB1 to CPU4: W_TB1_CPU4
    W_TB1_CPU4 <<
      Eigen::Matrix<double, CX_N_TB1, CX_N_TB1>::Identity(),
      Eigen::Matrix<double, CX_N_TB1, CX_N_TB1>::Identity();


    // Weights from CPU4 to CPU1: W_CPU4_CPU1
    W_CPU4_CPU1 <<
      0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0,
      1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0,
      0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1,
      0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0;

    W_CPU1_motor << -1, -1, -1, -1, -1, -1, -1, -1, 1, 1, 1, 1, 1, 1, 1, 1;

    // Set sigmoid function; voids unnecessary if-checks and looping
    // if we have no noise.
    sigmoid =
      (noise > 0) ?
      &MMCX::noisySigmoid :
      &MMCX::noiselessSigmoid;

    //Initialise neural populations.

    CL1.setConstant(0);
    TB1.setConstant(0);
    MEM.setConstant(0.5);
    CPU4.setConstant(0);
    CPU1.setConstant(0);

    LOG("CX init complete");
  }

  // Layer-wise functions for CX Operation
  void tl2_output(std::vector<bb_util::Cue>& cue_list,
                  std::vector<Eigen::Matrix<double, CX_N_TL2, 1>>& output);
  void cl1_output(std::vector<Eigen::Matrix<double, CX_N_TL2, 1>>& tl,
                  Eigen::Ref<Eigen::MatrixXd> cl1);
  void tb1_output(Eigen::Ref<Eigen::MatrixXd> cl1,
                  Eigen::Ref<Eigen::MatrixXd> tb1);
  void cpu4_update(double speed,
                   Eigen::Ref<Eigen::MatrixXd> tb1,
                   Eigen::Ref<Eigen::MatrixXd> cpu4_mem);
  void cpu4_output(Eigen::Ref<Eigen::MatrixXd> cpu4_mem);
  void cpu1_output(Eigen::Ref<Eigen::MatrixXd> tb1,
                   Eigen::Ref<Eigen::MatrixXd> cpu4,
                   Eigen::Ref<Eigen::MatrixXd> cpu1);
  double motor_output(Eigen::Ref<Eigen::MatrixXd> cpu1);

  // Monolithic CX Operation
  void set_goal_direction(double theta);
  double input(std::vector<bb_util::Cue>& cue_list, double speed);

  //
  // Status functions
  //

  /**
     Returns the status of each layer of the model (not yet
     augmented for multiple TL layers).
   */
  void get_status(std::vector<std::vector<double>> &activity);

  /**
     Returns the cue as encoded by each TL layer. The last element
     will be the CL1 encoding.
  */
  bb_util::encoding_status get_cue_encoding(std::vector<bb_util::Vec2D> &encodings);
};

void MMCX::noisySigmoid(Eigen::Ref<Eigen::MatrixXd> v,
                                  double slope,
                                  double bias,
                                  double noise){
  // Set input to sigmoid of itself
  for (int i = 0; i < v.rows(); i++){
    v(i,0) = 1 / (1 + std::exp( (-v(i,0) * slope) - bias) );
  }

  // Add Gaussian noiseo
  if (noise > 0){
    // Nice little pattern from the C++ docs.
    std::default_random_engine generator;
    std::normal_distribution<double> distribution(0.0, 1.0);
    auto nextGaussian = std::bind(distribution, generator);
    for (int i = 0; i < v.rows(); i++){
      v(i,0) = v(i,0) + (nextGaussian() * noise);
    }
  }

  // Only needed if we have noise; may be a better way of
  // implementing this.
  for (int i = 0; i < v.rows(); i++){
    if (v(i,0) > 1){
      v(i,0) = 1;
    } else if (v(i,0) < 0) {
      v(i,0) = 0;
    }
  }
}

void MMCX::noiselessSigmoid(Eigen::Ref<Eigen::MatrixXd> v,
                                      double slope,
                                      double bias,
                                      double noise=0){
  // Set input to sigmoid of itself
  for (int i = 0; i < v.rows(); i++){
    v(i,0) = 1 / (1 + std::exp( -(v(i,0) * slope - bias) ) );
  }
}


double MMCX::dot(Eigen::Ref<Eigen::MatrixXd> a,
                 Eigen::Ref<Eigen::MatrixXd> b){
   // Local to avoid multiple function calls
  int a_r = a.rows();
  int a_c = a.cols();
  int b_r = b.rows();
  int b_c = b.cols();

  // Check both are vectors; i.e. at least one dimension is 1D
  bool a_vec = (a_r == 1 || a_c == 1) ? true : false;
  bool b_vec = (b_r == 1 || b_c == 1) ? true : false;

  // Throw if one argument not suitable
  if (!(a_vec && b_vec)){
    throw std::invalid_argument("MMCX::dot(); "
                                "passed argument not a vector");
  }

  // Make sure matrices are appropriately transposed, want row * column.
  if (a_r != 1) a = a.transpose();
  if (b_c != 1) b = b.transpose();

  // Matrix multiplication should give dot product
  // We always expect this to be 1x1, if it isn't it's indicitive of a problem
  Eigen::Matrix<double, 1, 1> res;
  res = a * b;

  // This whole process is due to the typing and inheritence system
  // in Eigen being different to that in EJML.
  return res(0,0);
}


/**
   Compute the TL neuron output for each TL population
   \param[in] cue_list The cue list from the cue_manager (cue_manager.cpp)
   \param[out] output The vector containing all TL neuron populations.

   \note The weighting system used here is quite simple. Each cue in the 
   cue_list is given a relative weight by the cue manager. This weight is then
   used to scale the synaptic strengths from that population onto the CL1 neurons.
   \note
   This ignores the idea that the amplitude of TL responses could change and also
   ignores the fact that the TL -> CL1 synapses are highly plastic. See 
   Mitchell et al. (2023).
*/
void MMCX::tl2_output(std::vector<bb_util::Cue>& cue_list,
                      std::vector<Eigen::Matrix<double, CX_N_TL2, 1>>& output
                      ){
  //
  // Before going any further, make sure everything lines up
  //
  assert(cue_list.size() == n_tl_layers);
  assert(output.size() == n_tl_layers);

  //
  // For each TL layer, compute its response to its respective cue
  //
  for (int i = 0; i < cue_list.size(); i++){
    // Update the relative weightings for each cue.
    W_TL_CL1[i] <<
      Eigen::Matrix<double, CX_N_CL1, CX_N_TL2>::Identity().colwise().reverse();
    W_TL_CL1[i] *= -1 * cue_list[i].getRelativeWeight();

    // Update magnitudes for display
    TL_CL1_display_mag[i] = cue_list[i].getRelativeWeight();


    assert(output[i].rows() == this->tl2_prefs.rows() &&
           output[i].cols() == this->tl2_prefs.cols());

    output[i].setConstant(cue_list[i].getTheta()) ;
    output[i] = output[i] - this->tl2_prefs;

    for (int j = 0; j < output[i].rows(); ++j)
      output[i](j, 0) = cos(output[i](j,0));

    // Standard sigmoid, needs done for each layer.
    this->sigmoid(output[i], this->tl2_slope, this->tl2_bias, this->noise);
  }
}

/**
   Performs a weighted sum of all available TL layers onto the CL1 layer.
   \param[in] tl A vector containing all TL populations
   \param[out] cl1 The CL1 neuron population.
*/
void MMCX::cl1_output(std::vector<Eigen::Matrix<double, CX_N_TL2, 1>>& tl,
                      Eigen::Ref<Eigen::MatrixXd> cl1){

  assert(tl.size() == n_tl_layers);

  // Unset CL1 Layer
  cl1.setConstant(0);

  // Apply a weighted sum of TL layers to CL1
  for (int i = 0; i < n_tl_layers; i++){
    cl1 += W_TL_CL1[i] * tl[i];
  }

  this->sigmoid(cl1, this->cl1_slope, this->cl1_bias, this->noise);
}

void MMCX::tb1_output(Eigen::Ref<Eigen::MatrixXd> cl1,
                      Eigen::Ref<Eigen::MatrixXd> tb1){
  double prop_cl1 = 0.667;
  double prop_tb1 = 1 - prop_cl1;

  Eigen::Matrix<double, CX_N_TB1, 1> cl1_temp;

  cl1_temp = (this->W_CL1_TB1 * cl1) * prop_cl1;
  tb1 = (this->W_TB1_TB1 * tb1) * prop_tb1;
  tb1 = cl1_temp - tb1;

  this->sigmoid(tb1, this->tb1_slope, this->tb1_bias, this->noise);
}

void MMCX::cpu4_update(double speed,
                       Eigen::Ref<Eigen::MatrixXd> tb1,
                       Eigen::Ref<Eigen::MatrixXd> cpu4_mem){
  Eigen::Matrix<double, CX_N_TB1, 1> ones;
  ones.setConstant(1);
  ones -= tb1;

  Eigen::MatrixXd diff_matrix(cpu4_mem.rows(), cpu4_mem.cols());
  diff_matrix.setConstant(speed * this->cpu4_mem_loss);

  cpu4_mem -= diff_matrix;

  diff_matrix = (this->W_TB1_CPU4 * ones) * (speed * this->cpu4_mem_gain);

  cpu4_mem += diff_matrix;

    // Bounds checking
  for (int i = 0; i < cpu4_mem.rows(); i++){
    double tmp = cpu4_mem(i,0);

    if ((tmp < 0) || (tmp > 1)) {
      tmp = tmp > 1 ? 1 : 0;
      cpu4_mem(i,0) = tmp;
    }
  }
}

void MMCX::cpu4_output(Eigen::Ref<Eigen::MatrixXd> cpu4_mem){
  this->sigmoid(cpu4_mem, this->cpu4_slope, this->cpu4_bias, this->noise);
}

void MMCX::cpu1_output(Eigen::Ref<Eigen::MatrixXd> tb1,
                       Eigen::Ref<Eigen::MatrixXd> cpu4,
                       Eigen::Ref<Eigen::MatrixXd> cpu1){
  cpu1 = (this->W_CPU4_CPU1 * cpu4) - (this->W_TB1_CPU1 * tb1);
  this->sigmoid(cpu1, this->cpu1_slope, this->cpu1_bias, this->noise);
}

// Might be a source of problems due to dot method
double MMCX::motor_output(Eigen::Ref<Eigen::MatrixXd> cpu1){
  return this->dot(this->W_CPU1_motor, cpu1);
}

/**
   Analogue of CentralComplex::unimodal_monolithic_CX() which takes
   a list of bb_util::Cues instead of a single angle.
   \param cue_list The cue list given by the cue manager (cue_manager.cpp)
   \param speed
   \return The motor output (left/right steering) of the MMCX.
*/
double MMCX::input(std::vector<bb_util::Cue>& cue_list, double speed){
  double CXMotor = 0;

  this->tl2_output(cue_list, this->TL);
  this->cl1_output(this->TL, this->CL1);
  this->tb1_output(this->CL1, this->TB1);
  this->cpu4_update(speed, this->TB1, this->MEM);
  this->CPU4 = this->MEM;
  this->cpu4_output(this->CPU4);
  this->cpu1_output(this->TB1, this->CPU4, this->CPU1);

  CXMotor = this->motor_output(this->CPU1);

  return CXMotor;
}

/**
   Treats TL and CL populations as distributed sinusoid encodings for vectors.
   This method will decode each TL layer and the CL layer and report the stored
   vector angle and magnitude in the resultant encoding status message. 
   This information should be plottable using bb_graphics/scripts/integration_status

   \return A bb_util::encoding_status message for broadcast
*/
bb_util::encoding_status MMCX::get_cue_encoding(){
  //
  // Decode the cue vector held in each TL layer
  //

  // Basically works out as a circular average.
  // 1. Take only the first 8 TL neurons of each layer
  // 2. Take each neuron represented as a vector
  //    v = (preferred angle, rate)
  // 3. Take the circular average of the population;
  //    given that we only care about the angle this
  //    should work. It should be very close to the
  //    input angle but won't be identical.

  // Vector representation of each neuron
  // Angle/theta
  std::vector<bb_util::Vec2D> per_layer_vector_rep;
  for (int cue = 0; cue < TL.size(); ++cue){ //For each TL layer (cue type)
    bb_util::Vec2D avg_for_layer = bb_util::Vec2D::init_polar(0,0);

    for (int i = 0; i < TL[cue].rows(); ++i) // Sum neural representations
      avg_for_layer += bb_util::Vec2D::init_polar(TL[cue](i,0), tl2_prefs[i]);

    // Compute vector average for layer
    avg_for_layer = avg_for_layer / TL[cue].rows();

    // Weight this final representation by the relative weighting.
    // This should be avg_for_layer.get_r, I was scaling the whole thing rather
    // than just the radius
    avg_for_layer.set_r(avg_for_layer.get_r() * TL_CL1_display_mag[cue]);

    // Add to list of representations
    per_layer_vector_rep.push_back(avg_for_layer);
  }

  //
  // Compute average of the TL averages as a "ground truth" for the
  // CL layer.
  //
  bb_util::Vec2D true_avg = bb_util::Vec2D::init_polar(0,0);

  for (int i = 0; i < per_layer_vector_rep.size(); ++i)
    true_avg += per_layer_vector_rep[i];

  true_avg = true_avg / per_layer_vector_rep.size();

  //
  // Retrieve CL layer
  //

  // Note: I'm using tl2_prefs to do the vector-avg decode. The
  // receptive fields for the CL neurons can (technically) change in
  // this model so it's fair to say this isn't correct; however, I
  // don't think it causes a problem to use tl2_prefs for this specific
  // purpose. The result for display should still be valid.
  bb_util::Vec2D cl_avg = bb_util::Vec2D::init_polar(0,0);

  for (int i = 0; i < CL1.rows(); ++i){
    cl_avg += bb_util::Vec2D::init_polar(CL1(i, 0), tl2_prefs[i]);
  }

  cl_avg = cl_avg / CL1.rows();

  std::vector<bb_util::vec2d_msg> vecs;
  for (int i = 0; i < per_layer_vector_rep.size(); ++i){
    vecs.push_back(bb_util::Vec2D::to_message(per_layer_vector_rep[i]));
  }

  bb_util::encoding_status encoding_list;
  encoding_list.tls = vecs;
  encoding_list.cl1 = bb_util::Vec2D::to_message(cl_avg);
  encoding_list.true_cl1 = bb_util::Vec2D::to_message(true_avg);

  return encoding_list;

}

#endif
