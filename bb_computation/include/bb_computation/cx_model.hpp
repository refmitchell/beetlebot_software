#ifndef CX_CORE
#define CX_CORE

/**
   \file cx_model.hpp
   \brief Provides a definition (and implementation) for the CentralComplex class.
   \author Robert Mitchell

   Provides a definition for the CentralComplex class. This version of the model
   was ported directly from the AntBot. This class implements the Central Complex
   model for path integration given by Stone et al. (2017). The version from which
   this was ported was originally written by Luca Scimeca. 

   \warning The code CPU4 update used here is slightly different to that given 
   by Stone et al. (2017) but it is functionally identical to a version given 
   in their codebase. The difference stems from the fact that we do not model
   the TN neurons and instead just pass a 'speed' input to the model. This difference
   was present on the AntBot too.

   \b References

   Stone et al. (2017) - An anatomically constrained model for path integration in
   the bee brain.
 */

#include <iostream>
#include <random>
#include <cmath>
#include <exception>
#include <vector>
#include <string>

// Eigen linear algebra library: https://eigen.tuxfamily.org
#include "bb_util/Eigen/Eigen"

#include "bb_util/bb_util.h"


// Convenient to have these known at compile time
#define CX_N_TL2 16  /**< The number of TL2 neurons in the model. */
#define CX_N_CL1 16  /**< The number of CL1 neurons in the model. */
#define CX_N_TB1 8   /**< The number of TB1 neurons in the model. */
#define CX_N_CPU4 16 /**< The number of CPU4 neurons in the model. */
#define CX_N_CPU1 16 /**< The number of CPU1 neurons in the model. */

/** Matrix printing macro. */
#define MAT_LOG(x) std::cout << std::endl << x << std::endl;

/** General printing macro. */
#define LOG(x) std::cout << "cx: " << x << std::endl;

/**
   \brief Class definition of the Central Complex.
   The central complex path integration model as defined by Stone et al.
   Ported from the AntBot implementation provided by Luca Scimeca. This
   version of the model does not implement TN neurons for speed measurement.
 */
class CentralComplex {
protected:
  /**
      The parameters used to define the network dynamics.
  */
  int n_tl2 = CX_N_TL2; //!< The number of TL2 neurons in use.
  int n_cl1 = CX_N_CL1; //!< The number of CL1 neurons in use.
  int n_tb1 = CX_N_TB1; //!< The number of TB1 neurons in use
  int n_cpu4 = CX_N_CPU4; //!< The number of CPU4 neurons in use.
  int n_cpu1 = CX_N_CPU1; //!< The number of CPU1 neurons in use.
  double tb1_tb1_weight = 1; //!< The default TB1-TB1 weightings.

  // Class parameters
  double noise = 0.0; //<! Noise injected on the output of each layer.

  double tl2_slope = 6.8; //!< TL2 activation slope
  double tl2_bias = 3.0; //!< TL2 activation bias
  Eigen::Matrix<double, CX_N_TL2, 1> tl2_prefs; //!< The TL2 receptive fields.

  double cl1_slope = 3.0; //!< CL1 activation slope
  double cl1_bias = -0.5; //!< CL1 activation bias

  double tb1_slope = 5.0; //!< TB1 activation slope
  double tb1_bias = 0; //!< TB1 activation bias

  double cpu4_slope = 5.0; //!< CPU4 activation slope
  double cpu4_bias = 2.5; //!< CPU4 activation bias

  /** CPU4 memory gain */
  double cpu4_mem_gain = 0.005;  //0.005, default

  /** CPU4 memory loss*/
  double cpu4_mem_loss = 0.00026; //0.0026, default

  double cpu1_slope = 6.0; //!< CPU1 activation slope
  double cpu1_bias = 2.0; //!< CPU1 activation bias

  /*
     Anatomical weight matrices.
   */
  Eigen::Matrix<double, CX_N_TB1, CX_N_CL1> W_CL1_TB1; //!< CL1 -> TB1 connections
  Eigen::Matrix<double, CX_N_TB1, CX_N_TB1> W_TB1_TB1; //!< TB1 -> TB1 connections
  Eigen::Matrix<double, CX_N_CPU1, CX_N_TB1> W_TB1_CPU1; //!< TB1 -> CPU1 connections
  Eigen::Matrix<double, CX_N_CPU4, CX_N_TB1> W_TB1_CPU4; //!< TB1 -> CPU4 connections
  Eigen::Matrix<double, CX_N_CPU1, CX_N_CPU4> W_CPU4_CPU1; //!< CPU4 -> CPU1 connections
  Eigen::Matrix<double, 1, CX_N_CPU1> W_CPU1_motor; //!< CPU1 to motor output neuron connections

  /*
     Neural population matrices.
  */
  Eigen::Matrix<double, CX_N_TL2, 1> TL2; //!< TL2 neuron activities
  Eigen::Matrix<double, CX_N_CL1, 1> CL1; //!< CL1 neuron activities
  Eigen::Matrix<double, CX_N_TB1, 1> TB1; //!< TB1 neuron activities
  Eigen::Matrix<double, CX_N_CPU4, 1> MEM; //!< Memory neuron (CPU4_{t - 1}) activities.
  Eigen::Matrix<double, CX_N_CPU4, 1> CPU4; //!< CPU4 neuron activities
  Eigen::Matrix<double, CX_N_CPU1, 1> CPU1; //!< CPU1 neuron activities


  /**
     Sigmoid activation function pointer.

     We can know whether we want noise at construction and this
     pointer allows the class to determine which method to call,
     noiselessSigmoid or noisySigmoid, at construction to avoid
     checking for each call. 

     \note This functionality is not currently used by the constructor.
  */
  void (*sigmoid) (Eigen::Ref<Eigen::MatrixXd>, double, double, double);

  /**
     Sigmoid activation function with added noise. Note that the
     result is placed in v.

     \param[in,out] v The input population.
     \param slope The slope of the sigmoid.
     \param bias The bias (l/r shift) of the sigmoid.
     \param noise The additional noise
   */
  static void noisySigmoid (Eigen::Ref<Eigen::MatrixXd> v,
                            double slope,
                            double bias,
                            double noise);

  /**
     Noiseless sigmoid activation function. Note that the result
     is placed in v.

     \param[in,out] v The input population.
     \param slope The slope of the sigmoid.
     \param bias The bias (l/r shift) of the sigmoid.
     \param noise The additional noise (\b ignored)
   */
  static void noiselessSigmoid (Eigen::Ref<Eigen::MatrixXd> v,
                                double slope,
                                double bias,
                                double noise);

  /**
     Dot product which allows Eigen Matrices to be used as input.

     \param a Matrix a
     \param b Matrix b
     \return a dot b
   */
  double dot(Eigen::Ref<Eigen::MatrixXd> a, Eigen::Ref<Eigen::MatrixXd> b);

  /**
      Generate intra TB1 weights.
      \param w Weight scaling factor.
      \return The TB1 -> TB1 weight matrix.
  */
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
     Constructor. Initialise the model.
   */
  CentralComplex(){
    // TL2 preferred angles
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

    // CPU4_MEM
    //cpu4_mem << Eigen::Matrix<double, CX_N_CPU4, 1>::Zero();

    /**
       Set up anatomical matrices.
     */
    
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
      &CentralComplex::noisySigmoid :
      &CentralComplex::noiselessSigmoid;

    // Initialise neural populations.
    TL2.setConstant(0);
    CL1.setConstant(0);
    TB1.setConstant(0);
    MEM.setConstant(0.5);
    CPU4.setConstant(0);
    CPU1.setConstant(0);

    LOG("CX init complete");
  }

  // Layer-wise functions for CX Operation

  /**
     Given angular input, compute the TL2 population response.
     \param[in] theta The angular input (in radians) denoting the agent's current
                  direction.
     \param[out] output The population response.
   */
  void tl2_output(double theta, Eigen::Ref<Eigen::MatrixXd> output);

  /**
     Compute CL1 response given a current TL2 response.
     \param[in] tl2 The current TL2 activity matrix.
     \param[out] cl1 The CL1 activity output
   */
  void cl1_output(Eigen::Ref<Eigen::MatrixXd> tl2,
                  Eigen::Ref<Eigen::MatrixXd> cl1);

  /**
     Compute TB1 response given a current CL1 response.
     \param[in] cl1 The current CL1 activity matrix.
     \param[out] tb1 The TB1 activity output.
  */
  void tb1_output(Eigen::Ref<Eigen::MatrixXd> cl1,
                  Eigen::Ref<Eigen::MatrixXd> tb1);

  /**
     Update the CPU4 inputs given the current TB1 (direction) response
     and the agent's current speed.

     \note This function does not sigmoid the output. It just formats
     the CPU4 input into a single vector which can be sigmoided. This is
     used to implement the t - 1 term in the CPU4 update rule
     
     \param[in] speed The agent's current speed.
     \param[in] tb1 The current TB1 activity.
     \param[out] cpu4_mem The updated (unsigmoided) CPU4 activity.
  */
  void cpu4_update(double speed,
                   Eigen::Ref<Eigen::MatrixXd> tb1,
                   Eigen::Ref<Eigen::MatrixXd> cpu4_mem);

  /**
     Compute the CPU4 layer output. Pass the CPU4 activity through the
     CPU4 sigmoid function.

     \param[in,out] cpu4_mem The CPU4 activity which will be sigmoided.
  */
  void cpu4_output(Eigen::Ref<Eigen::MatrixXd> cpu4_mem);

  /**
     Compute the CPU1 layer output given the current TB1 and CPU4 activities.
     \param[in] tb1 The current TB1 activity.
     \param[in] cpu4 The current CPU4 activity.
     \param[out] cpu1 The CPU1 layer to be updated.
   */
  void cpu1_output(Eigen::Ref<Eigen::MatrixXd> tb1,
                   Eigen::Ref<Eigen::MatrixXd> cpu4,
                   Eigen::Ref<Eigen::MatrixXd> cpu1);

  /**
     Compute the motor output from the CPU1 layer activity.
     \param[in] The current CPU1 population.
     \return The network output which will be positive or negative depending on
     \return whether the model is indicating a left or right turn.
   */
  double motor_output(Eigen::Ref<Eigen::MatrixXd> cpu1);


  /**
     A helper function which wraps model usage, removing the need to call
     each individual function in sequence as in the AntBot.
     \param theta The current angular input from available cues.
     \param speed The agent's current speed.
     \returns The return value of CentralComplex::motor_output.
   */
  double unimodal_monolithic_CX(double theta, double speed);

  /**
     Collects the current network activity into a single data so that it can be
     read elsewhere in the ROS sofwtware ecosystem (implemented primarily for
     visualising the CX activity).
     \param[in,out] activity The data structure to be filled with the network
                             state.
   */
  void get_status(std::vector<std::vector<double>> &activity);
};

/*
  Implementation begins
 */

/*
  The original version had a bunch of getters and setters which I don't think
  were ever used. They've been ommitted unless they become necessary.
*/
void CentralComplex::noisySigmoid(Eigen::Ref<Eigen::MatrixXd> v,
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

void CentralComplex::noiselessSigmoid(Eigen::Ref<Eigen::MatrixXd> v,
                                      double slope,
                                      double bias,
                                      double noise=0){
  // Set input to sigmoid of itself
  for (int i = 0; i < v.rows(); i++){
    v(i,0) = 1 / (1 + std::exp( -(v(i,0) * slope - bias) ) );
  }
}

double CentralComplex::dot(Eigen::Ref<Eigen::MatrixXd> a,
                           Eigen::Ref<Eigen::MatrixXd> b){
  int a_r = a.rows();
  int a_c = a.cols();
  int b_r = b.rows();
  int b_c = b.cols();

  // Check both are vectors; i.e. at least one dimension is 1D
  bool a_vec = (a_r == 1 || a_c == 1) ? true : false;
  bool b_vec = (b_r == 1 || b_c == 1) ? true : false;

  // Throw if one argument not suitable
  if (!(a_vec && b_vec)){
    throw std::invalid_argument("CentralComplex::dot(); "
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

//
// Layer-wise CX functions; these can be strung together to form a complete
// network process. These can be called manually by a host process or
// you can use the unimodal_monolithic_CX function to simplify things.
//

// Compute tl2 output, store in output argument.
void CentralComplex::tl2_output(double theta, Eigen::Ref<Eigen::MatrixXd> output){
  if (output.rows() != this->tl2_prefs.rows() ||
      output.cols() != this->tl2_prefs.cols()
      ){
    throw std::invalid_argument(
                                "Tl2 output matrix dimensions "
                                "do not match TL2 tuning matrix."
                                );
  }

  output.setConstant(theta);
  output = output - this->tl2_prefs;

  if (std::isnan(theta)){
    /* 
       If a sensor returned NaN for whatever reason, we do not want this to
       propagate, setting output to bb_util::defs::PI/2 here will set each
       element to 0 in the following for-loop (cos(pi/2) == 0). This means
       no directional input to the rest of the network. If NaNs are allowed
       to enter the network then they will persist due to the memory circuit.
       (CPU4 gets input from the previous timestep).

       This was occasionally a problem with the polarisation sensor.
    */
    output.setConstant(bb_util::defs::PI/2);
  }

  for (int i = 0; i < output.rows(); i++){
    output(i,0) = std::cos(output(i,0));
  }

  this->sigmoid(output, this->tl2_slope, this->tl2_bias, this->noise);
}

// Compute CL1 output
void CentralComplex::cl1_output(Eigen::Ref<Eigen::MatrixXd> tl2,
                                Eigen::Ref<Eigen::MatrixXd> cl1){
  cl1 = -1 * tl2;
  this->sigmoid(cl1, this->cl1_slope, this->cl1_bias, this->noise);
}

// Compute TB1 layer output
void CentralComplex::tb1_output(Eigen::Ref<Eigen::MatrixXd> cl1,
                                Eigen::Ref<Eigen::MatrixXd> tb1){
  double prop_cl1 = 0.667;
  double prop_tb1 = 1 - prop_cl1;

  Eigen::Matrix<double, CX_N_TB1, 1> cl1_temp;

  cl1_temp = (this->W_CL1_TB1 * cl1) * prop_cl1;
  tb1 = (this->W_TB1_TB1 * tb1) * prop_tb1;
  tb1 = cl1_temp - tb1;

  this->sigmoid(tb1, this->tb1_slope, this->tb1_bias, this->noise);
}

// Update CPU4 based on TB1 and current speed; result stored in cpu4_mem
// reference. Note: This is the code from Luca, the equation used in the
// original paper is slightly different but this version is equivalent to
// one of the versions given in the associated codebase. The main difference
// appears to be the lack of TN neurons.
void CentralComplex::cpu4_update(double speed,
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

// Store CPU4 output in cpu4_mem
void CentralComplex::cpu4_output(Eigen::Ref<Eigen::MatrixXd> cpu4_mem){
  this->sigmoid(cpu4_mem, this->cpu4_slope, this->cpu4_bias, this->noise);
}

// Compute CPU1 activity
void CentralComplex::cpu1_output(Eigen::Ref<Eigen::MatrixXd> tb1,
                                 Eigen::Ref<Eigen::MatrixXd> cpu4,
                                 Eigen::Ref<Eigen::MatrixXd> cpu1){
  cpu1 = (this->W_CPU4_CPU1 * cpu4) - (this->W_TB1_CPU1 * tb1);
  this->sigmoid(cpu1, this->cpu1_slope, this->cpu1_bias, this->noise);
}

// Might be a source of problems due to dot method
double CentralComplex::motor_output(Eigen::Ref<Eigen::MatrixXd> cpu1){
  return this->dot(this->W_CPU1_motor, cpu1);
}

//
// Completed usage functions
//
double CentralComplex::unimodal_monolithic_CX(double theta, double speed){
  double CXMotor = 0;

  this->tl2_output(theta, this->TL2);
  this->cl1_output(this->TL2, this->CL1);
  this->tb1_output(this->CL1, this->TB1);
  this->cpu4_update(speed, this->TB1, this->MEM); // Note, this just formats the inputs
  this->CPU4 = this->MEM; // Preserve CPU4 inputs for next timestep.
  this->cpu4_output(this->CPU4);
  this->cpu1_output(this->TB1, this->CPU4, this->CPU1);

  CXMotor = this->motor_output(this->CPU1);

  return CXMotor;
}

//
// Format and output CX internal status for the
//
void CentralComplex::get_status(std::vector<std::vector<double>> &activity){
  std::vector<double> tl2_vec(this->TL2.data(),
                              this->TL2.data() +
                              this->TL2.rows() * this->TL2.cols()
                              );
  std::vector<double> cl1_vec(this->CL1.data(),
                              this->CL1.data() +
                              this->CL1.rows() * this->CL1.cols()
                              );
  std::vector<double> tb1_vec(this->TB1.data(),
                              this->TB1.data() +
                              this->TB1.rows() * this->TB1.cols()
                              );
  std::vector<double> cpu4_vec(this->CPU4.data(),
                               this->CPU4.data() +
                               this->CPU4.rows() * this->CPU4.cols()
                               );
  std::vector<double> mem_vec(this->MEM.data(),
                              this->MEM.data() +
                              this->MEM.rows() * this->MEM.cols()
                              );
  std::vector<double> cpu1_vec(this->CPU1.data(),
                               this->CPU1.data() +
                               this->CPU1.rows() * this->CPU1.cols()
                               );

  activity.push_back(tl2_vec);
  activity.push_back(cl1_vec);
  activity.push_back(tb1_vec);
  activity.push_back(cpu4_vec);
  activity.push_back(mem_vec);
  activity.push_back(cpu1_vec);
}

#endif
