#ifndef CX_CORE
#define CX_CORE

#include <iostream>
#include <random>
#include <math.h>
#include <exception>
#include <vector>
#include <string>

// Eigen linear algebra library: https://eigen.tuxfamily.org
#include "db_util/Eigen/Eigen"

// Using macro definitions because these values never change and I want to
// have these defaults known at compile time for initialisation.
#define CX_N_TL2 16
#define CX_N_CL1 16
#define CX_N_TB1 8
#define CX_N_CPU4 16
#define CX_N_CPU1 16

#define MAT_LOG(x) std::cout << std::endl << x << std::endl;
#define LOG(x) std::cout << "cx: " << x << std::endl;

// The central complex model as defined by Stone et al.
// Ported from the AntBot; Luca's code
// R. Mitchell
class CentralComplex {
protected:
  //
  // TODO: REMOVE? might not be needed.
  // Model parameters kept as variables
  //
  int n_tl2 = CX_N_TL2;
  int n_cl1 = CX_N_CL1;
  int n_tb1 = CX_N_TB1;
  int n_cpu4 = CX_N_CPU4;
  int n_cpu1 = CX_N_CPU1;
  double tb1_tb1_weight = 1;

  // Class parameters
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

  double cpu4_mem_gain = 0.0005;  //0.005, default
  double cpu4_mem_loss = 0.00026; //0.0026, default

  //Why do I have two of these? MEM and cpu4_mem; Think this was a mistake
  //  Eigen::Matrix<double, CX_N_CPU4, 1> cpu4_mem;

  double cpu1_slope = 6.0;
  double cpu1_bias = 2.0;

  // Anatomical matrices
  // Note: the dimensionality here may be incorrect, probable issue.
  Eigen::Matrix<double, CX_N_TB1, CX_N_CL1> W_CL1_TB1;
  Eigen::Matrix<double, CX_N_TB1, CX_N_TB1> W_TB1_TB1;
  Eigen::Matrix<double, CX_N_CPU1, CX_N_TB1> W_TB1_CPU1;
  Eigen::Matrix<double, CX_N_CPU4, CX_N_TB1> W_TB1_CPU4;
  Eigen::Matrix<double, CX_N_CPU1, CX_N_CPU4> W_CPU4_CPU1;
  Eigen::Matrix<double, 1, CX_N_CPU1> W_CPU1_motor;

  // Neural population matrices
  Eigen::Matrix<double, CX_N_TL2, 1> TL2;
  Eigen::Matrix<double, CX_N_CL1, 1> CL1;
  Eigen::Matrix<double, CX_N_TB1, 1> TB1;
  Eigen::Matrix<double, CX_N_CPU4, 1> MEM;
  Eigen::Matrix<double, CX_N_CPU4, 1> CPU4;
  Eigen::Matrix<double, CX_N_CPU1, 1> CPU1;

  // Function pointer and declarations for sigmoid
  // Means we can define noise or none at construction.
  // I don't think noise was ever used in the AntBot.
  // Function pointers can be updated at runtime if necessary.
  // The Eigen::Ref<T> type is an eigen reference to an object
  // of type T; required for use of Eigen types in functions.
  void (*sigmoid) (Eigen::Ref<Eigen::MatrixXd>, double, double, double);
  static void noisySigmoid (Eigen::Ref<Eigen::MatrixXd> v,
                            double slope,
                            double bias,
                            double noise);
  static void noiselessSigmoid (Eigen::Ref<Eigen::MatrixXd>,
                                double,
                                double,
                                double);

  // For sanity; Eigen's dot product only works on Vector types.
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
  CentralComplex(){
    // TL2_PREFS
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

    //Old version, this was ported directly from the AntBot;
    // I changed it so that the weight matrix is initialised correctly rather
    //than transposing at runtime, kept here for history's sake, at least for one
    //commit.

    /*
    W_CPU4_CPU1 <<
      0,  1,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
      0,  0,  0,  0,  0,  0,  0,  0,  1,  0,  0,  0,  0,  0,  0,  0,
      0,  0,  0,  0,  0,  0,  0,  0,  0,  1,  0,  0,  0,  0,  0,  0,
      0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  1,  0,  0,  0,  0,  0,
      0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  1,  0,  0,  0,  0,
      0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  1,  0,  0,  0,
      0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  1,  0,  0,
      1,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
      0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  1,
      0,  0,  1,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
      0,  0,  0,  1,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
      0,  0,  0,  0,  1,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
      0,  0,  0,  0,  0,  1,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
      0,  0,  0,  0,  0,  0,  1,  0,  0,  0,  0,  0,  0,  0,  0,  0,
      0,  0,  0,  0,  0,  0,  0,  1,  0,  0,  0,  0,  0,  0,  0,  0,
      0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  1,  0;
    W_CPU4_CPU1.transposeInPlace();
    MAT_LOG(W_CPU4_CPU1);
    */

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

    //    MAT_LOG(W_CPU4_CPU1);

    // Set sigmoid function; voids unnecessary if-checks and looping
    // if we have no noise.
    sigmoid =
      (noise > 0) ?
      &CentralComplex::noisySigmoid :
      &CentralComplex::noiselessSigmoid;

    //Initialise neural populations.
    TL2.setConstant(0);
    CL1.setConstant(0);
    TB1.setConstant(0);
    MEM.setConstant(0.5);
    CPU4.setConstant(0);
    CPU1.setConstant(0);

    LOG("CX init complete");
  }

  // Layer-wise functions for CX Operation
  void tl2_output(double theta, Eigen::Ref<Eigen::MatrixXd> output);
  void cl1_output(Eigen::Ref<Eigen::MatrixXd> tl2,
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
  double unimodal_monolithic_CX(double theta, double speed);

  // Status
  void get_status(std::vector<std::vector<double>> &activity);
};

//
// Definitions
//

//
// Protected methods
//

/*
  Original version had a bunch of getters and setters which I don't think
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

// Wrote this before realising that Luca's version isn't a dot
// product, it's just a basic matrix multiplication. I have no idea
// why he includes it.

// // Sometimes the matrix objects are Nx1/1xN i.e. vectors.
// // If we want to dot these, Eigen says no so we do our own.
// // Make sure both Matrix references are vectors
 double CentralComplex::dot(Eigen::Ref<Eigen::MatrixXd> a,
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
// Public
//

//
// Layer-wise CX functions; these can be strung together to form a complete
// network process. This is ported from the AntBot implementation and
// I never understood the implementation, I've included a monolithic
// or "black-box" version which takes an input angle and speed, then produces an
// output. These are here for legacy/convenience/utility should they be required.
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

  for (int i = 0; i < output.rows(); i++){
    output(i,0) = cos(output(i,0));
  }

  this->sigmoid(output, this->tl2_slope, this->tl2_bias, this->noise);
}

// Compute CL1 output, store in m argument
void CentralComplex::cl1_output(Eigen::Ref<Eigen::MatrixXd> tl2,
                                Eigen::Ref<Eigen::MatrixXd> cl1){
  cl1 = -1 * tl2;
  this->sigmoid(cl1, this->cl1_slope, this->cl1_bias, this->noise);
}

// Compute TB1 layer output, store in tb1 argument.
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
// reference. Note: This is the code from Luca. Tom's code in the paper
// is different, I want to test both to compare.
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
// Now that I've written this out I don't think this method is necessary
// Update: it's "necessary" due to the fact that sometimes we update CPU4
// but don't care about the output, i.e you avoid running the sigmoid
// when you don't need to.
// Update2: You also need the unsigmoided CPU4 input to feed back into
// the circuit at the next timestep.
void CentralComplex::cpu4_output(Eigen::Ref<Eigen::MatrixXd> cpu4_mem){
  this->sigmoid(cpu4_mem, this->cpu4_slope, this->cpu4_bias, this->noise);
}

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

/*
  This was a first approach to taking a "snapshot" and in theory it should
  work identically to the VM method. The only difference is where the snapshot
  is stored really. This function is for extremely specific use only. It should
  only be used in the case where we desire simple orientation, otherwise it won't
  work.
*/
void CentralComplex::set_goal_direction(double theta){
  double speed = 1;
  double pi = 3.14159265358979323846; //For convenience...

  // Assuming theta is in radians, flip it by pi. This makes
  // more sense as a goal direction should be where we want
  // to go.
  theta = theta - pi;

  LOG("CX set goal direction");

  this->tl2_output(theta, this->TL2);
  this->cl1_output(this->TL2, this->CL1);
  this->tb1_output(this->CL1, this->TB1);

  // Preserve unsigmoided CPU4 for next timestep
  this->cpu4_update(speed, this->TB1, this->MEM);
  this->CPU4 = this->MEM;

  this->cpu4_output(this->CPU4);

  LOG("goal direction set successfully");
}

double CentralComplex::unimodal_monolithic_CX(double theta, double speed){
  double CXMotor = 0;

  this->tl2_output(theta, this->TL2);
  this->cl1_output(this->TL2, this->CL1);
  this->tb1_output(this->CL1, this->TB1);

  // Preserve unsigmoided CPU4 for next timestep
  this->cpu4_update(speed, this->TB1, this->MEM);
  this->CPU4 = this->MEM;



  // Worth noting, after this call the CPU4 values all sit at about 0.99....
  // They are updated, just by minute amounts
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
