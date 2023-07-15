#ifndef STATIC_ERM
#def STATIC_ERM

/**
   @file static_erm.hpp
   @brief Provides a port of the neural cue integration model given by [1]
   @author Robert Mitchell

   This header file provides an implementation of the static variant of the
   head direction circuit cue integration model provided by [1]. This version
   of the model uses only the default R to E-PG mappings has defined in [1] and
   does not include any plasticity.

   The number of R neurons is fixed at 8 primarily for ease; it's
   useful to know the population sizes at compile time. R group sizes
   are also assumed to be the same size. [1] provided evidence that
   the number of R neurons does not change the behaviour of the model
   unless R group sizes are imbalanced.

   References
   1. Mitchell, Shaverdian, Dacke, and Webb (2023). A model of cue integration
      as vector summation in the insect brain.
 **/

#include <iostream>
#include <cmath>
#include <vector>
#include <string>
#include <algorithm>

// Eigen linear algebra library: https://eigen.tuxfamily.org
#include "bb_util/Eigen/Eigen"

/**
   @defgroup ERM_PARAMS Ring Model neuron counts
   @brief Neuron counts for each neuron class in the model.
   @{
*/

#define ERM_N_R 8    // TL2 neurons
#define ERM_N_EPG 8  // CL1a
#define ERM_N_D7 8   // TB1
#define ERM_N_PEN 16 // CL2
#define ERM_N_PEG 16 // CL1b

/**@}*/

/** Matrix printing macro. */
#define MAT_LOG(x) std::cout << std::endl << x << std::endl;

/** General printing macro. */
#define LOG(x) std::cout << "erm: " << x << std::endl;

/**
   @brief Class implementation of the ring model from [1].
 */
class RingModel{
protected:
  /**
     @defgroup NET_PARAM Network parameters
     @brief the parameters used to define the network dynamics
  */

  ///
  /// Neuron counts
  ///
  int n_r1 = ERM_N_R;
  int n_r2 = ERM_N_R;
  int n_epg = ERM_N_EPG;
  int n_pen = ERM_N_PEN;
  int n_peg = ERM_N_PEG;

  ///
  /// EPG out
  ///
  float sc_epg_peg = 1.2;
  float sc_epg_pen = 0.8;
  float sc_epg_d7 = 0.5;

  ///
  /// D7 out
  ///
  float sc_d7_peg = -0.3;
  float sc_d7_pen = -0.6;
  float sc_d7_d7 = 0.1;

  ///
  /// PEN/PEG out
  ///
  float sc_peg_epg = 0.5;
  float sc_pen_epg = 1.4;

  ///
  /// Self-motion
  ///
  float sc_sm_pen = 1;

  ///
  /// Default weight values
  ///
  float d_w1 = 0.5;
  float d_w2 = 0.5;


  ///
  /// Rate parameters
  ///
  float r_slope = 4;
  float r_bias = 1.8;
  float epg_slope = 4;
  float epg_bias = 1.8;
  float d7_slope = 3;
  float d7_bias = 3;
  float peg_slope = 4;
  float peg_bias = 3;
  float pen_slope = 4;
  float pen_bias = 5;


}

  RingModel::RingModel(){

    
  }


  Eigen::Ref<Eigen::MatrixXd> RingModel::generate_mapping(int n_r,
                                                          std::vector<float> r_prefs,
                                                          int n_epg,
                                                          std::vector<float> epg_prefs){
    Eigen::MatrixXd w(n_epg, n_r);
    w.setZero();

    for (int i = 0; i < n_r; i++){
      float r = r_prefs[i];
      std::vector<int> indices(0);

      for (int j = 0; j < n_j; j++){
        float e = epg_prefs[j];

        Eigen::Vector2f r_cart(std::cos(r), std::sin(r));
        Eigen::Vector2f e_cart(std::cos(e), std::sin(e));

        // Compute inner angle between cartesian representations
        arg = r_cart.dot(e_cart) / (r_cart.norm() + e_cart.norm());

        // Clip between -1 and 1, rounding was a problem in Python
        arg = std::max(-1, std::min(arg, 1));
        angle = std::acos(arg);

        // Weight inverse proportionally to distance
        w[j,i] = bb_util::defs::PI - angle;
      }

      // Normalise
      auto slice = Eigen::seq(0,n_epg - 1);
      w(slice, i) /= w(slice,i).sum();
    }

    return w;
  }

#endif
