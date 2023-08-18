#ifndef STATIC_ERM
#def STATIC_ERM

/**
   \file erm_plus_steering.hpp
   \brief Provides a C++ port of the RingModel cue integration model given by Mitchell
   et al. (2023) along with an implementation of the fruit-fly steering circuit.
   \author Robert Mitchell
   
   \warning This class is under development and there is no guarantee
   that it is complete or works at all. A working version of the
   extended ring model is provided in
   bb_computation/scripts/extended_ring_model.py. This can be tested
   with dummy cues by running.

   \verbatim
   $ roslaunch bb_launchers ci_dummy.launch
   \endverbatim

   This header file is planned to contain a C++ implementation of the 
   extended_ring_model.RingModel class from Mitchell et al. (2023) alongside
   an up-to-date goal direction and steering circuit from the fruit fly 
   connectome.
   
   References

   Mitchell, Shaverdian, Dacke, and Webb (2023). A model of cue integration
   as vector summation in the insect brain.
 **/

#include <iostream>
#include <cmath>
#include <vector>
#include <string>
#include <algorithm>

// Eigen linear algebra library: https://eigen.tuxfamily.org
#include "bb_util/Eigen/Eigen"

#define ERM_N_R 8    // TL2 neurons
#define ERM_N_EPG 8  // CL1a
#define ERM_N_D7 8   // TB1
#define ERM_N_PEN 16 // CL2
#define ERM_N_PEG 16 // CL1b


/** Matrix printing macro. */
#define MAT_LOG(x) std::cout << std::endl << x << std::endl;

/** General printing macro. */
#define LOG(x) std::cout << "erm: " << x << std::endl;

/**
   \brief Class implementation of the ring model from Mitchell et al. 2023).

   

   \warning This class is under active development and there is no guarantee
   that it works at all. A working version of the extended ring model is provided
   in bb_computation/scripts/extended_ring_model.py. This can be tested with dummy
   cues by running.
   \verbatim
   $ roslaunch bb_launchers ci_dummy.launch
   \endverbatim
   
 */
class RingModel{
protected:
  int n_r1 = ERM_N_R; /**< \brief The number of R1 neurons. */
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
