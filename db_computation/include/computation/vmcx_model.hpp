#ifndef _VMCX
#define _VMCX

#include <iostream>
#include <random>
#include <math.h>
#include <exception>
#include <vector>

// Eigen linear algebra library: https://eigen.tuxfamily.org
#include "db_util/Eigen/Eigen"

// Definitions for neuron population sizes are included here, do not
// override them.
#include "cx_model.hpp"

// [V]ector[M]emory[C]entralComple[X]
// Central complex network capable of storing a single vector memory.
// In the context of the beetle this represents multiple celestial
// snapshots. Vector Memory architecture taken from Le Moel et al. 2019.
class VMCX : public CentralComplex{
private:
  //
  // The core is brought in from CentralComplex, we just need the
  // singular vector memory components.
  //
  // Weight matrix
  Eigen::Matrix<double, CX_N_CPU4, 1> W_VM;

  // Vector memory neuron, basically acts as an on/off
  // switch. Should be 1 (max rate) when using vector memory,
  // 0 when not.
  double VM = 0;
  double baseline = 0.5;

public:
  VMCX(){
    this->W_VM.setConstant(0.5);
  }

  // Need to be able to store a vector memory
  void store_vm();

  // Clearing vector memory, for whatever reason.
  void clear_vm();

  // Want to be able to turn the system on or off
  void activate_vm();
  void deactivate_vm();

  // Toggle is more convenient, returns the new state at the end of the
  // function call
  bool toggle_vm();

  // Get the current status of the vm system (on or off)
  bool vm_status();

  // Single function for VMCX operation.
  double unimodal_VMCX(double theta, double speed);

  //
  // Overrides
  //

  // Override the CPU1 update rule
  void cpu1_output(Eigen::Ref<Eigen::MatrixXd> tb1,
                   Eigen::Ref<Eigen::MatrixXd> cpu4,
                   Eigen::Ref<Eigen::MatrixXd> cpu1);

  void get_status(std::vector<std::vector<double>>&);
};

//
// Public
//
// Question: Should I be using this->MEM here or this->CPU4?
// I think CPU4 is correct here but this may be an issue
void VMCX::store_vm(){
  //this->W_VM = -1 * this->CPU4;
  this->clear_vm(); //reset to 0.5
  //this->W_VM = -1 * this->CPU4; //-(rCPU4) : Le Moel et al.
  //this->W_VM = -1 * this->MEM; //-(Icpu4 - 0.5)
  //this->W_VM = -1 * (this->CPU4 - this->W_VM); //-(rCPU4 - 0.5)
  this->W_VM = -1 * (this->MEM - this->W_VM); //-(Icpu4 - 0.5)
  ROS_INFO("VM STORED");
}

void VMCX::clear_vm(){
  this->W_VM.setConstant(0.5);
}

// Turn on, no value check
void VMCX::activate_vm(){
  this->VM = 1;
  ROS_INFO("VM ACTIVATED");
}

// Turn off, no value check
void VMCX::deactivate_vm(){
  this->VM = 0;
  ROS_INFO("VM DEACTIVATED");
}

// Invert signal
bool VMCX::toggle_vm(){
  this->VM = -1 * (VM - 1); 
  std::string state = this->vm_status() ? "ACTIVATED" : "DEACTIVATED";
  ROS_INFO("VM %s", state.c_str());
  return this->vm_status();
}

// Return boolean conversion of VM activity
bool VMCX::vm_status(){ return this->VM == 1 ? true : false; };

/* VMCX operation
   Usage:
   - To update the CX without using it for steering, ignore the output
   - To use the CX for homing call VMCX::deactivate_vm() and then use the
     function return value to steer.
   - To use the VM component call VMCX::actiate_vm() and then use the
     return value to steer. Note you need to store a VM first.
 */
double VMCX::unimodal_VMCX(double theta, double speed){
  double CXMotor = 0;

  this->tl2_output(theta, this->TL2);
  this->cl1_output(this->TL2, this->CL1);
  this->tb1_output(this->CL1, this->TB1);
  this->cpu4_update(speed, this->TB1, this->MEM);

  this->CPU4 = this->MEM; // Copy unsigmoided CPU4 state
  this->cpu4_output(this->CPU4); // Squash

  this->cpu1_output(this->TB1, this->CPU4, this->CPU1);

  CXMotor = this->motor_output(this->CPU1);

  return CXMotor;
}

//
// Overrides
//
void VMCX::cpu1_output(Eigen::Ref<Eigen::MatrixXd> tb1,
                       Eigen::Ref<Eigen::MatrixXd> cpu4,
                       Eigen::Ref<Eigen::MatrixXd> cpu1){
  cpu1 =
    this->W_CPU4_CPU1 * (cpu4 + (this->W_VM * this->VM))
    - (this->W_TB1_CPU1 * tb1);
     // Include the VM component

  this->sigmoid(cpu1, this->cpu1_slope, this->cpu1_bias, this->noise);
}

// Will need to override CPU1 update rule but wait out to get the rest
// working first


//
// Format and output VMCX internal status for the
//
void VMCX::get_status(std::vector<std::vector<double>> &activity){
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
  std::vector<double> vm_vec(this->W_VM.data(),
                             this->W_VM.data() +
                             this->W_VM.rows() * this->W_VM.cols()
                             );

  Eigen::Matrix<double, 1, 16> active_mat =
    this->W_CPU4_CPU1 * (this->CPU4 +(this->VM * this->W_VM));

  std::vector<double> active_vm(active_mat.data(),
                                active_mat.data() +
                                active_mat.rows() * active_mat.cols()
                                );

  for (int i = 0; i < active_vm.size(); i++){
    ROS_INFO("%f\n", active_vm[i]);
  }

  activity.push_back(tl2_vec);
  activity.push_back(cl1_vec);
  activity.push_back(tb1_vec);
  activity.push_back(cpu4_vec);
  activity.push_back(mem_vec);
  activity.push_back(cpu1_vec);
  activity.push_back(vm_vec);
  activity.push_back(active_vm);
}

#endif
