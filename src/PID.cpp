#include "PID.h"
#include <vector>
#include <iostream>
#include <uWS/uWS.h>
#include "cmath"
using namespace std;

PID::PID() {}
PID::~PID() {}


//Initialize the PID object
void PID::Init(double Kp_, double Ki_, double Kd_) {
  /**
   * TODO: Initialize PID coefficients (and errors, if needed)
   */
  this->Ki = Ki_;
  this->Kp = Kp_;
  this->Kd = Kd_;
  
  // Errors
  this-> p_error = 0;
  this->  i_error = 0;
  this->  d_error = 0;
  
  
  //Variables for Backpropagation
  cumulative_epoch_error = 0;
  current_epoch_error = 0;
  prev_error = 0;
  need_training = true;
  i_error_abs = 0;
}

//Update the errors
void PID::UpdateError(double cte) {
  /**
   * TODO: Update PID errors based on cte.

  */
  d_error = cte - p_error;
  p_error = cte;
  i_error += cte;
  
  UpdateError_epoch(cte);
  
  
}

//Calculate the steering
double PID::TotalError() {
  /**
   * TODO: Calculate and return the total error
  */
  return -Kp * p_error - Ki *  i_error - Kd * d_error;
}

//Functions for PID Hyperparameters adjusting
void PID::evaluate(){
  //Evaluates the rsme of the epoch
  if(need_training){
  //update values 
    current_epoch_error = sqrt(cumulative_epoch_error/epoch_size);
    std::cout<<"current epoch error: "<<current_epoch_error<<std::endl;
    need_training = current_epoch_error > tolerance;
    if (!need_training){
      std::cout<<"Training is complete. Error is:  "<<current_epoch_error<<std::endl;
    }
  }
}

void PID::backpropagation(){
  //Calculates the variation on epoch error and updates the hyperparameters
  double delta_error =  prev_error- current_epoch_error;
  prev_error = current_epoch_error;
  

  update(Kp, p_error, delta_error);
  update(Ki, i_error_abs, delta_error);
  update(Kd, d_error, delta_error);
  
  std::cout <<"New values are - Kp: " <<Kp<<"    Ki: "<<Ki<<"    Kd: "<<Kd << std::endl;
  std::cout <<"Current delta error: " <<delta_error<<std::endl;
}

void PID::update(double &Kx,double delta_p,double error){
  //Update the value based on error
  double temp_delta = Kx * delta_p * error * learn_rate;
  Kx -= temp_delta;
}

void PID::UpdateError_epoch(double cte){
  //Update the epoch error
  i_error_abs += fabs(cte);
  cumulative_epoch_error += pow(cte,2);  
}

void PID::reset_epoch(){
  //Reset the epoch error
  i_error_abs = 0;
  cumulative_epoch_error = 0; 
}


