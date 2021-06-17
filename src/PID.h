#ifndef PID_H
#define PID_H
#include <vector>
#include <iostream>
#include <uWS/uWS.h>

class PID {
 public:
  /**
   * Constructor
   */
  PID();

  /**
   * Destructor.
   */
  virtual ~PID();

  
  void Init(double Kp_, double Ki_, double Kd_);

  void UpdateError(double cte);
  
  double TotalError();
  
  void evaluate();
  
  void backpropagation();
  
  void update(double& K,double delta_p,double error);
  
  void UpdateError_epoch(double cte);
  
  void reset_epoch();
  

  ///// Variables /////
  
  /**
   * PID Errors
   */
  double p_error;
  double i_error;
  double d_error;

  /**
   * PID Coefficients
   */ 
  double Kp;
  double Ki;
  double Kd;
  
  /**
   * PID helpers
   */ 
    
  //Training parameters for backpropagation:
  
  double learn_rate = 1e-3;
  double tolerance = 1e-2;
  bool need_training;
  double cumulative_epoch_error;
  double current_epoch_error;
  double prev_error;
  int epoch_size = 150;
  double i_error_abs;
  
  
};

#endif  // PID_H