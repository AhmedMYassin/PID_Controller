#ifndef PID_H
#define PID_H

class PID {
public:
  /*
  * Errors
  */
  double p_error;
  double i_error;
  double d_error;

  double prev_cte;
  double sum_cte;
  double abs_sum_cte;

  double best_err;
  int steps;
  bool istrained;
  enum PID_mode { PID_normal_mode, P_high_mode, P_low_mode, D_high_mode, D_low_mode, I_high_mode, I_low_mode} mode;

  /*
  * Coefficients
  */ 
  double Kp;
  double Ki;
  double Kd;

  double Kp_best;
  double Ki_best;
  double Kd_best;

  /*
  * Constructor
  */
  PID();

  /*
  * Destructor.
  */
  virtual ~PID();

  /*
  * Initialize PID.
  */
  void Init(double Kp, double Ki, double Kd);

  /*
  * Update the PID error variables given cross track error.
  */
  void UpdateError(double cte);

  /*
  * Calculate the total PID error.
  */
  double TotalError();
};

#endif /* PID_H */
