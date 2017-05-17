#ifndef PID_H
#define PID_H

class PID {
public:

  //Timestamps
  int last_timestamp;
  int current_timestamp;
  double dt;

  //track total cte error
  double total_error;

  //last output
  double last_output;
  /*
  * Errors
  */
  double p_error;
  double i_error;
  double i_error_max;
  double d_error;

  /*
  * Coefficients
  */
  double Kp;
  double Ki;
  double Kd;

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
  void Init(double Kp, double Ki, double Kd, double i_err_max);

  /*
  * Update the PID error variables given cross track error.
  */
  void UpdateError(double cte);

  /*
  * Calculate the total PID error.
  */
  void TotalError(double cte);

  double control_out(void);
};

#endif /* PID_H */
