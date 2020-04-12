#ifndef PID_H
#define PID_H
#include <vector>

class PID {
 public:
  /**
   * Constructor
   */
   
   double p_error;
   double i_error;
   double d_error;
   
   double Kp;
   double Ki;
   double Kd;
   
  PID();

  /**
   * Destructor.
   */
  virtual ~PID();

  /**
   * Initialize PID.
   * @param (Kp_, Ki_, Kd_) The initial PID coefficients
   */
  void Init(double Kp_, double Ki_, double Kd_, double p_error_, double i_error_, double d_error_);

  /**
   * Update the PID error variables given cross track error.
   * @param cte The current cross track error
   */
  double Err(std::vector<double> cte_list);
   
  bool Twiddle(std::vector<double> cte_list,bool abs_err); 
   
  void UpdateError(double cte);

  /**
   * Calculate the total PID error.
   * @output The total PID error
   */
  double TotalError();

	void Show();

 private:
  /**
   * PID Errors
   */
  
};

#endif  // PID_H