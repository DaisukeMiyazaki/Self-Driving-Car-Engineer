#include "PID.h"
#include <vector>
#include <cmath>
#include <numeric>
#include <iostream>

#define INCREASE 0
#define DECREASE 1


/**
 * TODO: Complete the PID class. You may add any additional desired functions.
 */

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp_, double Ki_, double Kd_,double p_error_, double i_error_, double d_error_) {
  /**
   * TODO: Initialize PID coefficients (and errors, if needed)
   */
   Kp = Kp_;
   Ki = Ki_;
   Kd = Kd_;
   
   p_error = p_error_;
   i_error = i_error_;
   d_error = d_error_;

}

double PID::Err(std::vector<double> cte_list){

	/* Return the total err value of the trajectory */
	
	double total_err = 0.0;
	int size = cte_list.size();
	int half_size = size / 2;

	if(size != 0)
	{
		for(int i = half_size ; i < size ; i++)
		{
			total_err += pow(cte_list[i],2);
		}
			total_err /= half_size;
	}
	return total_err;

}

bool PID::Twiddle(std::vector<double> cte_list,bool abs_err){
	
	/* Return bool value for Twiddle */
	/* true if continued, else false */
	
	double traj_err = PID::Err(cte_list);
	if(abs_err)
	{
		traj_err = 100000;
	}
	
	static double best_err = traj_err;
	
	
	std::vector<double> p = {Kp,Ki,Kd};
	static std::vector<double> dp = {Kp*0.1,Ki*0.1,Kd*0.1};
	
	static std::vector<double> copy_dp = {dp[0]*0.1,dp[1]*0.1,dp[2]*0.1};
	double sum_dp = fabs(dp[0]) + fabs(dp[1]) + fabs(dp[2]);
	
	static int index = 0;
	static char step = -1;
	
	
	switch(step)
	{
		case INCREASE:
			if(traj_err < best_err || copy_dp[index] > dp[index])
			{
				best_err = traj_err;
				p[index] += dp[index];
				dp[index] *= 1.1;
				index = ( index + 1 ) % 3;
			}	
			break;
		case DECREASE:
			if(traj_err < best_err || copy_dp[index] > dp[index])
			{
				best_err = traj_err;
				p[index] -= dp[index];
				dp[index] *= 1.1;
				index = ( index + 1 ) % 3;
			}
			else
			{
				dp[index] *= 0.5;
				//step = INCREASE;
			}
			break;
		
		default:
			step = DECREASE;
			break;
			/* DO NOTHING */

	}
	if(copy_dp[0] > dp[0] && copy_dp[1] > dp[1] && copy_dp[2] > dp[2])
	{
		return false;
	}
	else
	{
	/* when changing p worsens total err */
		switch(step)
		{
			case INCREASE:
				p[index] -= 2 * dp[index];
				step = DECREASE;
				break;
		
			case DECREASE:
				p[index] += dp[index];
				step = INCREASE;
				break;
					
			default:
				step = INCREASE;
				break;
				/* DO NOTHING */
		}	
	}

	PID::Init(p[0],p[1],p[2],0,0,0);
	
		
	std::cout << "first cte" << cte_list[700] << std::endl;
	std::cout << "err: " << traj_err << std::endl;	
	std::cout << "sum: " << sum_dp << std::endl;
	std::cout << "---------" << std::endl;
	
	return true;
}

void PID::UpdateError(double cte) {
  /**
   * TODO: Update PID errors based on cte.
   */
   
   d_error = cte - p_error;
   i_error += cte;
   p_error = cte;

}

double PID::TotalError() {
  /**
   * TODO: Calculate and return the total error
   */
   double alpha = -Kp * p_error - Ki * i_error - Kd * d_error;
   if(alpha < -1 ){
   		alpha = -1;
   }
   else if(alpha > 1){
   		alpha = 1;
   }
   return alpha;  // TODO: Add your total error calc here!
}

void PID::Show(){

	std::cout << "-------------------------------" << std::endl;
	std::cout << "Kp: " << Kp << " Ki: " << Ki << " Kd: " << Kd << std::endl;
	std::cout << "-------------------------------" << std::endl;
}