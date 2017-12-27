#include "PID.h"
#include <iostream>

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {

	this->Kp = Kp;
	this->Ki = Ki;
	this->Kd = Kd;

	this->Kp_best = Kp;
	this->Ki_best = Ki;
	this->Kd_best = Kd;

	p_error = 0.2;
	i_error = 0.01;
	d_error = 1;

	steps = istuned? 35 : 0;
	mode = PID_normal_mode;

	prev_cte = 0;
	sum_cte = 0;
	abs_sum_cte = 0;
}

void PID::UpdateError(double cte) {

	if(cte < 0)
	{
		cte *= -1;
	}
	switch (mode)
	{
		case PID_normal_mode:

			best_err = cte;
			Kp += p_error;
			mode = P_high_mode;
			break;

		case P_high_mode:
			if(cte < best_err)
			{
				best_err = cte;
				Kp_best = Kp;
				p_error *= 1.25;

				Kd += d_error;
				mode = D_high_mode;
			}
			else
			{
				Kp -= 2*p_error;
				mode = P_low_mode;
			}
			break;

		case P_low_mode:
			if(cte < best_err)
			{
				best_err = cte;
				Kp_best = Kp;
				p_error *= 1.25;
			}
			else
			{
				Kp += p_error;
				p_error *= 0.75;
			}

			Kd += d_error;
			mode = D_high_mode;

			break;

		case D_high_mode:
			if(cte < best_err)
			{
				best_err = cte;
				Kd_best = Kd;
				d_error *= 1.25;

				Ki += i_error;
				mode = I_high_mode;

				/*
				Kp += p_error;
				mode = P_high_mode;
				*/

			}
			else
			{
				Kd -= 2*d_error;
				mode = D_low_mode;
			}
			break;

		case D_low_mode:
			if(cte < best_err)
			{
				best_err = cte;
				Kd_best = Kd;
				d_error *= 1.25;
			}
			else
			{
				Kd += d_error;
				d_error *= 0.75;
			}


			Ki += i_error;
			mode = I_high_mode;

			/*
			Kp += p_error;
			mode = P_high_mode;
			*/

			break;

		case I_high_mode:
			if(cte < best_err)
			{
				best_err = cte;
				Ki_best = Ki;
				i_error *= 1.1;

				Kp += p_error;
				mode = P_high_mode;
			}
			else
			{
				Ki -= 2*i_error;
				mode = I_low_mode;
			}
			break;

		case I_low_mode:
			if(cte < best_err)
			{
				best_err = cte;
				Ki_best = Ki;
				i_error *= 1.1;
			}
			else
			{
				Ki += i_error;
				i_error *= 0.9;
			}

			Kp += p_error;
			mode = P_high_mode;

			break;

		default:
			break;
	}

}

double PID::TotalError() {
	return (p_error + d_error + i_error);
}

