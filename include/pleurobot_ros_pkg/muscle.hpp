#ifndef MUSCLE_HPP
#define MUSCLE_HPP

#define NUM_MUSCLE_PARAMETERS 6

enum{IS_MUSCLE, GAIN, ACTIVE_STIFFNESS, PASSIVE_STIFFNESS, DAMPING, RESTING_POSITION};

class Muscle
{
	double d_alpha, 
		  d_beta,
		  d_gamma,
		  d_delta,
		  d_resting_posture;
	
	public:
	
		Muscle(double, double, double, double, double);
		double Torque(double, double, double, double);	 
};

#endif
