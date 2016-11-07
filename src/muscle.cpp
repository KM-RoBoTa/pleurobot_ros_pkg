#include <cmath>
#include <iostream>
#include "pleurobot_ros_pkg/muscle.hpp"

Muscle::Muscle(double a, double b, double c, double d, double r)
{
	d_alpha = a;
	d_beta = b;
	d_gamma = c;
	d_delta = d;
	d_resting_posture = r;
}

double
Muscle::Torque(double pos, double vel, double u1, double u2)
{
	double T;
	
	T =  d_alpha * (u1 - u2) 
       	- d_beta * (u1 + u2) * (pos - d_resting_posture)
          - d_gamma * (pos - d_resting_posture)
          - d_delta * vel;
     
	if( fabs(T) > 5.0)
	{
		if (T < 0.0) T = -5.0;
		if (T > 0.0) T =  5.0;
	}
	
	return(T);	
}
