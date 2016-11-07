#include "pleurobot_ros_pkg/controller.h"

using namespace std;
using namespace Eigen;





/* Get force sensors */
void 
Controller :: getForce(double *force)
{
    MatrixXd ftmp(12,1);
    for(int i=0; i<12; i++){
        ftmp(i)=force[i];
    }
    feet_force.block<3,1>(0,0)=HJfl_g[4].block<3,3>(0,0)*ftmp.block<3,1>(0,0);
    feet_force.block<3,1>(0,1)=HJfr_g[4].block<3,3>(0,0)*ftmp.block<3,1>(3,0);
    feet_force.block<3,1>(0,2)=HJhl_g[4].block<3,3>(0,0)*ftmp.block<3,1>(6,0);
    feet_force.block<3,1>(0,3)=HJhr_g[4].block<3,3>(0,0)*ftmp.block<3,1>(9,0);

    slippingFromFrictionCone();
}


/* Get force sensors */
void
Controller :: slippingFromFrictionCone()
{
	static MatrixXd fc_feet_force_old=MatrixXd::Zero(3, 4);
	static MatrixXd is_slipping(4,1);
	static double coneAngle = atan(coneFriction);
	// filter forces
    fc_feet_force=pt1_vec(feet_force, fc_feet_force_old, coneForceFilter, dt);
	fc_feet_force_old=fc_feet_force;

	//detect slipping
	for(int i=0; i<4; i++){
		feet_friction_angles(i)=atan(
			sqrt( fc_feet_force(0, i)*fc_feet_force(0, i) + fc_feet_force(1, i)*fc_feet_force(1, i) ) / 
			fc_feet_force(2, i)
			);
		if(abs(feet_friction_angles(i))>coneAngle && 
			sqrt( fc_feet_force(0, i)*fc_feet_force(0, i) + 
				fc_feet_force(1, i)*fc_feet_force(1, i) +
				fc_feet_force(2, i)*fc_feet_force(2, i)) > forceTreshold){
			feet_is_slipping(i)=1;
		}
		else
			feet_is_slipping(i)=0;
	}
}

