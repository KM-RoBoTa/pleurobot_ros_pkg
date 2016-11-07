#include "pleurobot_ros_pkg/controller.h"

using namespace std;
using namespace Eigen;

extern int IS_SIMULATION, USE_JOYSTICK, JOYSTICK_TYPE, SWIM, SPINE_COMPENSATION_WITH_FEEDBACK, USE_REFLEXES;


bool
Controller :: DirectTorqueSetup()
{
    

    if(!updateState()){
        return false;
    }
    
    
    t+=dt;
    
    for(int i=0; i<27;i++){
	torques(i)=0;
    }
    
    torques(18)=0.1*joy_y1;

    
    
    return true;
}





/* PID torque controller to follow angle references coming from inverse kinematics */
void
Controller :: torquePID(){

	static MatrixXd pos_error(27,1);
	static MatrixXd D_pos_error=MatrixXd::Zero(27,1), I_pos_error=MatrixXd::Zero(27,1), old_pos_error=MatrixXd::Zero(27,1), old_torques=MatrixXd::Zero(27,1);
	pos_error=-(angles-fbck_position);

	static ofstream torquePIDlog("./data/torquePIDlog.txt");


	D_pos_error=pt1_vec((pos_error-old_pos_error)/dt, D_pos_error, torque_D_filt, dt);


	pos_error=pt1_vec(pos_error, old_pos_error, torque_error_filt, dt);


	
	I_pos_error+=I_pos_error*dt;


	torques=pos_error.cwiseProduct(Pgain_torques) + D_pos_error.cwiseProduct(Dgain_torques) + I_pos_error.cwiseProduct(Igain_torques);


	for(int i=0; i<27; i++){
		if(torques(i)>7)
			torques(i)=7;
		if(torques(i)<-7)
			torques(i)=-7;
	}


	old_pos_error=pos_error;

	torques=pt1_vec(torques, old_torques, output_torque_filt, dt);
	old_torques=torques;


	torquePIDlog<<t<<"\t"<<angles.transpose()<<"\t"<<fbck_position.transpose()<<"\t"<<torques.transpose()<<endl;
}



/* VMC component to help keep body up and horizontal */
void 
Controller :: VMC_bodyPosture(){

	// nominal reference for a body height
	static Vector4d z_spring_diff, z_spring_diff_old=VectorXd::Zero(4);

	z_spring_diff(0)=-(pFL(2)-HJfl[4](2,3));
	z_spring_diff(1)=-(pFR(2)-HJfr[4](2,3));
	z_spring_diff(2)=-(pHL(2)-HJhl[4](2,3));
	z_spring_diff(3)=-(pHR(2)-HJhr[4](2,3));


	// get spring force
	static Vector4d z_spring_force=VectorXd::Zero(4), z_spring_force_old=VectorXd::Zero(4);

	z_spring_force=vmc_bodyPosture_stiffness*z_spring_diff + vmc_bodyPosture_damping*(z_spring_diff-z_spring_diff_old)/dt;
	//legJacob[0]


	
	//z_spring_force_vec[0] = (MatrixXd(3,1) << 0, 1, 0).finished();



	// get torques
	static vector<Vector3d> z_spring_force_vec(4);
	static vector<Vector4d> z_spring_torques_vec(4);

	for(int i=0; i<4; i++){
		z_spring_force_vec[i] << 0,0,z_spring_force(i);
		z_spring_torques_vec[i]=legJacob[i].transpose()*z_spring_force_vec[i];

		if(legs_stance(i)){
			torques.block<4,1>(11+4*i,0)+=z_spring_torques_vec[i];
		}

	}

	//cout<<z_spring_torques_vec[0]<<endl;





}








