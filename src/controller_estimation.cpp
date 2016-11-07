 #include "pleurobot_ros_pkg/controller.h"

using namespace std;
using namespace Eigen;

extern int IS_SIMULATION, IS_PLEUROBOT, USE_JOYSTICK, JOYSTICK_TYPE, SWIM, SPINE_COMPENSATION_WITH_FEEDBACK, USE_REFLEXES;


double
Controller :: arxModel(MatrixXd u, MatrixXd y0, MatrixXd A, MatrixXd B, MatrixXd nK, double NoiseVariance, bool NoiseIntegration){

	int ny=A.cols();
	int nu=B.rows();
	int nb=B.cols();

	MatrixXd y(ny, 1);
	y.block(1, 0, ny-1, 1)=y0.block(0, 0, ny-1, 1);

	double Bu=0;
	/*for(int i=0; i<nu; i++){
		Bu+=(B.block(i, 0, 1, nb)*u.block(nK(i), i, nb, 1)).sum();
	}*/
	Bu=B.cwiseProduct(u.block(0,0, nu, nb)).sum();

	MatrixXd Ay=A.block(0,1,1,ny-1)*y0.block(0,0,ny-1,1);
	y(0)= Ay(0) + Bu;
	

	return y(0);


}



double
Controller :: bjModel(MatrixXd u, MatrixXd y0, MatrixXd B, MatrixXd C, MatrixXd D, MatrixXd F, MatrixXd nK, double NoiseVariance, bool NoiseIntegration){
	//int ny=y0.rows();
	int nu=B.rows();
	int nb=B.cols();
	int nw=F.rows();
	int nf=F.cols();
	double y;

	static MatrixXd w=MatrixXd::Zero(nf, nw);
	

	MatrixXd wi(1, nw);


	for(int i=0; i<nu; i++){
		wi(i)=0;
		for(int j=0; j<nb; j++){
			wi(i)+=B(i, j)*u(j, i);
		}
		for(int j=1; j<nf; j++){
			wi(i)-=F(i, j)*w(j, i);
		}
	}
	y=wi.sum();
	w.block(1, 0, nf-1, nw)=w.block(0, 0, nf-1, nw);
	w.block(0, 0, 1, nw)=wi;
 		

	return y;
}


double
Controller :: speedEstimation(){
	static MatrixXd u_new(1,12);
	static MatrixXd u=MatrixXd::Zero(20, 12), y0=MatrixXd::Zero(20,1);
	double y; 

	static int cnt=0;
	if(cnt++<1000)
		return 0;

	// get acc data
	u_new(0)=accData[0];
	u_new(1)=-accData[2];
	u_new(2)=accData[1];

	// get norm of forces
	for(int i=0; i<4; i++){
		u_new(3+2*i)=sqrt(fc_feet_force(3*i)*fc_feet_force(3*i) + fc_feet_force(3*i+1)*fc_feet_force(3*i+1));
		u_new(3+2*i+1)=fc_feet_force(3*i+2);
	}

	// ferquency
	u_new(11)=freq_walk;

	// remove gravity
	u_new(2)-=9.81;

	u.block<19,12>(1,0)=u.block<19,12>(0,0);
	u.block<1,12>(0,0)=u_new;

	// use ARX model
	//y=arxModel(u, y0, ARXA, ARXB, ARXnK, 0, 1);

	// use BJ model
	y=bjModel(u, y0, BJB, BJC, BJD, BJF, BJnK, 0, 1);


	y0.block(1, 0, 19, 1)=y0.block(0, 0, 19, 1);
	y0(0)=y;

	
	if((cnt)%20==0){
		//cout<<fc_feet_force.block<3,1>(0,0)<<endl<<endl;
	}


	return y;
}

void
Controller :: GetVelocityFromGPS(){
	static Vector3d gps_old=gpsPos, gps_rot, estVelFromGps_old;

	gps_rot=rotMatFgird.inverse()*gpsPos;



	estVelFromGps=pt1_vec((gps_rot-gps_old)/dt, estVelFromGps_old, 2, dt);

 	estVelFromGps_old=estVelFromGps;
 	gps_old=gps_rot;
/*
 	static int cnt=0;
 	if(((cnt++)%50)==0){
 		cout<<estVelFromGps.transpose()<<endl;
	}*/
}
