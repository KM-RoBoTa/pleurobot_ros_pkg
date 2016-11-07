#include "pleurobot_ros_pkg/controller.h"

using namespace std;
using namespace Eigen;



/* Modifies trajectories to correct posture (roll and pitch angles) */
void
Controller :: postureControl()
{

    // if we have a raw data and not direct attitude information
    //Kalman();
    static double dx[4]={0}, dy[4]={0}, dx_old[4]={0}, dy_old[4]={0};


    for(int i=0; i<4; i++){
        if(legs_stance(i) || 1){
            dx[i]=pt1(xGain*attData[0], dx[i], Troll_posture, dt);          //roll
            dy[i]=pt1(yGain*attData[1], dy[i], Tpitch_posture, dt);         //pitch
        }
        else{
            dx[i]=pt1(0, dx[i], Troll_posture, dt);
            dy[i]=pt1(0, dy[i], Tpitch_posture, dt);
        }
    }

/*
    for(int i=0; i<3; i+=2){
        trPointsFL(2,i) = trPointsFL(2,i) + dx[0] - dy[0]   - dx_old[0] + dy_old[0]; 
        trPointsFR(2,i) = trPointsFR(2,i) - dx[1] - dy[1]   + dx_old[1] + dy_old[1]; 
        trPointsHL(2,i) = trPointsHL(2,i) + dx[2] + dy[2]   - dx_old[2] - dy_old[2]; 
        trPointsHR(2,i) = trPointsHR(2,i) - dx[3] + dy[3]   + dx_old[3] - dy_old[3]; 
    }
*/
    for(int i=0; i<3; i+=2){
        pFL(2) = pFL(2) + dx[0] - dy[0]; 
        pFR(2) = pFR(2) - dx[1] - dy[1]; 
        pHL(2) = pHL(2) + dx[2] + dy[2]; 
        pHR(2) = pHR(2) - dx[3] + dy[3]; 
    }
    //cout<<pFR(2)<<"\t"<<dy[1]<<"\t"<<attData[1]<<endl;

    for(int i=0; i<4; i++){
        dx_old[i]=dx[i]*0;
        dy_old[i]=dy[i]*0;
    }
    
       // cout<<attData[0]*180/pi<<"\t"<<attData[1]*180/pi<<"\t"<<attData[2]*180/pi<<endl;

    // inclination estimation



}

/* Leg extension reflex */
void
Controller :: legExtension()
{
    static MatrixXd feet_force_old=MatrixXd::Zero(3, 4);
    static double extt[4]={0, 0, 0, 0};
    static double ext_spikeZ[4];

    // filter force readings
    feet_force_old=pt1_vec(feet_force, feet_force_old, forceFilterExt, dt);


    // reflex activation logic
    for(int i=0; i<4; i++){
        if((sqrt(feet_force_old(2,i)*feet_force_old(2,i)+feet_force_old(1,i)*feet_force_old(1,i)+feet_force_old(0,i)*feet_force_old(0,i))<extRefForceLim) && (legs_stance(i)==1) && extt[i]>(extRefTimeout/100.0/freq_walk)){
            ext_reflex_active[i]=1;
        }
        else
            ext_reflex_active[i]=0;

        // stance phase time
        if(legs_stance(i)==0){
            extt[i]=0;
        }
        else
            extt[i]+=dt;
    }

    // calculate extension
    for(int i=0; i<4; i++){
        if(ext_reflex_active[i])
            ext_spikeZ[i]=pt1(extRefSpike, ext_spikeZ[i], extRefOnFilter/freq_walk, dt);
        else if(legs_stance(i)==1)
            ext_spikeZ[i]=pt1(0, ext_spikeZ[i], extRefOffFilter/freq_walk, dt);
        else
            ext_spikeZ[i]=pt1(0, ext_spikeZ[i], extRefOffFilter/freq_walk/100, dt);
        // reset extension
        if(extt[i]==0)
            ext_spikeZ[i]=0;
    }

    // add extension to the foot reference
    pFL(2)+=ext_spikeZ[0];
    pFR(2)+=ext_spikeZ[1];
    pHL(2)+=ext_spikeZ[2];
    pHR(2)+=ext_spikeZ[3];

}

/* Stumble reflex */
void
Controller :: stumbleReflex()
{
    static MatrixXd feet_force_old=MatrixXd::Zero(3, 4);
    static double stut[4]={0, 0, 0, 0};
    static double stu_spikeX[4];
    static double stu_spikeZ[4];

    feet_force_old=pt1_vec(feet_force, feet_force_old, forceFilterStu, dt);



    // reflex activation logic
    for(int i=0; i<4; i++){
        if((feet_force_old(0,i)<stuRefForceLimX) && (legs_stance(i)==0) && (feet_force_old(2,i)<stuRefForceLimZ) && stut[i]>(stuRefTimeout/100.0/freq_walk)){
            stu_reflex_active[i]=1;
        }
        else
            stu_reflex_active[i]=0;

        // swing phase time
        if(legs_stance(i)==0){
            stut[i]+=dt;
        }
        else
            stut[i]=0;
    }
    
    // calculate extension
    for(int i=0; i<4; i++){
        if(stu_reflex_active[i]){
            stu_spikeX[i]=pt1(stuRefDx, stu_spikeX[i], stuRefOnFilter/freq_walk, dt);
            stu_spikeZ[i]=pt1(stuRefDz, stu_spikeZ[i], stuRefOnFilter/freq_walk, dt);
        }
        else{
            stu_spikeX[i]=pt1(0, stu_spikeX[i], stuRefOffFilter/freq_walk, dt);
            stu_spikeZ[i]=pt1(0, stu_spikeZ[i], stuRefOffFilter/freq_walk, dt);
        }

        // reset extension
        if(stut[i]==0){
            stu_spikeX[i]=0;
            stu_spikeZ[i]=0;
        }
    }



    pFL(0)+=stu_spikeX[0];
    pFR(0)+=stu_spikeX[1];
    pHL(0)+=stu_spikeX[2];
    pHR(0)+=stu_spikeX[3];

    pFL(2)+=stu_spikeZ[0];
    pFR(2)+=stu_spikeZ[1];
    pHL(2)+=stu_spikeZ[2];
    pHR(2)+=stu_spikeZ[3];



}



/* Kalman filter for posture estimation from Gryro and Acc data */
void
Controller :: Kalman()
{

    static MatrixXd xp=MatrixXd::Zero(6, 1), xc=MatrixXd::Zero(6, 1);
    MatrixXd A(6,6), B(6,3), H(3,6);
    MatrixXd tmp(3,3);
    static MatrixXd Pc=MatrixXd::Identity(6,6);
    static MatrixXd Pp=MatrixXd::Identity(6,6);
    static MatrixXd K=MatrixXd::Identity(6,3);
    MatrixXd Q=MatrixXd::Identity(6,6)*1;
    MatrixXd R=MatrixXd::Identity(3,3)*1000;
    Vector3d u, z;
    double ax, ay, az;

    A<< 1, 0, 0, -dt, 0, 0,
        0, 1, 0, 0, -dt, 0,
        0, 0, 1, 0, 0, -dt,
        0, 0, 0, 1, 0, 0,
        0, 0, 0, 0, 1, 0,
        0, 0, 0, 0, 0, 1;
    B<< dt, 0, 0,
        0, dt, 0,
        0, 0, dt,
        0, 0, 0,
        0, 0, 0,
        0, 0, 0;
    H<< 1, 0, 0, 0, 0, 0,
        0, 1, 0, 0, 0, 0,
        0, 0, 1, 0, 0, 0;



    u(0)=gyroData[0];
    u(1)=gyroData[1];
    u(2)=gyroData[2];
    ax=accData[0];
    ay=accData[1];
    az=accData[2];
    z(0)=atan2(ay, az);
    z(1)=atan2(ax, az);
    z(2)=atan2(ax, ay);


    // prediction
    xp=A*xp+B*u;
    Pp=A*Pc*A.transpose() + Q;

    // correction
    tmp=(H*Pp*H.transpose() + R);
    K=Pp*H.transpose()*tmp.inverse();
    xc=xp+K*(z-H*xp);
    Pc=(MatrixXd::Identity(6,6) - K*H)*Pp;

    posture=xc.block<3,1>(0,0);
    /*
    static ofstream loga("data/loga.txt");
    loga<<posture.transpose()<<"\t";
    loga<<u.transpose()<<"\t";
    loga<<ax<<"\t"<<ay<<"\t"<<az<<endl;
    */
}


/* Emergency reflex */
void
Controller :: emergencyReflex()
{
    static double t_er=0;
    static bool is_on=false;
    static MatrixXd spine_gains=MatrixXd::Constant(11,1,1), legs_height=MatrixXd::Zero(4,1);
    static bool er_turn_on;
    static double forward_vel, camera_height;
    er_turn_on=false;
    

    
    
    
    ////////////////////////// reflex activation logic ////////////////////////////
    
    // OPTICAL FLOW
    // estimate expected forward velocity
    /*
    forward_vel=0.5*(trF0(0,0)-trF0(0,1))*freq_walk/Duty(0) + 0.5*(trH0(0,0)-trH0(0,1))*freq_walk/Duty(1);
    
    if(joy_l2>0){
	   er_turn_on=true;
    }
    
    // estimate height
    camera_height=0.5*max(-feet_position_local(2,0),-feet_position_local(2,1))+0.5*max(-feet_position_local(2,2),-feet_position_local(2,3));
    camera_height*=tan(FOV_V/2.);
    
    
    fwdvel<<forward_vel<<"\t"<<camera_height<<endl;
    
    if((filtFlowY/25.)<(er_flow_limit/100.0*forward_vel)&&!er_turn_on){
	   er_turn_on=true;
    }
    */


    // FRICTION CONE & slipping detection
    static double feet_slipping_cumulative[4]={0}, feet_slipping_cumulative_old[4]={0}, feet_slipping_sum;
    feet_slipping_sum=0;
    if(state==WALKING){
        for(int i=0; i<4; i++){
            feet_slipping_cumulative[i]=pt1(feet_is_slipping(i), feet_slipping_cumulative_old[i], er_slipping_Tfilt/freq_walk, dt);
            feet_slipping_cumulative_old[i]=feet_slipping_cumulative[i];
            feet_slipping_sum+=feet_slipping_cumulative[i];
        }
    }

    if(feet_slipping_sum>er_slipping_threshold){
        er_turn_on=true;
    }



    
    /////////////////////////////////////////////////////////////////////////////
    if(er_turn_on){
	   is_on=true;
    }
    else if(!er_turn_on&&!is_on){
	   return;
    }
    
    
    
    //////////////////////// start reflex /////////////////////////////
    if(t_er<er_timeout){
    	spine_gains=pt1_vec(spine_gains0, spine_gains, erRefOnFilt, dt);
    	legs_height=pt1_vec(legs_height0, legs_height, erRefOnFilt, dt);
    }
    else{
    	spine_gains=pt1_vec(MatrixXd::Constant(11,1,1), spine_gains, erRefOffFilt, dt);
    	legs_height=pt1_vec(MatrixXd::Zero(4,1), legs_height, erRefOffFilt, dt);
    }
    
    qs=qs.cwiseProduct(spine_gains);
    pFL(2)-=legs_height(0);
    pFR(2)-=legs_height(1);
    pHL(2)-=legs_height(2);
    pHR(2)-=legs_height(3);
    
    
    
    
    
    
    

    
    t_er+=dt;
    
    if(t_er>(er_duration+2*erRefOffFilt)){
    	t_er=0;
    	
    	is_on=false;
    	spine_gains=MatrixXd::Constant(11,1,1);
    	legs_height=MatrixXd::Zero(4,1);
    	return;
    }
    

    return;
}
    
    
    
    
    
    
    
    
    
    
    
