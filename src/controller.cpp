#include "pleurobot_ros_pkg/controller.h"
#include <iostream>
#include <cmath>
#include <ctime>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string.h>
#include <stdlib.h>

using namespace std;
using namespace Eigen;

extern int IS_SIMULATION, IS_PLEUROBOT, USE_JOYSTICK, JOYSTICK_TYPE, SWIM, SPINE_COMPENSATION_WITH_FEEDBACK, IS_SNAKE;
extern int USE_REFLEXES, USE_IMU, AUTO_RUN, AUTO_RUN_STATE;

/* Controller constructor */
Controller :: Controller(double time_step) : HJs(12), HJs_g(12), HJfl(5), HJfr(5), HJhl(5), HJhr(5), HJfl_g(5), HJfr_g(5), HJhl_g(5), HJhr_g(5), legJacob(4)
{
    setTimeStep(time_step);
    #ifndef OPTIMIZATION
        camImg=cv::Mat(240, 320, CV_8UC3);
        imgArray=(unsigned char*)malloc(320*240*3);
    #endif

    rotMatFgird=Matrix3d::Identity();
    //############################ PARAMETERS ########################################################
    getParameters();

    cout<<"I successfully read the parameters"<<endl;
    state=INITIAL;

    t=0;

    js.load(JOYSTICK_DEVNAME);

    if(js.is_ready()) {
        printf("Joystick Ready\n");
        updateState();
    }
    headq=0;
    gamma=0;
    //########## STANDING POSITION #################
    posFL<< 0, trPointsFL0(1,0), trPointsFL0(2, 0);
    posFL<< 0, trPointsFR0(1,0), trPointsFR0(2, 0);
    posHL<< 0, trPointsHL0(1,0), trPointsHL0(2, 0);
    posHL<< 0, trPointsHR0(1,0), trPointsHR0(2, 0);

    pFL0=posFL;
    pFR0=posFR;
    pHL0=posHL;
    pHR0=posHR;
    force_filt << 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
    qs << 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
    //############################ LEGS ########################################################

    DH1 <<  0,      0.06185,    0,      pi/2,
    0,      0,          0,      pi/2,
    0,      0.143,      0,      pi/2,
    0,      0,          0.1145, 0;

    DH2 <<  0,      0.0856,     0,      -pi/2,
    0,      0,          0,      pi/2,
    0,      -0.118,     0,      -pi/2,
    0,      0,          0.1145, 0;


    // initial conditions
    qFL=q0FL;
    qFR=q0FR;
    qHL=q0HL;
    qHR=q0HR;
    init_angles<<0,0,0,0,0,0,0,0,0,0,0,
            0,0,0,0,
            0,0,0,0,
            0,0,0,0,
            0,0,0,0;
    init_angles*=0;

    rFL << 0, trPointsFL(1,0), trPointsFL(2,0);
    rFR << 0, trPointsFR(1,0), trPointsFR(2,0);
    rHL << 0, trPointsHL(1,0), trPointsHL(2,0);
    rHR << 0, trPointsHR(1,0), trPointsHR(2,0);

    getObstacles();

    joint_angles.setZero();
    feet_force<<0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
    //#############################Joystick dynamics########################################
    spd=0;

    nfl=0; nfr=0; nhl=0; nhr=0;
    fxfl=0; fxfr=0; fxhl=0; fxhr=0;
    isTalking=0;
    posing_xFH=0; posing_yF=0; posing_zF=0; posing_head=0;
    crab_rotAng=0;

    //############################# REFLEXES ########################################
    for(int i=0; i<4; i++){
        stu_reflex_active[i]=0;
        ext_reflex_active[i]=0;
    }

    // image proc stuff
    filtFlowX=0;
    filtFlowY=0;

    aquastep_pitch_offset=0;

    //############################# LEG PHASES ########################################
    for(int i=0; i<4; i++){
        legPhase(i)=phShifts(i)<0?phShifts(i)+1:phShifts(i);
    }

    girdleTraj << 0,0,0,0,0,0, 0,0,0,0,0,0;
    
    joint_sub_ = nh_.subscribe<sensor_msgs::JointState>("/pleurobot/joint_states", 1, &Controller::GetROSJointStatus, this);
}

/* Sets time step */
void
Controller :: setTimeStep(double time_step)
{
    dt=time_step;
    return;
}

/* Updates angles for all joints - MAIN FUNCTION */
bool
Controller :: runStep()
{


    if(!updateState()){
        return false;
    }

    joint_angles=angles;
    //joint_angles=fbck_position;  // This line is for the robot position feedback(joint angles)
    forwardKinematics();
    //############################ TIME #########################################################
    if(state==WALKING|| state==SWIMMING || STANDING)
        t+=dt;

    //############################ SPINE ########################################################
    runSpine();

    //############################ TRAJECTORIES ########################################################
    //simpleTrajectoryFollower();
    //gpsCheatedTrajectoryFollower();
    
    walkingTrajectories();

    //headingController(0);
    //steeringSpineless(0.1);

    //############################ WALKING ############################
    if(state==WALKING){
        if(USE_REFLEXES && t>2){
            legExtension();
            stumbleReflex();
            emergencyReflex();
        }
        
        if(USE_IMU){
            postureControl();
        }
    }
    //postureControl();
    //############################ STANDING ############################
 /*   else if(state==STANDING){
        postureControl();
        rFL=posFL;
        rFL(2)=trPointsFL(2, 0);

        rFR=posFR;
        rFR(2)=trPointsFR(2, 0);

        rHL=posHL;
        rHL(2)=trPointsHL(2, 0);

        rHR=posHR;
        rHR(2)=trPointsHR(2, 0);

    }*/
    //############################ POSING ############################
    if(state == SWIMMING){
        //swimFun();
        swimFunCPG();
        return true;
    }

    if(state==WALKING || state==STANDING || state==POSING){
        inverseKinematicsController(); 
        angles.block<11,1>(0,0)=qs;
        angles.block<4,1>(11,0)=qFL;
        angles.block<4,1>(15,0)=qFR;
        angles.block<4,1>(19,0)=qHL;
        angles.block<4,1>(23,0)=qHR;

        joint_angles=angles;
    }
    getLegJacobians();
    //VmcController();
    if(IS_SNAKE){
       // SnakeController();
    }

    torquePID();
    VMC_bodyPosture();

    return true;
}

/* Reads joystick */
void
Controller :: readJoystick()
{
    js.update();
    switch(JOYSTICK_TYPE){
            case 1:
                // PS1 CONTROLLER
                joy_bU=js.buttons[0];
                joy_bR=js.buttons[1];
                joy_bD=js.buttons[2];
                joy_bL=js.buttons[3];
                joy_l2=js.buttons[4];
                joy_r2=js.buttons[5];
                joy_l1=js.buttons[6];
                joy_r1=js.buttons[7];
                joy_sel=js.buttons[8];
                joy_start=js.buttons[9];
                joy_l3=js.buttons[10];
                joy_r3=js.buttons[11];

                joy_x1=js.axes[0];
                joy_y1=-js.axes[1];
                joy_x2=js.axes[3];
                joy_y2=-js.axes[2];
                joy_x3=js.axes[4];
                joy_y3=-js.axes[5];
            break;
            case 2:
                // PS3 CONTROLLER
                joy_sel=js.buttons[0];
                joy_l3=js.buttons[1];
                joy_r3=js.buttons[2];
                joy_start=js.buttons[3];
                joy_l2=js.buttons[8];
                joy_r2=js.buttons[9];
                joy_l1=js.buttons[10];
                joy_r1=js.buttons[11];
                joy_bU=js.buttons[12];
                joy_bR=js.buttons[13];
                joy_bD=js.buttons[14];
                joy_bL=js.buttons[15];

                joy_x1=js.axes[0];
                joy_y1=-js.axes[1];
                joy_x2=js.axes[2];
                joy_y2=-js.axes[3];
                joy_x3=js.axes[4];
                joy_y3=-js.axes[5];
                break;
            case 3:
                // LOGITECH CONTROLLER
                joy_bL=js.buttons[0];
                joy_bD=js.buttons[1];
                joy_bR=js.buttons[2];
                joy_bU=js.buttons[3];
                joy_l2=js.buttons[6];
                joy_r2=js.buttons[7];
                joy_l1=js.buttons[4];
                joy_r1=js.buttons[5];
                joy_sel=js.buttons[8];
                joy_start=js.buttons[9];
                joy_l3=js.buttons[10];
                joy_r3=js.buttons[11];

                joy_x1=js.axes[0];
                joy_y1=-js.axes[1];
                joy_x2=js.axes[2];
                joy_y2=-js.axes[3];
                joy_x3=js.axes[4];
                joy_y3=-js.axes[5];
            break;
        }

        joy_lsr=sqrt(joy_x1*joy_x1+joy_y1*joy_y1);
        joy_lsr=joy_lsr>1?1:joy_lsr;
        if(joy_lsr<0.1){
            joy_lsr=0;
        }
        if(joy_lsr>0.2){
            joy_lsphi=atan2(-joy_x1, joy_y1);
        }
        else{
            joy_lsphi=0;
        }
    
        joy_lsr=joy_lsr>0.2?1:0;
    
        
        joy_rsr=sqrt(joy_x2*joy_x2+joy_y2*joy_y2);
        joy_rsr=joy_rsr>1?1:joy_rsr;
        if(joy_rsr<0.1){
            joy_rsr=0;
        }
        if(joy_rsr>0.2){
            joy_rsphi=atan2(joy_y2, joy_x2)-pi/2;
        }

}

/* Reads joystick inputs and modifies trajectories */
bool
Controller :: updateState()
{
    if(USE_JOYSTICK){
        readJoystick();
    }
    else{
        joy_bU=0;
        joy_bR=0;
        joy_bD=0;
        joy_bL=0;
        joy_l2=0;
        joy_r2=0;
        joy_l1=0;
        joy_r1=0;
        joy_sel=0;
        joy_start=0;
        joy_l3=0;
        joy_r3=0;

        joy_x1=0;
        joy_y1=0;
        joy_x2=0;
        joy_y2=0;
        joy_x3=0;
        joy_y3=0;

        joy_lsr=1;
        joy_rsr=1;
        joy_lsphi=0;
        joy_rsphi=0;
        pathPlanner();
    }

    phases=phases0;
    //============================== EMERGENCY STOP ==============================
    if(joy_start == 1){
        return false;
    }

    //=============================== POSING ===================================
    if(joy_sel==0 && joy_bU==1 && state != POSING){
        state=POSING;
        offsets << 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
        posing_head=0;
        T_trans=T_trans0;
    }

    //=============================== ANIMAL_WALKING ===================================
    else if(joy_sel==0 && joy_bL==1 && state != ANIMAL_WALKING){
        state=ANIMAL_WALKING;
        T_trans=T_trans0;
    }

    //=============================== ANIMAL_SWIMMING ===================================
    else if(joy_sel==1 && joy_bL==1 && state != ANIMAL_SWIMMING){
        state=ANIMAL_SWIMMING;
        freq_swim=0;
        T_trans=T_trans0;
    }
    
    //=============================== ANIMAL AQUATIC STEPPING ===========================
    else if(joy_sel==1 && joy_bD==1 && state != ANIMAL_AQUASTEP){
        state=ANIMAL_AQUASTEP;
        freq_aquastep=0;
        T_trans=T_trans0;
    }
    

    //=============================== STANDING ===================================

    else if(joy_sel==0 && joy_bR==1 && !(state==WALKING || state == STANDING)){
        state=STANDING;
        T_trans=T_trans0;
    }

    //=============================== WALKING ===================================
    if(((state == STANDING && ((joy_lsr>0.2))) || !USE_JOYSTICK)){
        state=WALKING;
    }

    //=============================== STANDING ===================================
    else if(state == WALKING && (!(joy_lsr>0.2)) && (legs_stance(0) && legs_stance(1) && legs_stance(2) && legs_stance(3))){
        state=STANDING;
    }

    //=============================== INITIAL ===================================
    if(joy_sel==0 && joy_bD==1 && !(state==INITIAL)){
        state=INITIAL;
        T_trans=T_trans0;
    }

    //=============================== SWIMMING ===================================
    if(joy_sel==1 && joy_bR==1 && state!=SWIMMING){
        state=SWIMMING;
        T_trans=T_trans0;
        freq_swim=0;
        cpg_offset=0;
    }

    //=============================== NONPERIODIC ===================================
    if(joy_sel==1 && joy_bU==1 && state!=NONPERIODIC){
        state=NONPERIODIC;
        T_trans=T_trans0;
    }


    //=============================== DECAY TRANSITION FILTERING CONSTANT ===================================
    T_trans=pt1(0, T_trans, 1, dt);


    //=============================== JOYSTICK INTERACTION ===================================
    if(AUTO_RUN){
        joy_lsr=1;
        state=AUTO_RUN_STATE;
    }
    joystickManipulation();

    return true;
}

/* Joystick manipulation of different parameters/variables */
void
Controller :: joystickManipulation()
{
    //=============================== TALKING ===================================
/*  if(!IS_SIMULATION){
        static double talkingTime=0;
        static int joy_bD_old=0;
        if(!isTalking && joy_bD && !joy_bD_old){
            system("apps/speak.sh &");
            isTalking=1;
            joy_bD_old=1;
            talkingTime=0;
        }
        else if(isTalking && joy_bD && talkingTime>2){
            isTalking=0;
            talkingTime=0;
            system("apps/stop_speaking.sh &");
        }
        if(talkingTime>70){
            talkingTime=0;
        }
        if(isTalking){
            talkingTime+=dt;
        }
        if(joy_bD==0){
            joy_bD_old=0;
        }
    }*/

    //=============================== WALKING ===================================
    if(state == WALKING){

        // speed - left stick
        freq_walk=pt1(joy_lsr*joy_walk_max_freq, freq_walk, joy_walk_speed_change_filt, dt);
        if(freq_walk<joy_walk_min_freq){
            freq_walk=joy_walk_min_freq;
        }
        if(freq_walk>3.1){
            freq_walk=3.1;
        }
    


        // crab walking
        if(abs(gamma)>disable_crab_walk_lim){
            crab_rotAng=pt1(0, crab_rotAng, 0.2, dt);
        }
        else{
            crab_rotAng=pt1(joy_lsphi, crab_rotAng, 0.2, dt);
        }
        if((crab_rotAng<0 && joy_lsphi>0 && joy_lsr>stick_r_lim) || (crab_rotAng>0 && joy_lsphi<0 && joy_lsr>stick_r_lim)){
            crab_rotAng=joy_lsphi;
        }

        trPointsFL.block<2,1>(0,0)=rotEllipse(trPointsFL0(0,0), ellipse_small_axis, crab_rotAng, trPointsFL0(1, 0));
        trPointsFL.block<2,1>(0,1)=rotEllipse(trPointsFL0(0,1), ellipse_small_axis, crab_rotAng, trPointsFL0(1, 1));
        trPointsFL.block<2,1>(0,2)=rotEllipse(trPointsFL0(0,2), ellipse_small_axis, crab_rotAng, trPointsFL0(1, 2));

        trPointsFR.block<2,1>(0,0)=rotEllipse(trPointsFR0(0,0), ellipse_small_axis, crab_rotAng, trPointsFR0(1, 0));
        trPointsFR.block<2,1>(0,1)=rotEllipse(trPointsFR0(0,1), ellipse_small_axis, crab_rotAng, trPointsFR0(1, 1));
        trPointsFR.block<2,1>(0,2)=rotEllipse(trPointsFR0(0,2), ellipse_small_axis, crab_rotAng, trPointsFR0(1, 2));

        trPointsHL.block<2,1>(0,0)=rotEllipse(trPointsHL0(0,0), ellipse_small_axis, crab_rotAng, trPointsHL0(1, 0));
        trPointsHL.block<2,1>(0,1)=rotEllipse(trPointsHL0(0,1), ellipse_small_axis, crab_rotAng, trPointsHL0(1, 1));
        trPointsHL.block<2,1>(0,2)=rotEllipse(trPointsHL0(0,2), ellipse_small_axis, crab_rotAng, trPointsHL0(1, 2));

        trPointsHR.block<2,1>(0,0)=rotEllipse(trPointsHR0(0,0), ellipse_small_axis, crab_rotAng, trPointsHR0(1, 0));
        trPointsHR.block<2,1>(0,1)=rotEllipse(trPointsHR0(0,1), ellipse_small_axis, crab_rotAng, trPointsHR0(1, 1));
        trPointsHR.block<2,1>(0,2)=rotEllipse(trPointsHR0(0,2), ellipse_small_axis, crab_rotAng, trPointsHR0(1, 2));


        // side walking amps damping
        static double ampsgain=1;
        double ampsgain_u;
        ampsgain_u=1-2/pi*abs(crab_rotAng);
        ampsgain_u=ampsgain_u>1?1:ampsgain_u;
        ampsgain_u=ampsgain_u<-1?-1:ampsgain_u;
        ampsgain=pt1(ampsgain_u, ampsgain, 1, dt);

        // steer
        gamma=pt1((joy_x2)*pi/2, gamma, 1, dt);

        offsets=MatrixXd::Ones(11,1)*gamma/5;
        amps=ampsgain*exp(-2*abs(gamma))*amps0;
        offsets.block<5, 1>(6, 0) = offsets.block<5, 1>(6, 0)*0;
        //offsets(0)=offsets(0)*0;


    }

    //=============================== POSING ===================================
    else if(state==POSING){
        freq_walk=0;
        double tmp_tf=exp(-dt/Tf1);
        posing_xFH=tmp_tf*posing_xFH + (1-tmp_tf)*joy_y1*posing_joy_y1_rate;
        posing_yF=tmp_tf*posing_yF + (1-tmp_tf)*joy_x1*posing_joy_x1_rate;
        posing_zF=tmp_tf*posing_zF + (1-tmp_tf)*joy_y2*posing_joy_y2_rate;


        rFL<< 0-posing_xFH, trPointsFL0(1, 0)+posing_yF, trPointsFL0(2, 0)+posing_zF;
        rFR<< 0-posing_xFH, trPointsFR0(1, 0)+posing_yF, trPointsFR0(2, 0)+posing_zF;
        rHL<< 0-posing_xFH, trPointsHL0(1, 0), trPointsHL0(2, 0);
        rHR<< 0-posing_xFH, -trPointsHR0(1, 0), trPointsHR0(2, 0);


        double tmp5;
        tmp5=atan2(5*posing_yF, 0.5136);

        for(int i=0;i<5;i++){
            qsr(i)=-tmp5/5;
        }
        for(int i=5;i<11;i++){
            qsr(i)=0;
        }
        posing_head=posing_head-posing_head_rate*( joy_r2 - joy_l2);
        posing_head=posing_head>posing_head_limit ? posing_head_limit : posing_head;
        posing_head=posing_head<-posing_head_limit ? -posing_head_limit : posing_head;
        qsr(0)=posing_head;
    }

    //=============================== INITIAL ===================================
    else if(state==INITIAL){
        angles=init_angles;
    }

    //=============================== SWIMMING ===================================
    else if(state==SWIMMING){
        freq_swim=pt1(joy_lsr*joy_swim_max_freq, freq_swim, joy_swim_speed_change_filt, dt);
        if(joy_lsr<stick_r_lim){
            freq_swim=0;
            R_cpg=MatrixXd::Zero(11,1);
        }
        else{
            R_cpg=R_cpg0*R_cpg_gain;
        }

        cpg_offset=pt1(joy_x1*joy_swim_max_offset, cpg_offset, joy_swim_speed_change_filt, dt);

        if(abs(joy_lsphi)<joy_swim_turning_dead_zone){
            cpg_offset=0;
        }

        ph_lagfwd0+= (joy_r1-joy_l1)*phase_lag_change_rate*dt;
        ph_lagfwd0=ph_lagfwd0>phase_lag_max?phase_lag_max:ph_lagfwd0;
        ph_lagfwd0=ph_lagfwd0<phase_lag_min?phase_lag_min:ph_lagfwd0;

        if(abs(joy_lsphi)>pi/2){
            ph_lagfwd=-ph_lagfwd0/10*2*pi/100;
        }
        else{
            ph_lagfwd=ph_lagfwd0/10*2*pi/100;
        }
    }

    //=============================== ANIMAL_WALKING ===================================
    else if(state==ANIMAL_WALKING){


        freq_walk=pt1(joy_lsr*joy_walk_max_freq, freq_walk, joy_walk_speed_change_filt, dt);
        if(freq_walk<joy_walk_min_freq){
            freq_walk=joy_walk_min_freq;
        }

        if(joy_lsr<stick_r_lim){
            freq_walk=0.00001;
        }
        else if(freq_walk>3.1){
            freq_walk=3.1;
        }
        gamma=pt1((-joy_x2)*pi/2, gamma, 1, dt);
        
       
    }

    //=============================== ANIMAL_SWIMMING ===================================
    else if(state==ANIMAL_SWIMMING){
        freq_swim=pt1(joy_lsr*joy_swim_max_freq, freq_swim, joy_swim_speed_change_filt, dt);
        if(joy_lsr<stick_r_lim){
            freq_swim=0;
        }
        gamma=pt1(-joy_x1*joy_swim_max_offset, gamma, joy_swim_speed_change_filt, dt);

        if(abs(joy_lsphi)<joy_swim_turning_dead_zone){
            gamma=0;
        }
    }
    
    //=============================== ANIMAL_AQUASTEP ===================================
    else if(state==ANIMAL_AQUASTEP){
        freq_aquastep=pt1(joy_lsr*joy_swim_max_freq, freq_aquastep, joy_swim_speed_change_filt, dt);
    freq_aquastep=joy_swim_max_freq;
        if(joy_lsr<stick_r_lim){
            freq_aquastep=0;
        }
        gamma=pt1(-joy_x1*joy_swim_max_offset, gamma, joy_swim_speed_change_filt, dt);

        if(abs(joy_lsphi)<joy_swim_turning_dead_zone){
            gamma=0;
        }


        aquastep_pitch_offset+=(joy_r1-joy_l1)/2000.;


    }




    else if(state==STANDING){
        freq_walk=0;
    }

}

/* Writes angles calculated by runStep function into a table - interface with Pleurobot class */
void
Controller :: getAngles(double table[N_SERVOS])
{
/*
        static MatrixXd angles2(27,1);
        static MatrixXd angles2_old=MatrixXd::Zero(27, 1);
        static MatrixXd angles_old=init_angles;
*/

        static MatrixXd angles_old=init_angles;
        static MatrixXd angles2=transformation_Ikin_Robot(init_angles,1,1);
        static MatrixXd angles2_old=transformation_Ikin_Robot(init_angles,1,1);

        if(state==ANIMAL_WALKING){
            runAnimalDataWalking();
        }
        if(state==ANIMAL_SWIMMING){
            runAnimalDataSwimming();
        }
        if(state==ANIMAL_AQUASTEP){
        	runAnimalDataAquastepping();
   		}

        angles=pt1_vec(angles, angles_old, T_trans, dt);
        angles_old=angles;


        if(IS_SIMULATION){
            angles2=transformation_Ikin_Webots(angles,1, 1);
        }

        if(!IS_SIMULATION){
            angles2=transformation_Ikin_Robot(angles,1,1);
        }


        angles2=pt1_vec(angles2, angles2_old, Tfilt_angles, dt);
        angles2_old=angles2;

        for(int i=0; i<N_SERVOS; i++){
            table[i]=angles2(i);
        }

        // CONSTRAINTS
        for(int i=0; i<11; i++){
            table[i]=table[i]>constrS?constrS:table[i];
            table[i]=table[i]<-constrS?-constrS:table[i];
        }
        if(!IS_SIMULATION){
            /*table[26] = table[26] + 90 * pi / 180.0;
            table[22] = table[22] - 90 * pi / 180.0;
            for(int i=0; i<4; i++){
                table[11+i]=table[11+i]<constrF(0, i) ? constrF(0, i) : table[11+i];
                table[11+i]=table[11+i]>constrF(1, i) ? constrF(1, i) : table[11+i];
                table[15+i]=table[15+i]>-constrF(0, i) ? -constrF(0, i) : table[15+i];
                table[15+i]=table[15+i]<-constrF(1, i) ? -constrF(1, i) : table[15+i];
                table[19+i]=table[19+i]<constrH(0, i) ? constrH(0, i) : table[19+i];
                table[19+i]=table[19+i]>constrH(1, i) ? constrH(1, i) : table[19+i];
                table[23+i]=table[23+i]>-constrH(0, i) ? -constrH(0, i) : table[23+i];
                table[23+i]=table[23+i]<-constrH(1, i) ? -constrH(1, i) : table[23+i];
            }
            table[26] = table[26] - 90 * pi / 180.0;
            table[22] = table[22] + 90 * pi / 180.0;*/
        }

    return;
}

/* Writes torques calculated by runStep function into a table - interface with Pleurobot class */

void
Controller :: getTorques(double table[N_SERVOS])
{
        static MatrixXd torques2(27,1);
        //static MatrixXd torques(27,1);

        static MatrixXd torques2_old=MatrixXd::Zero(27, 1);
        static MatrixXd torques_old=MatrixXd::Zero(27, 1);


        if(IS_SIMULATION){
            torques2=transformation_Ikin_Webots(torques, 1, 0);
        }

        if(!IS_SIMULATION){
            torques2=transformation_Ikin_Robot(torques, 1, 0);
        }


        for(int i=0; i<N_SERVOS; i++){
            table[i]=torques2(i);
        }


        // CONSTRAINTS
    if(!IS_SIMULATION){
        for(int i=0; i<27; i++){
            table[i]=table[i]>7?7:table[i];
            table[i]=table[i]<-7?-7:table[i];
        }
    }

    return;
}

/* Actuates just one leg */
void
Controller :: oneLegTesting()
{
    angles.setZero();
    static double xRef=0, yRef=0.18, zRef=0.14;
    double tmp_tf=exp(-dt/Tf1);
    double scaling=0.001;
    Vector3d pRef;

    double x1, x2, y1, y2, x3, y3;
    int l1, l2, l3, r1, r2, r3, sel, start, bD, bL, bR, bU;

    // PS1 CONTROLLER
    bU=js.buttons[0];
    bR=js.buttons[1];
    bD=js.buttons[2];
    bL=js.buttons[3];
    l2=js.buttons[4];
    r2=js.buttons[5];
    l1=js.buttons[6];
    r1=js.buttons[7];
    sel=js.buttons[8];
    start=js.buttons[9];
    l3=js.buttons[10];
    r3=js.buttons[11];

    x1=js.axes[0];
    y1=-js.axes[1];
    x2=js.axes[3];
    y2=-js.axes[2];
    x3=js.axes[4];
    y3=-js.axes[5];

    xRef=xRef + y1*scaling;
    yRef=yRef - x1*scaling;
    zRef=zRef - y2*scaling*3;

    pRef << xRef, yRef, zRef;
/*
    qFL=iKinDLS(DH1, pRef, q0FL, qFL, CinvF, MF, max_dist, 0);

    angles.block<4,1>(11,0)=qFL.transpose();

    angles=angles*180/pi;
    angles=angles.cwiseProduct(ang_signsM)+ang_shiftsM;

*/

    return;
}

/* Estimates forces acting to the feet from torque readings */
MatrixXd
Controller :: forceEstimation()
{


    MatrixXd ang_signsM23(27,1);
    ang_signsM23 << -1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
    1,-1,-1, 1,
    1,-1,-1, 1,
    1, 1, 1, 1,
    1, 1, 1, 1;

    MatrixXd ang_shiftsM23(27,1);
    ang_shiftsM23 << 0,0,0,0,0,0,0,0,0,0,0,
    -90,-90,0,0,
    -90,-90,0,0,
    -90,-90,0,0,
    -90,-90,0,0;

    MatrixXd ang_signs23(27,1);
    ang_signs23<< -1,1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
          -1, -1, 1, -1,
          1, 1, -1, 1,
          -1, -1, 1, -1,
          1, 1, -1, 1;


    MatrixXd ang_shifts23(27,1);
    ang_shifts23<< 0,0,0,0,0,0,0,0,0,0,0,
                0,0,90,0,
                0,0,90,0,
                0,0,0,90,
                0,0,0,90;

    VectorXd offset(27,1);
    offset << 0,0,0,0,0,0,0,0,0,0,0,
            0, 0, 90, 0,
            0, 0, -90, 0,
            0, 0, 0, 90,
            0, 0, 0, -90;
    VectorXd d_ang_signs(27,1);
    d_ang_signs << -1,
        1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
        -1, -1, 1, -1,
        1, 1, -1, 1,
        -1, -1, 1, -1,
        1, 1, -1, 1;



    static MatrixXd JFL(3,4), JFR(3,4), JHL(3,4), JHR(3,4), tmp3(3,3);


    if(IS_SIMULATION){
        fbck_position=fbck_position+ang_shifts23*pi/180;
        //fbck_position=fbck_position.cwiseProduct(ang_signs23);
        fbck_torques=fbck_torques.cwiseProduct(ang_signs23);

        fbck_position=fbck_position+ang_shiftsM23*pi/180;
        //fbck_position=fbck_position.cwiseProduct(ang_signsM23);
        fbck_torques=fbck_torques.cwiseProduct(ang_signsM23);
    }
    else{
      //  fbck_position=fbck_position-ang_shifts23*pi/180;
        fbck_position=fbck_position.cwiseProduct(d_ang_signs);
        fbck_torques=fbck_torques.cwiseProduct(d_ang_signs);
        fbck_position(26)+=pi/2;
        fbck_position(22)-=pi/2;

        fbck_position=fbck_position+ang_shiftsM23*pi/180;
        fbck_position=fbck_position.cwiseProduct(ang_signsM23);
        fbck_torques=fbck_torques.cwiseProduct(ang_signsM23);
    }

    JFL=Jacob(fbck_position.block<4,1>(11,0), 0);
    JFR=Jacob(fbck_position.block<4,1>(15,0), 0);
    JHL=Jacob(fbck_position.block<4,1>(19,0), 1);
    JHR=Jacob(fbck_position.block<4,1>(23,0), 1);

    tmp3=JFL*JFL.transpose();
    tmp3=tmp3.inverse()*JFL;
    force.block<3,1>(0,0)=tmp3*fbck_torques.block<4,1>(11,0);

    tmp3=JFR*JFR.transpose();
    tmp3=tmp3.inverse()*JFR;
    force.block<3,1>(3,0)=tmp3*fbck_torques.block<4,1>(15,0);

    tmp3=JHL*JHL.transpose();
    tmp3=tmp3.inverse()*JHL;
    force.block<3,1>(6,0)=tmp3*fbck_torques.block<4,1>(19,0);

    tmp3=JHR*JHR.transpose();
    tmp3=tmp3.inverse()*JHR;
    force.block<3,1>(9,0)=tmp3*fbck_torques.block<4,1>(23,0);


    //force_filt=pt1_vec(force, force_filt, forceFilter, dt);
    return force_filt;



}

/* Estimates foot contact for stance and swing (obstacle) phases and for different reflexes*/
Vector3d
Controller :: contactDetection()
{
    double limF, limH, forceFilter;
    Vector3d tmpForce;
    reCon << 0, 0, 0, 0;
    rsCon << 0, 0, 0, 0;
    vmCon << 0, 0, 0, 0;

    // leg extension reflex

    static MatrixXd force_filt1=MatrixXd::Zero(12, 1);
    limF=10;
    limH=10;
    force_filt1=pt1_vec(force, force_filt1, forceFilter, dt);

    if(force_filt1.block<3,1>(0,0).norm()>limF)
        reCon(0)=1;
    if(force_filt1.block<3,1>(3,0).norm()>limF)
        reCon(1)=1;
    if(force_filt1.block<3,1>(6,0).norm()>limH)
        reCon(2)=1;
    if(force_filt1.block<3,1>(9,0).norm()>limH)
        reCon(3)=1;


    // stumble reflex

    static MatrixXd force_filt2=MatrixXd::Zero(12, 1);
    limF=30;
    limH=50;
    force_filt2=pt1_vec(force, force_filt2, forceFilter, dt);

    if(force_filt2.block<3,1>(0,0).norm()>limF)
        rsCon(0)=1;
    if(force_filt2.block<3,1>(3,0).norm()>limF)
        rsCon(1)=1;
    if(force_filt2.block<3,1>(6,0).norm()>limH)
        rsCon(2)=1;
    if(force_filt2.block<3,1>(9,0).norm()>limH)
        rsCon(3)=1;





    // posture control
    //forceFilter=0.15;
    static double ffl3=0, ffr3=0, fhl3=0, fhr3=0;

    ffl3=pt1(force.block<3,1>(0,0).norm(), ffl3, forceFilter, dt);
    ffr3=pt1(force.block<3,1>(3,0).norm(), ffr3, forceFilter, dt);
    fhl3=pt1(force.block<3,1>(6,0).norm(), fhl3, forceFilter, dt);
    fhr3=pt1(force.block<3,1>(9,0).norm(), fhr3, forceFilter, dt);


}

/* Calculates joint angles for the spine */
void
Controller :: runSpine()
{
    static MatrixXd qs_cmp(11,1);
    static int k_anSP;
    static double t_spine=0;

    if(state==WALKING){
        t_spine+=dt*freq_walk;
        k_anSP=floor(fmod(t_spine, 1)*1000);
    }

    if((state==WALKING || state==STANDING) && (useAnSP<0)){
        for(int i=0; i<11; i++) {
            qsr(i)=amps(i)*sin(t_spine*2*pi+phases(i)); 
        }
        //spineFromGirdleOsc(dt);
        //spineFromGirdle(dt);
    }
    else if((state==WALKING || state==STANDING) && (useAnSP>=0)){
        for(int i=0; i<11; i++){
            qsr(i)=useAnSP*animalSpine(k_anSP, i);
        }
    }



    // find displacement of hind girdle
    for(int i=0; i<11; i++){
        offsets(i)=(girdleTraj(5,0)-girdleTraj(2,0))/5;
    }
    qs=qsr+offsets;
    qs(0)=qsr(0);


    if(SPINE_COMPENSATION_WITH_FEEDBACK==1){
        qs_cmp=fbck_position.block<11,1>(0,0);
    }
    else{
        qs_cmp=qs;
    }

    
    qs(0)=headq;
    if(!IS_PLEUROBOT){
        qs(0)=headq/2.;
        qs(1)=headq/2.;
    }


    // HIND GIRDLE SWAY

    hgirdsway=hgirdsway_amp*sin(2*pi*t_spine + hgirdsway_offset*pi/180.);
}



/* Actuates spine during swimming */
void
Controller :: swimFun()
{
    static double t_spine=0;
    t_spine+=dt*freq_swim;
    for(int i=0; i<11; i++){
        qs(i)=amps_swim(i)*sin(2*pi*t_spine - phases_swim(i)) + offsets(i);
    }
    qs(0)=-qs(0);

    angles.block<11,1>(0,0)=qs;
    angles.block<16,1>(11,0)=swim_legs_pos*pi/180.;

}

/* Rotation on ellipse */
Vector2d
Controller :: rotEllipse(double a, double b, double rphi, double yoff)
{
    double rs=sin(rphi), rc=cos(rphi);
    double r=a*b/sqrt(a*a*rs*rs + b*b*rc*rc);
    Vector2d p;
    p<<r*rc, r*rs + yoff;
    return p;
}

/* Replay prerecorded angles */
void
Controller :: runAnimalDataWalking()
{

    int line;
    static double t_an=0;
    t_an+=dt*freq_walk;
    t_an=t_an>=1?0:t_an;
    t_an=t_an<0?0.999:t_an;

    line=floor( t_an * 1000 );


    for(int i=0; i<27; i++){
        angles(i)=animalAnglesWalking[line][i];
        //angles(i)=(animalAnglesWalking[line][i]*angSignsCorrAnimal[i] + angShiftsCorrAnimal[i])*pi/180.;
        //angles(i)=((animalAnglesWalking[line][i]-ang_shifts_corr[i])*pi/180.)*ang_signs_corr[i];
    }

     for(int i=0; i<6; i++){
        angles(i)+=gamma/10;
    }

}


/* Replay prerecorded nonperiodic angles */
void
Controller :: runAnimalDataNonperiodic()
{

    int line;
    static double t_an=0;
    t_an+=dt*freq_walk;

    line=floor( t_an * 1000 );

    if(line >= 1000){
        state=INITIAL;
        T_trans=T_trans0;
        t_an=0;
        return;
    }

    for(int i=0; i<27; i++){
        angles(i)=animalAnglesNonperiodic[line][i];
        //angles(i)=(animalAnglesWalking[line][i]*angSignsCorrAnimal[i] + angShiftsCorrAnimal[i])*pi/180.;
        //angles(i)=((animalAnglesWalking[line][i]-ang_shifts_corr[i])*pi/180.)*ang_signs_corr[i];
    }

     for(int i=0; i<6; i++){
        angles(i)+=gamma/10;
    }

}


/* Replay prerecorded angles */
void
Controller :: runAnimalDataSwimming()
{

    int line;
    static double t_an=0;
    t_an+=dt*freq_swim;
    t_an=t_an>=1?0:t_an;
    t_an=t_an<0?0.999:t_an;

    line=floor( t_an * 1000 );


    int ang_signs_corr[11] =
    {
      -1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
    };



    for(int i=0; i<11; i++){
        angles(i)=((animalAnglesSwimming[line][i])*pi/180.)*ang_signs_corr[i];
    }

     for(int i=0; i<11; i++){
        angles(i)+=gamma/10;
    }
    angles.block<16,1>(11,0)=swim_legs_pos*pi/180.;

}

/* Replay prerecorded angles */
void
Controller :: runAnimalDataAquastepping()
{
    int line;
    static double t_an=0;
    t_an+=dt*freq_aquastep;
    t_an=t_an>=1?0:t_an;
    t_an=t_an<0?0.999:t_an;
    line=floor( t_an * 1000 );
    int ang_signs_corr[27] =
    {
      -1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
    1,-1,-1, 1,
    1,-1,-1, 1,
    1, 1, 1, 1,
    1, 1, 1, 1
    };
    int ang_shifts_corr[27] =
    {
    0,0,0,0,0,0,0,0,0,0,0,
    -90,90,0,0,
    -90,90,0,0,
    -90,90,0,0,
    -90,90,0,0
    };

    for(int i=0; i<27; i++){
        angles(i)=((animalAnglesAquastepping[line][i]-ang_shifts_corr[i])*pi/180.)*ang_signs_corr[i];
    }
    angles(12)-=aquastep_pitch_offset;
    angles(16)-=aquastep_pitch_offset;
    angles(20)+=aquastep_pitch_offset;
    angles(24)+=aquastep_pitch_offset;

    for(int i=0; i<6; i++){
        angles(i)+=gamma/15;
    }
}

/* Update robot positions and fbck_torques */
void
Controller :: updateRobotState(double *d_posture, double *d_torque)
{
    for(int i=0; i<27; i++){
        fbck_position(i)=d_posture[i];
        fbck_torques(i)=d_torque[i];
    }

    if(IS_SIMULATION){
        fbck_position=transformation_Ikin_Webots(fbck_position, -1, 1);
        fbck_torques=transformation_Ikin_Webots(fbck_torques, -1, 0);
    }
    else{
        fbck_position=transformation_Ikin_Robot(fbck_position, -1, 1);
        fbck_torques=transformation_Ikin_Robot(fbck_torques, -1, 0);
    }
    //FKIN_posture();
}

/* Read obstacles config file */
void
Controller :: getObstacles()
{
    ifstream file_gates, file_obstacles, file_global_path;
    file_gates.open("/home/mparsapo/catkin_ws/src/pleurobot/data/gates.txt");
    file_obstacles.open("/home/mparsapo/catkin_ws/src/pleurobot/data/obstacles.txt");
    file_global_path.open("/home/mparsapo/catkin_ws/src/pleurobot/data/global_path.txt");
    if(file_gates.is_open()) {
        file_gates >> gateNum;
        file_gates >> gateDist;
        file_gates >> gateLen;

        for(int i=0; i<gateNum; i++){
            file_gates >> gatePos(i, 0); file_gates >> gatePos(i, 1); file_gates >> gatePos(i, 2);
        }
        cout<<"GATES SUCCESSFULLY LOADED"<<endl;
    }
    if(file_obstacles.is_open()) {
        file_obstacles >> obsNum;

        for(int i=0; i<obsNum; i++){
            file_obstacles >> obsPos(i, 0); file_obstacles >> obsPos(i, 1); file_obstacles >> obsPos(i, 2);
        }
        cout<<"OBSTACLES SUCCESSFULLY LOADED"<<endl;
    }
    if(file_global_path.is_open()) {
        file_global_path >> pathNum;

        for(int i=0; i<pathNum; i++){
            file_global_path >> path(i, 0); file_global_path >> path(i, 1);
        }
        cout<<"PLANNED PATH SUCCESSFULLY LOADED"<<endl;
    }

}

/* Path planner */
void
Controller :: pathPlanner()
{

// read current position and orientation from GPS and compass placed on the front girdle:
// cout<<"position: "<<gpsData[0]<<"\t"<<gpsData[1]<<"\t"<<gpsData[2]<<endl;
// cout<<"orientation: "<<compassData[0]<<"\t"<<compassData[1]<<"\t"<<compassData[2]<<endl;

// modify spine curvature (joint offsets) and gait cycle time (speed):
//*gamma=pi/2*sin(0.5*t);
//*tcyc=2+sin(0.7*t);

// following variables might also be usefull:
// t - time vector
// dt - step time

// gates:
// gateNum - number of gates
// gateLen - length of the walls
// gateDist - distance between walls (gate width)
// gatePos - Eigen matrix, each row represents position and orientation of one gate (x, y, phi). number of rows is equal to gateNum
// for example to access y coordinate of 3rd gate, use gatePos(2, 1)  -> indexing starts from 0

// obstacles:
// obsNum - number of obstacles
// obsPos - Eigen matrix, each row represents position and radius of one cylinder obstacle (x, y, r). number of rows is equal to obsNum
// for example to access radius of 4rd gate, use obsPos(3, 2)

// planned path:
// in case you plan the path in matlab (offline) you can store it in a file global_path.txt and read it from here
// it's done in getObstacles() function. The path should be stored in a following format:
// first line: number of points
// all other lines have 2 elements - x and y coordinate
// loaded path is stored in variable "path" which is eigen matrix of size 3000 x 2, and number of points is in pathNum.
// For example to access x coordinate of 5th point - path(4, 0). To access 7th point (both coordinates) - path.block<1, 2>(6,0) (check eigen syntax)


}

/* Get sensor data */
void
Controller :: getSensors(int acc, double *accData_i, int gyro, double *gyroData_i, int compass, double *compassData_i, int gps, double *gpsData_i)
{

    if(acc){
        accData[0]=accData_i[0];
        accData[1]=accData_i[1];
        accData[2]=accData_i[2];
    }
    if(gyro){
        gyroData[0]=gyroData_i[0];
        gyroData[1]=gyroData_i[1];
        gyroData[2]=gyroData_i[2];
    }
    if(compass){
        compassData[0]=compassData_i[0];
        compassData[1]=compassData_i[1];
        compassData[2]=compassData_i[2];
    }
    if(gps){
        gpsData[0]=gpsData_i[0];
        gpsData[1]=gpsData_i[1];
        gpsData[2]=gpsData_i[2];
    }

    #ifdef OPTIMIZATION
    rolling+=gyroData[0]*gyroData[0]*dt;
    pitching+=gyroData[2]*gyroData[2]*dt;
    yawing+=gyroData[1]*gyroData[1]*dt;
    if(t>0.7){
    for(int i=0; i<27; i++){
        applied_torques+=fbck_torques(i)*fbck_torques(i)*dt;
        if(i<11){
            applied_torques_s+=fbck_torques(i)*fbck_torques(i)*dt;
        }
        else{
            applied_torques_l+=fbck_torques(i)*fbck_torques(i)*dt;
        }
    }
    }
    #endif
}


/* Get acc data */
void
Controller :: getAcceleration(double acc[3])
{
    accData[0]=acc[0];
    accData[1]=acc[1];
    accData[2]=acc[2];
}


/* Get sensor data */
void
Controller :: getAttitude(double *attitude, double *rotmat){

    for(int i=0; i<3; attData[i]=attitude[i], i++);

    for(int i=0; i<9; compassRotMat(i)=rotmat[i], i++);
           
}



/* Swimming based on CPG network */
void
Controller :: swimFunCPG()
{
    MatrixXd nu(22,1), R(22,1), a(22,1), w(22,22), ph_lag(22,22), r0(22,1), phi0(22,1), fbck(22,1), x_cpg_ref(11,1);


    nu=MatrixXd::Constant(22,1,freq_swim);
    R.block<11,1>(0,0)=R_cpg;
    R.block<11,1>(11,0)=R_cpg;
    a=MatrixXd::Constant(22,1,a_cpg);

  //  wbck=11; wfwd=5; wlat=1;
  //  ph_lagfwd=-pi/11;
    ph_lagbck=-ph_lagfwd;
    ph_laglat=3.14;

    w=MatrixXd::Zero(22,22);
    w.block<10, 10>(0, 1)=w.block<10, 10>(0, 1)+MatrixXd::Identity(10,10)*wbck;
    w.block<10, 10>(11, 12)=w.block<10, 10>(11, 12)+MatrixXd::Identity(10,10)*wbck;

    w.block<10, 10>(1, 0)=w.block<10, 10>(1, 0)+MatrixXd::Identity(10,10)*wfwd;
    w.block<10, 10>(12, 11)=w.block<10, 10>(12, 11)+MatrixXd::Identity(10,10)*wfwd;

    w.block<11, 11>(11, 0)=w.block<11, 11>(11, 0)+MatrixXd::Identity(11,11)*wlat;
    w.block<11, 11>(0, 11)=w.block<11, 11>(0, 11)+MatrixXd::Identity(11,11)*wlat;


    ph_lag=MatrixXd::Zero(22,22);
    ph_lag.block<10, 10>(0, 1)=ph_lag.block<10, 10>(0, 1)+MatrixXd::Identity(10,10)*ph_lagbck;
    ph_lag.block<10, 10>(11, 12)=ph_lag.block<10, 10>(11, 12)+MatrixXd::Identity(10,10)*ph_lagbck;

    ph_lag.block<10, 10>(1, 0)=ph_lag.block<10, 10>(1, 0)+MatrixXd::Identity(10,10)*ph_lagfwd;
    ph_lag.block<10, 10>(12, 11)=ph_lag.block<10, 10>(12, 11)+MatrixXd::Identity(10,10)*ph_lagfwd;

    ph_lag.block<11, 11>(11, 0)=ph_lag.block<11, 11>(11, 0)+MatrixXd::Identity(11,11)*(-ph_laglat);
    ph_lag.block<11, 11>(0, 11)=ph_lag.block<11, 11>(0, 11)+MatrixXd::Identity(11,11)*ph_laglat;


    r0=MatrixXd::Zero(22,1);
    phi0.block<11,1>(0,0)=MatrixXd::Zero(11,1);
    phi0.block<11,1>(11,0)=MatrixXd::Constant(11,1, pi);
    fbck=MatrixXd::Zero(22,1);
    for(int i=0; i<11; i++){
        fbck(i)=fbck_w(i)*fbck_position(i);
        fbck(i+11)=-fbck(i);
    }

    //

    qs=CPGnetwork(nu, R, a, w, ph_lag, r0, phi0, fbck, cpg_offset);


    angles.block<11,1>(0,0)=qs;
    angles.block<16,1>(11,0)=swim_legs_pos*pi/180.;





    //angles=angles*180/pi;
   // cout<<angles.block<1,11>(0,0)<<endl;

}

/* Run spine CPG network */
MatrixXd
Controller :: CPGnetwork(Matrix<double, 22, 1> nu, Matrix<double, 22, 1> R, Matrix<double, 22, 1> a, Matrix<double, 22, 22> w, Matrix<double, 22, 22> ph_lag, Matrix<double, 22, 1> r0, Matrix<double, 22, 1> phi0, Matrix<double, 22, 1> fbck, double cpg_offset)
{

    static MatrixXd r(22,1), phi(22,1); static ofstream cpgLog("cpgLog.txt"), posLog("posLog.txt");
    MatrixXd tmp1(22,1), tmp2(22,22), ThetaIJ(22,22), ThetaJI(22,22), x(22, 1);
    static int cnt=0;
    if(cnt==0){
        cnt++;
        r=r0;
        phi=phi0;
    }
    for(int i=0; i<22; i++){
        if(i==11){
            cpg_offset*=-1;
        }
        tmp1(i)=(1-cpg_offset)*R(i)-r(i);
    }



    tmp1=tmp1.cwiseProduct(a)*dt;


    for(int i=0; i<22; i++){
        r(i)=r(i) + tmp1(i)+dt*fbck(i)*cos(phi(i));
    }


    ThetaIJ=phi.replicate(1,22);
    ThetaJI=phi.transpose().replicate(22,1);


    tmp2=-ThetaIJ + ThetaJI - ph_lag;
    tmp2=tmp2.array().sin();
    tmp2=tmp2.cwiseProduct(w);
    tmp2=tmp2.cwiseProduct(r.replicate(1, 22));
    tmp1=tmp2.rowwise().sum();

    tmp1=phi + dt * (2 * pi * nu + tmp1);
    for(int i=0; i<22; i++){
        phi(i)=tmp1(i);//-dt*fbck(i)/r(i)*sin(phi(i));
    }




    tmp1=phi.array().cos();
    x=r.cwiseProduct(tmp1)+r;

    for(int i=0; i<22; i++)
        cpgLog<<x(i)<<"\t";
    cpgLog<<endl;

    for(int i=0; i<11; i++)
        posLog<<fbck_position(i)<<"\t";
    posLog<<endl;


    return (x.block<11,1>(0,0)-x.block<11,1>(11,0))/2;
}

/* Subscribing from ROS to read each joint status */
void Controller::GetROSJointStatus(const sensor_msgs::JointState::ConstPtr& msg)
{
 	for(int i=0; i<N_SERVOS; i++){
 		position_ros[i] = msg->position[i];
 		velocity_ros[i] = msg->velocity[i];
 		effort_ros[i] = msg->effort[i];
 	}
}

/* Controller destructor */
Controller::~Controller()
{

}
