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

extern int IS_SIMULATION, IS_PLEUROBOT, USE_JOYSTICK, JOYSTICK_TYPE, SWIM, SPINE_COMPENSATION_WITH_FEEDBACK;
extern int USE_REFLEXES, USE_IMU, AUTO_RUN, AUTO_RUN_STATE, ANIMAL_DATASET;



/* Reads parameters from parameters_ikin.txt file */
bool
Controller :: getParameters()
{
    int i, j;
    stringstream stringstream_file;
    ifstream file_kinematics;
    ifstream file_ikinController;
    ifstream file_animalAnglesWalking;
    ifstream file_animalAnglesNonperiodic;
    ifstream file_animalSpine;
    ifstream file_reflex_posture;
    ifstream file_cpg;
    ifstream file_joystick;
    ifstream file_animalAnglesSwimming;
    ifstream file_VMC;
    ifstream file_snake;
    ifstream file_imageProc;
    ifstream file_hill_muscle;
    ifstream file_animalAnglesAquastepping;
    ifstream file_animalSwingStance;
    ifstream file_forces;
    ifstream file_footsteps;
    ifstream file_arxModel;
    ifstream file_armaxModel;
    ifstream file_bjModel;
    ifstream file_trajFollower;
    ifstream file_masses;
    ifstream file_torquesCtrl;
    file_masses.open("/home/mparsapo/catkin_ws/src/pleurobot/config/masses.config");

    

    if(IS_PLEUROBOT){
        file_animalAnglesWalking.open("/home/mparsapo/catkin_ws/src/pleurobot/config/animal_data/animalAnglesWalking_pleurobot.txt");
        file_animalAnglesNonperiodic.open("/home/mparsapo/catkin_ws/src/pleurobot/config/animal_data/animalAnglesNonperiodic_pleurobot.txt");
        file_animalSpine.open("/home/mparsapo/catkin_ws/src/pleurobot/config/animal_data/animalSpine_pleurobot.txt");
        file_kinematics.open("/home/mparsapo/catkin_ws/src/pleurobot/config/kinematics_pleurobot.config");
        file_animalSwingStance.open("/home/mparsapo/catkin_ws/src/pleurobot/config/animal_data/animalSwingStance_pleurobot.txt");
        file_ikinController.open("/home/mparsapo/catkin_ws/src/pleurobot/config/ikinController_pleurobot.config");
        file_torquesCtrl.open("/home/mparsapo/catkin_ws/src/pleurobot/config/torquesCtrl.config");
    }
    else{
        file_animalAnglesWalking.open("/home/mparsapo/catkin_ws/src/pleurobot/config/animal_data/animalAnglesWalking_orobot.txt");
        file_kinematics.open("/home/mparsapo/catkin_ws/src/pleurobot/config/kinematics_orobot.config");
        file_animalSwingStance.open("/home/mparsapo/catkin_ws/src/pleurobot/config/animal_data/animalSwingStance_orobot.txt");
        file_ikinController.open("/home/mparsapo/catkin_ws/src/pleurobot/config/ikinController_orobot.config");

        switch(ANIMAL_DATASET){
            case 0:
                file_animalAnglesWalking.open("/home/mparsapo/catkin_ws/src/pleurobot/config/animal_data/pleurodeles/animalAnglesWalking_orobot_pleurodeles.txt");
                file_animalSwingStance.open("/home/mparsapo/catkin_ws/src/pleurobot/config/animal_data/pleurodeles/animalSwingStance_orobot_pleurodeles.txt");
                file_animalSpine.open("/home/mparsapo/catkin_ws/src/pleurobot/config/animal_data/pleurodeles/animalSpine_orobot_pleurodeles.txt");
                break;
            case 1:
                file_animalAnglesWalking.open("/home/mparsapo/catkin_ws/src/pleurobot/config/animal_data/ambystoma/animalAnglesWalking_orobot_ambystoma.txt");
                file_animalSwingStance.open("/home/mparsapo/catkin_ws/src/pleurobot/config/animal_data/ambystoma/animalSwingStance_orobot_ambystoma.txt");
                file_animalSpine.open("/home/mparsapo/catkin_ws/src/pleurobot/config/animal_data/ambystoma/animalSpine_orobot_ambystoma.txt");
                break;
            case 2:
                file_animalAnglesWalking.open("/home/mparsapo/catkin_ws/src/pleurobot/config/animal_data/skink/pleurodeles/animalAnglesWalking_orobot_skink.txt");
                file_animalSwingStance.open("/home/mparsapo/catkin_ws/src/pleurobot/config/animal_data/skink/pleurodeles/animalSwingStance_orobot_skink.txt");
                file_animalSpine.open("/home/mparsapo/catkin_ws/src/pleurobot/config/animal_data/skink/animalSpine_orobot_skink.txt");
                break;
            case 3:
                file_animalAnglesWalking.open("/home/mparsapo/catkin_ws/src/pleurobot/config/animal_data/iguana/pleurodeles/animalAnglesWalking_orobot_iguana.txt");
                file_animalSwingStance.open("/home/mparsapo/catkin_ws/src/pleurobot/config/animal_data/iguana/pleurodeles/animalSwingStance_orobot_iguana.txt");
                file_animalSpine.open("/home/mparsapo/catkin_ws/src/pleurobot/config/animal_data/iguana/animalSpine_orobot_iguana.txt");
                break;
            case 4:
                file_animalAnglesWalking.open("/home/mparsapo/catkin_ws/src/pleurobot/config/animal_data/caiman/pleurodeles/animalAnglesWalking_orobot_caiman.txt");
                file_animalSwingStance.open("/home/mparsapo/catkin_ws/src/pleurobot/config/animal_data/caiman/pleurodeles/animalSwingStance_orobot_caiman.txt");
                file_animalSpine.open("/home/mparsapo/catkin_ws/src/pleurobot/config/animal_data/caiman/animalSpine_orobot_caiman.txt");
                break;
            default:                
                file_animalAnglesWalking.open("/home/mparsapo/catkin_ws/src/pleurobot/config/animal_data/pleurodeles/animalAnglesWalking_orobot_pleurodeles.txt");
                file_animalSwingStance.open("/home/mparsapo/catkin_ws/src/pleurobot/config/animal_data/pleurodeles/animalSwingStance_orobot_pleurodeles.txt");
                file_animalSpine.open("/home/mparsapo/catkin_ws/src/pleurobot/config/animal_data/pleurodeles/animalSpine_orobot_pleurodeles.txt");
        }

    }


    file_arxModel.open("/home/mparsapo/catkin_ws/src/pleurobot/config/arxModel.config");
    file_armaxModel.open("/home/mparsapo/catkin_ws/src/pleurobot/config/armaxModel.config");
    file_bjModel.open("/home/mparsapo/catkin_ws/src/pleurobot/config/bjModel.config");

    file_reflex_posture.open("/home/mparsapo/catkin_ws/src/pleurobot/config/reflex_posture.config");
    file_cpg.open("/home/mparsapo/catkin_ws/src/pleurobot/config/cpg.config");
    file_joystick.open("/home/mparsapo/catkin_ws/src/pleurobot/config/joystick.config");
    file_animalAnglesSwimming.open("/home/mparsapo/catkin_ws/src/pleurobot/config/animal_data/animalAnglesSwimming.txt");
    file_VMC.open("/home/mparsapo/catkin_ws/src/pleurobot/config/VMC.config");
    file_snake.open("/home/mparsapo/catkin_ws/src/pleurobot/config/snake.config");
    file_imageProc.open("/home/mparsapo/catkin_ws/src/pleurobot/config/imageProc.config");
    file_hill_muscle.open("/home/mparsapo/catkin_ws/src/pleurobot/config/hill_muscle.config");
    file_animalAnglesAquastepping.open("/home/mparsapo/catkin_ws/src/pleurobot/config/animal_data/animalAnglesAquastepping.txt");
    file_forces.open("/home/mparsapo/catkin_ws/src/pleurobot/config/forces.config");
    file_footsteps.open("/home/mparsapo/catkin_ws/src/pleurobot/config/tracks/footsteps.txt");
    file_trajFollower.open("/home/mparsapo/catkin_ws/src/pleurobot/config/file_trajFollower.config");
    
    

    if(file_kinematics.is_open()) {
        readFileWithLineSkipping(file_kinematics, stringstream_file);

        //spine
        for(i=0; i<11; stringstream_file>>spine_kin(i), i++);    cout<<"spine"<<spine_kin.transpose()<<endl;

        // IG
        stringstream_file>>IG;
        
        // legs
        for(i=0;i<5;i++){
            for(j=0;j<3;j++){
                stringstream_file>>FL_kin(i, j); 
            }
        }    
        cout<<"FL_kin\n"<<FL_kin<<endl;
        for(i=0;i<5;i++){
            for(j=0;j<3;j++){
                stringstream_file>>FR_kin(i, j);
            }
        }   
        cout<<"FR_kin\n"<<FR_kin<<endl;
        for(i=0;i<5;i++){
            for(j=0;j<3;j++){
                stringstream_file>>HL_kin(i, j);
            }
        }   
        cout<<"HL_kin\n"<<HL_kin<<endl;
        for(i=0;i<5;i++){
            for(j=0;j<3;j++){
                stringstream_file>>HR_kin(i, j);
            }
        }       
        cout<<"HR_kin\n"<<HR_kin<<endl;

        for(i=0; i<27; stringstream_file>>angSignsCorrAnimal[i], i++);
        for(i=0; i<27; stringstream_file>>angShiftsCorrAnimal[i], i++);

        for(i=0; i<27; stringstream_file>>angSignsCorrIkin2Webots[i], i++);
        for(i=0; i<27; stringstream_file>>angShiftsCorrIkin2Webots[i], i++);

        for(i=0; i<27; stringstream_file>>angSignsCorrIkin2Robot[i], i++);
        for(i=0; i<27; stringstream_file>>angShiftsCorrIkin2Robot[i], i++);

        for(i=0; i<27; stringstream_file>>angSignsCorrWebots2Robot[i], i++);
        for(i=0; i<27; stringstream_file>>angShiftsCorrWebots2Robot[i], i++);
    }

    if(file_ikinController.is_open()) {
        readFileWithLineSkipping(file_ikinController, stringstream_file);

        for(i=0; i<11; stringstream_file>>amps0(i), i++);    cout<<amps0.transpose()<<endl;
        //amps0=-amps0;
        for(i=0; i<11; stringstream_file>>phases0(i), i++);  cout<<phases0.transpose()<<endl;
        for(i=0; i<11; stringstream_file>>offsets0(i), i++); cout<<offsets0.transpose()<<endl;

        stringstream_file >> hgirdsway_amp;
        stringstream_file >> hgirdsway_offset;     
        // duty cycles for legs (front and hind)
        for(i=0; i<2; stringstream_file>>Duty(i), i++);      cout<<Duty.transpose()<<endl;

        // trajectories
        for(i=0; i<3; stringstream_file>>trPointsFL0(0, i), i++);
        for(i=0; i<3; stringstream_file>>trPointsFL0(1, i), i++);
        for(i=0; i<3; stringstream_file>>trPointsFL0(2, i), i++);     cout<<trPointsFL0<<endl;
        for(i=0; i<4; stringstream_file>>trAnglesF0[i], trAnglesF0[i]*=pi/180., i++); 
        for(i=0; i<2; stringstream_file>>bezierParamF0[i], i++);
        

        for(i=0; i<3; stringstream_file>>trPointsHL0(0, i), i++);
        for(i=0; i<3; stringstream_file>>trPointsHL0(1, i), i++);
        for(i=0; i<3; stringstream_file>>trPointsHL0(2, i), i++);     cout<<trPointsHL0<<endl;
        for(i=0; i<4; stringstream_file>>trAnglesH0[i], trAnglesH0[i]*=pi/180., i++);   
        for(i=0; i<2; stringstream_file>>bezierParamH0[i], i++);                                                    
        
        trPointsFR0=trPointsFL0;
        trPointsHR0=trPointsHL0;
        for(int i=0; i<3; i++){
            trPointsFR0(1, i)*=-1;
            trPointsHR0(1, i)*=-1;
        }

        trPointsFL=trPointsFL0;
        trPointsFR=trPointsFR0;
        trPointsHL=trPointsHL0;
        trPointsHR=trPointsHR0;



        
        stringstream_file>>phiScl;

        // constraints
        for(i=0; i<4; stringstream_file>>constrFL(0, i), constrFL(0, i)*=pi/180., i++);
        for(i=0; i<4; stringstream_file>>constrFL(1, i), constrFL(1, i)*=pi/180., i++);   cout<<constrFL.transpose()<<endl;
        for(i=0; i<4; stringstream_file>>constrFR(0, i), constrFR(0, i)*=pi/180., i++);
        for(i=0; i<4; stringstream_file>>constrFR(1, i), constrFR(1, i)*=pi/180., i++);   cout<<constrFR.transpose()<<endl;
        for(i=0; i<4; stringstream_file>>constrHL(0, i), constrHL(0, i)*=pi/180., i++);
        for(i=0; i<4; stringstream_file>>constrHL(1, i), constrHL(1, i)*=pi/180., i++);   cout<<constrHL.transpose()<<endl;
        for(i=0; i<4; stringstream_file>>constrHR(0, i), constrHR(0, i)*=pi/180., i++);
        for(i=0; i<4; stringstream_file>>constrHR(1, i), constrHR(1, i)*=pi/180., i++);   cout<<constrHR.transpose()<<endl;
        stringstream_file >> constrS;

        /*
        constrF(0, 2)=constrF(0, 2) - pi/2;
        constrF(1, 2)=constrF(1, 2) - pi/2;
        constrH(0, 3)=constrH(0, 3) - pi/2;
        constrH(1, 3)=constrH(1, 3) - pi/2;
        */

        // use animal data for front and hind limbs
        stringstream_file>>useAnDF;                          cout<<useAnDF<<endl;
        stringstream_file>>useAnDH;                          cout<<useAnDH<<endl;
        stringstream_file>>useAnSP;                          cout<<useAnSP<<endl;

        // phase shift between legs
        for(i=0; i<4; stringstream_file>>phShifts(i), i++);  cout<<phShifts.transpose()<<endl;


        // trajectory filtering
        stringstream_file>>Tf1;

        // fixed initial conditions for iKin
        for(i=0; i<4; stringstream_file>>q0FL(i), q0FL(i)*=pi/180., i++);
        for(i=0; i<4; stringstream_file>>q0FR(i), q0FR(i)*=pi/180., i++);
        for(i=0; i<4; stringstream_file>>q0HL(i), q0HL(i)*=pi/180., i++);
        for(i=0; i<4; stringstream_file>>q0HR(i), q0HR(i)*=pi/180., i++);

        // lam, M, Cinv and max_dist for iKinDLS
        
        stringstream_file >> lamF(0);
        stringstream_file >> lamF(1);
        stringstream_file >> lamH(0);
        stringstream_file >> lamH(1);

        for(i=0; i<4; stringstream_file>>MF(0, i), i++);
        for(i=0; i<4; stringstream_file>>MF(1, i), i++);
        for(i=0; i<4; stringstream_file>>MF(2, i), i++);
        for(i=0; i<4; stringstream_file>>MF(3, i), i++);   cout<<MF<<endl;
        for(i=0; i<4; stringstream_file>>MH(0, i), i++);
        for(i=0; i<4; stringstream_file>>MH(1, i), i++);
        for(i=0; i<4; stringstream_file>>MH(2, i), i++);
        for(i=0; i<4; stringstream_file>>MH(3, i), i++);   cout<<MH<<endl;

        stringstream_file >> max_dist;

        stringstream_file >> ikin_tol; cout<<ikin_tol<<endl;

        stringstream_file >> ikin_maxIter; cout<<ikin_maxIter<<endl;

        stringstream_file >> ikin_constr_penalty(0);
        stringstream_file >> ikin_constr_penalty(1);
        stringstream_file >> ikin_constr_penalty(2);

        for(i=0; i<11; stringstream_file>>amps_swim(i), i++);    cout<<amps_swim.transpose()<<endl;

        for(i=0; i<11; stringstream_file>>phases_swim(i), i++);    cout<<phases_swim.transpose()<<endl;

        CinvF=lamF(0)/2.*Matrix4d::Identity()+MF.transpose()*MF;
        CinvF=CinvF.inverse().eval();

        CinvH=lamH(0)/2.*Matrix4d::Identity()+MH.transpose()*MH;
        CinvH=CinvH.inverse().eval();


        stringstream_file>>T_trans0;
        T_trans=T_trans0;

        stringstream_file>>Tfilt_angles;

        for(i=0; i<16; stringstream_file>>swim_legs_pos(i), i++);
        cout<<swim_legs_pos<<endl;

        stringstream_file>>steeringSpinelessGains[0];
        stringstream_file>>steeringSpinelessGains[1];
        stringstream_file>>steeringSpinelessGains[2];
    }


    if(file_animalSwingStance.is_open()){
        readFileWithLineSkipping(file_animalSwingStance, stringstream_file);
        for(int i=0; i<4; i++)
            stringstream_file >> animalStanceLen[i]; 
        for(int i=0; i<4; i++)
            stringstream_file >> animalSwingLen[i];

        for(int i=0; i<1000; i++){
            for(int j=0; j<16; j++){
                stringstream_file >> animalStance[i][j];
                animalStance[i][j]=(animalStance[i][j]*angSignsCorrAnimal[j+11] + angShiftsCorrAnimal[j+11])*pi/180.;
            }
            for(int j=0; j<16; j++){
                stringstream_file >> animalSwing[i][j];
                animalSwing[i][j]=(animalSwing[i][j]*angSignsCorrAnimal[j+11] + angShiftsCorrAnimal[j+11])*pi/180.;
            }
        }
    }

    if(file_footsteps.is_open()){
        readFileWithLineSkipping(file_footsteps, stringstream_file);
        int Nfootsteps;
        stringstream_file >> Nfootsteps;

        for(int i=0; i<Nfootsteps; i++){
            for(int j=0; j<12; j++){
                stringstream_file >> footsteps[i][j];
            }
        }
    }


    if(file_imageProc.is_open()){
        readFileWithLineSkipping(file_imageProc, stringstream_file);
        stringstream_file >> FOV_V;
        stringstream_file >> track_feat_max_count;
        stringstream_file >> track_feat_quality;
        stringstream_file >> track_feat_min_dist;
        stringstream_file >> track_feat_block_size;
        stringstream_file >> track_feat_use_Harris;
        stringstream_file >> track_feat_Harris_k;
        stringstream_file >> LK_flow_win_size[0];
        stringstream_file >> LK_flow_win_size[1];
        stringstream_file >> LK_flow_max_level;
        stringstream_file >> LK_flow_term_crit_max_count;
        stringstream_file >> LK_flow_term_crit_eps;
        stringstream_file >> LK_flow_flags;
        stringstream_file >> LK_flow_min_eig_treshhold;
        stringstream_file >> crop_H0;
        stringstream_file >> crop_W;
        stringstream_file >> warp_in_width;
        stringstream_file >> warp_in_height;
        stringstream_file >> frame_time_step;
        stringstream_file >> MA_WINDOW_SIZE_TIME;
    }
    
    
    if(file_hill_muscle.is_open()) {
        N_hillParams = 12;
        readFileWithLineSkipping(file_hill_muscle, stringstream_file);
        for(int rows=0; rows<N_SERVOS; rows++){
            for(int cols=0; cols<N_hillParams; cols++){
                stringstream_file >> muscleParams(rows,cols);
            }
        }
    cout << "muscle parameters:\n " << muscleParams.block<N_SERVOS,12>(0,0) << endl;
    }
    

    if(file_snake.is_open()){
        readFileWithLineSkipping(file_snake, stringstream_file);
        stringstream_file >> amp_wave_snake;
        stringstream_file >> freq_snake;
        stringstream_file >> snake_phase_lag;
        stringstream_file >> snake_adapt_gain;
        stringstream_file >> snake_adapt_filter;
        stringstream_file >> snake_P_gain;
        stringstream_file >> snake_D_gain;
        stringstream_file >> snake_I_gain;
        stringstream_file >> snake_torque_filt;
        stringstream_file >> snake_torque_damping_filt;
        stringstream_file >> snake_path_gain;
        stringstream_file >> snake_path_limit;
        stringstream_file >> snake_path_acceptance_region;
        stringstream_file >> snake_path_LAD;
        stringstream_file >> snake_path_WP_number;
        for(i=0; i<snake_path_WP_number; stringstream_file>>snake_path_WP(i,0), stringstream_file>>snake_path_WP(i,1), i++);
    }

    if(file_VMC.is_open()){
        readFileWithLineSkipping(file_VMC, stringstream_file);
        stringstream_file >> GW_spring_stiffness_z;
        stringstream_file >> GW_spring_damping_z;
        stringstream_file >> GW_spring_stiffness_xy;
        stringstream_file >> GW_spring_damping_xy;
        stringstream_file >> GW_Zref_initial;
        stringstream_file >> GW_x_offset;
        stringstream_file >> GW_y_offset;
        stringstream_file >> GW_Tfilt;

        stringstream_file >>VMC_roll_ref_front;
        stringstream_file >>VMC_roll_ref_hind;
        stringstream_file >>VMC_roll_spring_stiffness_front;
        stringstream_file >>VMC_roll_spring_stiffness_hind;
        stringstream_file >>VMC_roll_damping_front;
        stringstream_file >>VMC_roll_damping_hind;
    }

    if(file_animalSpine.is_open()){
        readFileWithLineSkipping(file_animalSpine, stringstream_file);
        for(int i=0; i<1000; i++){
            for(int j=0; j<11; j++){
                stringstream_file >> animalSpine(i, j);
                animalSpine(i, j)*=pi/180.;
            }
        }
        
    }

    if(file_animalAnglesWalking.is_open()) {
        readFileWithLineSkipping(file_animalAnglesWalking, stringstream_file);
        for (int lines = 0; lines < 1000; lines++)
        {
            for (int i = 0; i < 27; i++)
            {
                stringstream_file >> animalAnglesWalking[lines][i];
                animalAnglesWalking[lines][i]*=pi/180;
                //animalAnglesWalking[lines][i]=(animalAnglesWalking[lines][i]*angSignsCorrAnimal[i] + angShiftsCorrAnimal[i])*pi/180.;
                //animalAnglesWalking[lines][i]=(animalAnglesWalking[lines][i])*pi/180.;
                /*if(i<11){
                    animalSpine(lines, i)=animalAnglesWalking[lines][i];
                }
                else{
                    animalLegs(lines, i-11)=animalAnglesWalking[lines][i];
                }*/
            }
        }
        cout<<"Animal data (Walking) LOADED"<<endl;

        Matrix<double, 1, 27> ang_signsM, ang_shiftsM;
        ang_signsM << -1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
        1,-1,-1, 1,
        1,-1,-1, 1,
        1, 1, 1, 1,
        1, 1, 1, 1;

        ang_shiftsM << 0,0,0,0,0,0,0,0,0,0,0,
        -90,90,0,0,
        -90,90,0,0,
        -90,90,0,0,
        -90,90,0,0;


        for(int i=0; i<N_anTr; i++) {
            //animalSpine.block<1,11>(i,0)=animalSpine.block<1,11>(i,0)-ang_shiftsM.block<1,11>(0,0);
            //animalSpine.block<1,11>(i,0)=animalSpine.block<1,11>(i,0).cwiseProduct(ang_signsM.block<1,11>(0,0))*pi/180.;
            animalLegs.block<1,16>(i,0)=animalLegs.block<1,16>(i,0)-ang_shiftsM.block<1,16>(0,11);
            animalLegs.block<1,16>(i,0)=animalLegs.block<1,16>(i,0).cwiseProduct(ang_signsM.block<1,16>(0,11))*pi/180.;
        }

    }


        if(file_animalAnglesNonperiodic.is_open()) {
        readFileWithLineSkipping(file_animalAnglesNonperiodic, stringstream_file);
        for (int lines = 0; lines < 1000; lines++)
        {
            for (int i = 0; i < 27; i++)
            {
                stringstream_file >> animalAnglesNonperiodic[lines][i];
                //animalAnglesNonperiodic[lines][i]=(animalAnglesNonperiodic[lines][i]*angSignsCorrAnimal[i] + angShiftsCorrAnimal[i])*pi/180.;
                //animalAnglesWalking[lines][i]=(animalAnglesWalking[lines][i])*pi/180.;
            }
        }
        cout<<"Animal data (Nonperiodic) LOADED"<<endl;
    }


    if(file_reflex_posture.is_open()){
        readFileWithLineSkipping(file_reflex_posture, stringstream_file);
        stringstream_file >> forceFilterExt;
        stringstream_file >> extRefForceLim;
        stringstream_file >> extRefOnFilter;
        stringstream_file >> extRefOffFilter;
        stringstream_file >> extRefSpike;
        stringstream_file >> extRefTimeout;

        stringstream_file >> forceFilterStu;
        stringstream_file >> stuRefForceLimX;
        stringstream_file >> stuRefForceLimZ;
        stringstream_file >> stuRefOnFilter;
        stringstream_file >> stuRefOffFilter;
        stringstream_file >> stuRefDx;
        stringstream_file >> stuRefDz;
        stringstream_file >> stuRefTimeout;
    
        for(i=0;i<11; stringstream_file>>spine_gains0(i), i++);
        for(i=0;i<4; stringstream_file>>legs_height0(i), i++);
        stringstream_file >> erRefOnFilt;
        stringstream_file >> erRefOffFilt;
        stringstream_file >> er_duration;
        stringstream_file >> er_timeout;
        stringstream_file >> er_flow_limit;
        stringstream_file >> er_slipping_Tfilt; 
        stringstream_file >> er_slipping_threshold;
        stringstream_file >> xGain;
        stringstream_file >> yGain;
        stringstream_file >> Troll_posture;
        stringstream_file >> Tpitch_posture;
    
    }

    

    if(file_cpg.is_open()){
        readFileWithLineSkipping(file_cpg, stringstream_file);
        cout<<"CPG param"<<endl;
        stringstream_file>>nu_cpg;
        for(i=0;i<11; stringstream_file>>R_cpg0(i), i++);
        stringstream_file>>R_cpg_gain;
        stringstream_file>>a_cpg;
        stringstream_file>>wbck;
        stringstream_file>>wfwd;
        stringstream_file>>wlat;
        stringstream_file>>ph_lagfwd0;
        for(i=0;i<11; stringstream_file>>fbck_w(i), i++);

        stringstream_file>>akio_legs_sigma_v;           cout << akio_legs_sigma_v<<endl;
        stringstream_file>>akio_legs_sigma_h;           cout << akio_legs_sigma_h<<endl;
        stringstream_file>>akio_legs_sigma_mag;         cout << akio_legs_sigma_mag<<endl;

        for(i=0;i<2; stringstream_file>>girdleOscAmp(i), i++);
        for(i=0;i<2; stringstream_file>>girdleOscOffset(i), girdleOscOffset(i)*=pi/180, i++);
        for(i=0;i<5; stringstream_file>>spineGirdleCPGsWeights(i), i++);
        for(i=0;i<2; stringstream_file>>akio_spine_sigma(i), i++);
            
    }

    if(file_joystick.is_open()){
        readFileWithLineSkipping(file_joystick, stringstream_file);
        stringstream_file >> stick_r_lim;
        stringstream_file >> joy_walk_max_freq;
        stringstream_file >> joy_walk_min_freq;
        stringstream_file >> joy_walk_speed_change_filt;
        stringstream_file >> desiredSpeed; 
        stringstream_file >> speedPriority;


        stringstream_file >> disable_crab_walk_lim;
        stringstream_file >> ellipse_small_axis;
        stringstream_file >> posing_joy_y1_rate;
        stringstream_file >> posing_joy_x1_rate;
        stringstream_file >> posing_joy_y2_rate;
        stringstream_file >> posing_head_rate;
        stringstream_file >> posing_head_limit;
        stringstream_file >> joy_swim_max_freq;
        stringstream_file >> joy_swim_speed_change_filt;
        stringstream_file >> joy_swim_max_offset;
        stringstream_file >> phase_lag_change_rate;
        stringstream_file >> phase_lag_min;
        stringstream_file >> phase_lag_max;
        stringstream_file >> joy_swim_turning_dead_zone; joy_swim_turning_dead_zone*=pi/180;

    }

    if(file_animalAnglesSwimming.is_open()) {
        readFileWithLineSkipping(file_animalAnglesSwimming, stringstream_file);
        for (int lines = 0; lines < 1000; lines++)
        {
            for (int i = 0; i < 11; i++)
            {
                stringstream_file >> animalAnglesSwimming[lines][i];
            }
        }
        cout<<"Animal data (Swimming) LOADED"<<endl;
    }

    if(file_animalAnglesAquastepping.is_open()) {
        readFileWithLineSkipping(file_animalAnglesAquastepping, stringstream_file);
        for (int lines = 0; lines < 1000; lines++)
        {
            for (int i = 0; i < 27; i++)
            {
                stringstream_file >> animalAnglesAquastepping[lines][i];
                if(i<11){
                    //animalSpine(lines, i)=animalAnglesAquastepping[lines][i];
                }
                else{
                    animalLegs(lines, i-11)=animalAnglesAquastepping[lines][i];
                }
            }
        }
        cout<<"Animal data (Aquatic stepping) LOADED"<<endl;
    }

    if(file_forces.is_open()){ cout<<"FILE_FORCES_OPENED"<<endl;
        readFileWithLineSkipping(file_forces, stringstream_file);
        stringstream_file >> coneForceFilter;
        stringstream_file >> coneFriction;
        stringstream_file >> forceTreshold;
    }

    


    if(file_masses.is_open()){
        readFileWithLineSkipping(file_masses, stringstream_file);
        for(i=0; i<23; i++){
            stringstream_file >> masses(0, i);  // mass
            stringstream_file >> masses(1, i);  // coordinates
            stringstream_file >> masses(2, i);
            stringstream_file >> masses(3, i);
            cout<<i<<endl;
        }
        cout<<masses.transpose()<<endl;
    }
/*
    if(file_trajFollower.is_open()){ cout<<"FILE_FORCES_OPENED"<<endl;
        readFileWithLineSkipping(file_trajFollower, stringstream_file);
        for(int i=0; i<10; i++){
            stringstream_file >> trajFollowerParam(i);
        }
    }


    if(file_arxModel.is_open()){
        readFileWithLineSkipping(file_arxModel, stringstream_file);
        stringstream_file >> ARXna; cout<<"ARXna: "<<ARXna<<endl;
        stringstream_file >> ARXnb;
        stringstream_file >> ARXnu;
        stringstream_file >> ARXnk;
        ARXA.resize(1, ARXna);
        ARXB.resize(ARXnu, ARXnb);
        ARXnK.resize(1, ARXnk);
        for(int i=0; i<ARXna; i++){
            stringstream_file >> ARXA(0, i);
        }
        for(int i=0; i<ARXnu; i++){
            for(int j=0; j<ARXnb; j++){
                stringstream_file >> ARXB(i, j);
            }
        }
        for(int i=0; i<ARXnk; i++){
            stringstream_file >> ARXnK(0, i);
        }
        
    }

    if(file_bjModel.is_open()){
        readFileWithLineSkipping(file_bjModel, stringstream_file);
        stringstream_file >> BJnb; 
        stringstream_file >> BJnc;
        stringstream_file >> BJnd;
        stringstream_file >> BJnf;
        stringstream_file >> BJnu;
        stringstream_file >> BJnk;

        cout<<"BJnk:\n "<<BJnk<<endl;

        BJB.resize(BJnu, BJnb);
        BJC.resize(1, BJnc);
        BJD.resize(1, BJnd);
        BJF.resize(BJnu, BJnf);
        BJnK.resize(1, BJnk);

        for(int i=0; i<BJnu; i++){
            for(int j=0; j<BJnb; j++){
                stringstream_file >> BJB(i, j);
            }
        }
        cout<<"BJB:\n "<<BJB<<endl;

        for(int i=0; i<BJnc; i++){
            stringstream_file >> BJC(0, i);
        }
        
        cout<<"BJC:\n "<<BJC<<endl;

        for(int i=0; i<BJnd; i++){
            stringstream_file >> BJD(0, i);
        }

        cout<<"BJD:\n "<<BJD<<endl;

        for(int i=0; i<BJnu; i++){
            for(int j=0; j<BJnf; j++){
                stringstream_file >> BJF(i, j);
            }
        }

        cout<<"BJF:\n "<<BJF<<endl;

        for(int i=0; i<BJnk; i++){
            stringstream_file >> BJnK(0, i);
        }
        
        cout<<"BJnK:\n "<<BJnK<<endl;

    }

*/


    if(file_torquesCtrl.is_open()){
        readFileWithLineSkipping(file_torquesCtrl, stringstream_file);
        double tmp_torque_gain_scaling;    
        stringstream_file>>tmp_torque_gain_scaling;
        for(i=0; i<27; i++){
            stringstream_file>>Pgain_torques(i);
            Pgain_torques(i)*=tmp_torque_gain_scaling;
        }
        cout<<Pgain_torques.transpose()<<endl;


        stringstream_file>>tmp_torque_gain_scaling;
        for(i=0; i<27; i++){
            stringstream_file>>Igain_torques(i);
            Igain_torques(i)*=tmp_torque_gain_scaling;
        }
        cout<<Igain_torques.transpose()<<endl;



        stringstream_file>>tmp_torque_gain_scaling;
        for(i=0; i<27; i++){
            stringstream_file>>Dgain_torques(i);
            Dgain_torques(i)*=tmp_torque_gain_scaling;
        }
        cout<<Dgain_torques.transpose()<<endl;
        
        stringstream_file>>torque_D_filt;
        stringstream_file>>torque_error_filt;
        stringstream_file>>output_torque_filt;

        // VMC bodyPosture
        stringstream_file>>vmc_bodyPosture_stiffness;
        stringstream_file>>vmc_bodyPosture_damping;


    }


    return true;
    
}
