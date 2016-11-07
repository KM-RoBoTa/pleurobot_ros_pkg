#ifndef CONTROLLER_H
#define CONTROLLER_H

#define OPTIMIZATION
//#define EIGEN_DONT_ALIGN_STATICALLY
#define EIGEN_DONT_VECTORIZE
#define EIGEN_DISABLE_UNALIGNED_ARRAY_ASSERT

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include "Eigen/Dense"
#include "Eigen/Geometry"
#include "Eigen/SVD"
#include "pleurobot_ros_pkg/joystick.h"
#include "pleurobot_ros_pkg/utils.h"
#include <fstream>
#include <pthread.h>
#include <vector>
#include <string.h>
/////// OPENCV ///////////////////
#include "opencv2/opencv.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/video/tracking.hpp"
#include <opencv2/core/eigen.hpp>
//////// MUSCLE //////////////////
#include "pleurobot_ros_pkg/hill_muscle.h"
/////////////////////////////////

#define pi          3.141592653589793

#define N_SERVOS    27
#define N_anTr      1000
#define maxSpeed    50    // mm/s
#define JOYSTICK_DEVNAME "/dev/input/js0"
/*
#define USE_JOYSTICK   1
#define JOYSTICK_TYPE 1     //PS1 - 1, PS3 - 2, Logitech - 3
#define SWIM   0
#define IS_SIMULATION 1
#define SPINE_COMPENSATION_WITH_FEEDBACK 0
*/

using namespace Eigen;
//using namespace Robot;
//using Eigen::Matrix Matrix;
enum{WALKING, STANDING, POSING, SWIMMING, ANIMAL_WALKING, ANIMAL_SWIMMING, ANIMAL_AQUASTEP, INITIAL, NONPERIODIC};

class Controller{

  public:
    //================== public variables ===============================================
    Joystick js;
    int state;
    double gamma, tcyc, tcyc_an, freq_swim, freq_walk, freq_aquastep;
    Matrix<double, 27, 1> fbck_torques, fbck_position, joint_angles;
	Matrix<double,12, 1> force, force_filt;
    Matrix<double,3, 1> posture;
    Matrix<double, 4, 1> legs_stance;
    Eigen::Matrix<float,Dynamic,Dynamic> imX, imY;
    double compassData[3], gpsData[3], gyroData[3], accData[3];
    unsigned char *imgArray;
    Matrix<double, 3, 27> cart_joint_pos;
    Matrix<double, 3, 27> global_joint_pos;
    Matrix<double, 3, 4> global_feet_pos;
    //Matrix<double, 3, 4> feet_position_local;
    Matrix<double, 3, 4> feet_position_global;
    Matrix<double, 3, 4> feet_orientation_local;
    Matrix<double, 12, 3> feet_orientation_global;
    Matrix<double, 3, 4> feet_force, fc_feet_force;
    vector<Matrix4d> HJs, HJs_g;
    vector<Matrix4d> HJfl, HJfr, HJhl, HJhr, HJfl_g, HJfr_g, HJhl_g, HJhr_g;
    vector<MatrixXd> legJacob;
    Matrix<double, 4, 4> Fgird, Hgird, Fgird0, Hgird0, Fgird_mean, Hgird_mean, HJh, HJh_g;
    double hgirdlePhi;
    Vector4d phaseTransitionStanceSwing, phaseTransitionSwingStance, legs_stance_old;
    double IG;
	
	// ROS variables to read joint status
	double effort_ros[27], velocity_ros[27], position_ros[27];   

    // needed for RUN
    Matrix<double, 11, 1> spine_kin, amps0, amps, phases0, phases, offsets0, offsets, phases_swim, offsets_swim, amps_swim, qs_posing, qsr;
    Matrix<double, 5, 3> FL_kin, FR_kin, HL_kin, HR_kin;
    Matrix<double, 2, 1> Duty;
    Matrix<double, 4, 1> phShifts;
    Matrix<double, 3, 3> trPointsFL0, trPointsFR0, trPointsFL, trPointsFR, trPointsHL0, trPointsHR0, trPointsHL, trPointsHR;
    Matrix<double, 4, 1> stancePhase, swingPhase, legPhase;

    //joystick manipulation
    double stick_r_lim, joy_walk_max_freq, joy_walk_min_freq, joy_walk_speed_change_filt, disable_crab_walk_lim;
    double posing_xFH, posing_yF, posing_zF, posing_head;
    double crab_rotAng, ellipse_small_axis;
    double posing_joy_x1_rate, posing_joy_y1_rate, posing_joy_y2_rate, posing_head_rate, posing_head_limit;
    double joy_swim_max_offset, joy_swim_max_freq, joy_swim_speed_change_filt, joy_swim_turning_dead_zone, aquastep_pitch_offset;
    double steeringRadius;

    //cpg
    double wbck, wfwd, wlat, ph_lagbck, ph_lagfwd0, ph_lagfwd, ph_laglat, nu_cpg, a_cpg, cpg_offset, phase_lag_change_rate, phase_lag_min, phase_lag_max, R_cpg_gain;
    Matrix<double, 11, 1> R_cpg, fbck_w, R_cpg0;
    
    //muscles...
    hill_muscle hill_muscles[N_SERVOS][2];
    int N_hillParams;
    Matrix<double, N_SERVOS, 1> phi_attach, phi_attach_opt, phi_attach_conv;
    Matrix<double, 50,50> muscleParams;  

    //joystick
    double joy_x1, joy_x2, joy_y1, joy_y2, joy_x3, joy_y3;
    int joy_l1, joy_l2, joy_l3, joy_r1, joy_r2, joy_r3, joy_sel, joy_start, joy_bD, joy_bL, joy_bR, joy_bU;
    double joy_lsr, joy_rsr;
    double joy_lsphi, joy_rsphi;

    // force sensors and friction cones
    double coneForceFilter, coneFriction, forceTreshold;   
    Matrix<double, 4, 1> feet_friction_angles, feet_is_slipping;

    //reflexes 
    int stu_reflex_active[4], ext_reflex_active[4];
    double extRefForceLim;
    double extRefOnFilter;
    double extRefOffFilter;
    double extRefSpike;
    double extRefTimeout, stuRefTimeout;
    double stuRefForceLimX, stuRefForceLimZ;
    double stuRefOnFilter;
    double stuRefOffFilter;
    double stuRefDx;
    double stuRefDz;
    Matrix<double, 3, 3> compassRotMat, fgird2ForRotMat;

    // footsteps
    double footsteps[2000][12];
    Matrix<double, 20, 1> trajFollowerParam;

    double attData[3];
    Vector3d globalPosFL, globalPosFR, globalPosHL, globalPosHR;
    Vector3d gpsPos;
    Matrix3d rotMatFgird;
    double FoROrientation;
    Vector2d FFParam;

    // estimation models
    Matrix<double, Dynamic, Dynamic> ARXA, ARXB, ARXnK;
    Matrix<double, Dynamic, Dynamic> BJB, BJC, BJD, BJF, BJnK;
    int ARXna, ARXnb, ARXnu, ARXnk;
    int BJnb, BJnc, BJnd, BJnf, BJnu, BJnk;
    Vector3d estVelFromGps;
    double desiredSpeed; 
    int speedPriority;

    // girdle trajectories
    Matrix<double, 6, 2> girdleTraj;
    Matrix<double, 2, 4> legStanceTraj;

    //================== public functions ===============================================
    Controller(double time_step); // constructor
    ~Controller(); //Destructor
	void GetROSJointStatus(const sensor_msgs::JointState::ConstPtr& msg); // Callback function for ROS Subscriber
    void setTimeStep(double time_step);
    bool runStep();
    void readJoystick();
    bool updateState();
    void getAngles(double table[N_SERVOS]);
    void getTorques(double table[N_SERVOS]);
    void oneLegTesting();
    void forwardKinematics();
    MatrixXd legKinematics(Vector4d q, int leg);
    Matrix<double, 3, 4> Jacob(Vector4d q, int leg);
    MatrixXd forceEstimation();
    void updateRobotState(double *d_posture, double *d_torque);
    void getForce(double *force);
    void getSensors(int acc, double *accData_i, int gyro, double *gyroData_i, int compass, double *compassData_i, int gps, double *gpsData_i);
    void getAttitude(double *attitude, double *rotmat);
    void globalKinematics(double gpsPosition[3], double rotMat[9]);
    void getFloatingFrame();
    #ifndef OPTIMIZATION
        cv::Mat camImg;
        static void * imageProc(void*);
        void imageShow(unsigned char *img0);
        void getCamera();
        void initImageProcessing(unsigned char *srcPtr);
        void getImage(unsigned char * src);
        void optFlowFarneback();
        void optFlowLK();
        void visualFeedback();
    #endif OPTIMIZATION
    #ifdef OPTIMIZATION
        void optimizationInit();
        void optimizationEnd();
        int optimizationShouldWeStopIt(double timestep);
        std::vector<double> params;
        std::vector<double> settings;
        std::vector<std::string> params_names;
        std::vector<std::string> settings_names;
        double rolling, pitching, yawing, applied_torques, applied_torques_l, applied_torques_s;
    #endif

    // muscles
    void createHillMuscles(Matrix<double,N_SERVOS,1>, Matrix<double,N_SERVOS,1>, MP [N_SERVOS]);
    void updateHillMuscles(Matrix<double,N_SERVOS,2>, double [N_SERVOS], double);
    double muscleLengthFromAngles(int,int,double);
    void createEkebergMuscles();
    void updateEkebergMuscles();
    
    // playing with torque
    bool DirectTorqueSetup();

    void SnakeGetForces(double *ts_data);

    // estimation
    double arxModel(MatrixXd u, MatrixXd y0, MatrixXd A, MatrixXd B, MatrixXd nK, double NoiseVariance, bool NoiseIntegration);
    double bjModel(MatrixXd u, MatrixXd y0, MatrixXd B, MatrixXd C, MatrixXd D, MatrixXd F, MatrixXd nK, double NoiseVariance, bool NoiseIntegration);
    double speedEstimation();
    void getAcceleration(double acc[3]);
    void getFeetPosition(double *fl, double *fr, double *hl, double *hr);
    MatrixXd getReferenceFootsteps();
    void GetCompass(double data[3]);
    void GetFeetGPS(double *FL_feet_gpos, double *FR_feet_gpos, double *HL_feet_gpos, double *HR_feet_gpos);
    void GetVelocityFromGPS();
    void maintainDesiredSpeed();
    Vector3d getCoM();
    void torquePID();

  private:
    //================== private variables ===============================================
	// ROS - Subscriber
    ros::NodeHandle nh_;
    ros::Subscriber joint_sub_;

    int useAnDF, useAnDH, useAnSP, ikin_maxIter;
    double ikin_tol, t, dt;
    double T_trans, T_trans0;
    Vector3d ikin_constr_penalty;
    // joystick objects
    js_event event;
    
    double trAnglesF0[4], trAnglesH0[4], bezierParamF0[2], bezierParamH0[2];

    Matrix<double, 2, 4> constrFL, constrHL, constrFR, constrHR; 
    double constrS;
    Matrix<double, 3, 1> posFL, posFR, posHL, posHR, pFL0, pFR0, pHL0, pHR0, rFL, rFR, rHL, rHR, pFL, pFR, pHL, pHR, tmp3, rFL_posing, rFR_posing, rHL_posing, rHR_posing;
    Matrix<double, 4, 1> reCon, rsCon, vmCon;
    double hgirdsway, hgirdsway_amp, hgirdsway_offset;
	double z_offset;
    double angSignsCorrAnimal[27], angShiftsCorrAnimal[27];
    double angSignsCorrIkin2Webots[27], angShiftsCorrIkin2Webots[27];
    double angSignsCorrIkin2Robot[27], angShiftsCorrIkin2Robot[27];
    double angSignsCorrWebots2Robot[27], angShiftsCorrWebots2Robot[27];

    // constants
    Matrix4d DH1, DH2;

    // animal data
    Matrix<double, N_anTr, 11> animalSpine;
    Matrix<double, N_anTr, 16> animalLegs;
    Matrix<double, 27, 1> angles, torques;


    // initial conditions
    Vector4d q0FL, q0FR, q0HL, q0HR, qFL, qFR, qHL, qHR;
    Matrix<double, 27, 1> init_angles;


    // auxiliary variables
    Matrix<double, 11, 1> qs;
    Matrix<double, N_anTr, 1> Err;
    Matrix<double, 12, 2> jpos, jpos2, pos;
    Matrix<double, 11, 1> phi, phi2;
    Matrix<double, N_anTr,1> tmpX, tmpY, tmpZ;
    Matrix<double, 4,4> CinvF, CinvH, MF, MH;
    double max_dist;
    double phiScl;
    double headq;
    Vector2d lamF, lamH;


    Vector2d posF, posH, posF2, posH2;
    double girdHiAng, R;
    // Vector3d pFL, pFR, pHL, pHR;
    Matrix2d rot2;
    Matrix3d rot3;

    // dynamics
    double spd; //spine
    double Tf1; // trajectory filtering
    double Tfilt_angles;

    // filter parameters
    Transform<double,3,Affine> HFL, HHL, HFR, HHR;

    double  tmps;
    double forceFilterExt, forceFilterStu;

    // trajectories
    double nfl, nfr, nhl, nhr, rFLy, rFRy, rHLy, rHRy, fxfl, fxfr, fxhl, fxhr;
    
    //
    double animalAnglesWalking[1000][27], animalAnglesSwimming[1000][11], animalAnglesAquastepping[1000][27], animalAnglesNonperiodic[1000][27];
    int isTalking;

    // obstacles
    Matrix<double, 100, 3> gatePos, obsPos;
    Matrix<double, 3000, 2> path;
    int gateNum, obsNum, pathNum;
    double gateDist;
    double gateLen;

    double xGain, yGain, Troll_posture, Tpitch_posture;
    
    Matrix<double, 11, 1> spine_gains0;
    Matrix<double, 4, 1> legs_height0;
    double erRefOnFilt, erRefOffFilt, er_timeout, er_duration, er_flow_limit;
    double er_slipping_Tfilt, er_slipping_threshold;

    //swimming
    Matrix<double, 16, 1> swim_legs_pos;

    //VMC
    double GW_spring_stiffness_z, GW_spring_damping_z, GW_spring_stiffness_xy, GW_spring_damping_xy, GW_Zref_initial;
    double GW_x_offset, GW_y_offset;
    double GW_Tfilt;

    double VMC_roll_ref_front, VMC_roll_ref_hind, VMC_roll_spring_stiffness_front, VMC_roll_spring_stiffness_hind, VMC_roll_damping_front, VMC_roll_damping_hind;

    // SNAKE STUFF
    double amp_wave_snake, freq_snake, snake_phase_lag, snake_adapt_gain, snake_adapt_filter;
    double snake_P_gain, snake_D_gain, snake_I_gain, snake_torque_filt, snake_torque_damping_filt;
    double snake_path_gain, snake_path_acceptance_region, snake_path_WP_number, snake_path_LAD, snake_path_limit;
    Matrix<double, 50, 2> snake_path_WP;
    Matrix<double, 12, 1> spine_forces;
     
    // IMAGE PROCESSING STUFF
    double FOV_V;
    double track_feat_quality, track_feat_min_dist, track_feat_Harris_k;
    int track_feat_max_count, track_feat_block_size;
    bool track_feat_use_Harris;
    
    double LK_flow_term_crit_eps, LK_flow_min_eig_treshhold, frame_time_step;
    int LK_flow_win_size[2], LK_flow_max_level, LK_flow_term_crit_max_count, crop_H0, crop_W, LK_flow_flags, warp_in_width, warp_in_height;
    double MA_WINDOW_SIZE_TIME;
    
    double filtFlowX, filtFlowY;
    		
	// Animal data for Inverse Kinematics
    double animalSwing[1000][16], animalStance[1000][16];
    int animalSwingLen[4], animalStanceLen[4];
       
	// trackways
    double steeringSpinelessGains[3];
    Matrix<double, 3, 4> feetGPS;
		
	// akio stuff
    double akio_legs_sigma_v, akio_legs_sigma_h, akio_legs_sigma_mag;
	
	Matrix<double, 5, 1> spineGirdleCPGsWeights;
    Matrix<double, 2, 1> girdleOscAmp, girdleOscOffset, akio_spine_sigma;

    Matrix<double, 4, 23> masses;
    Matrix<double, 3, 1> CoM;
	
    Matrix<double, 27, 1> Pgain_torques, Igain_torques, Dgain_torques;
    double torque_D_filt, torque_error_filt, output_torque_filt, vmc_bodyPosture_stiffness, vmc_bodyPosture_damping;
        
    //================== private functions ===============================================
    void Hmat(MatrixXd *H, MatrixXd DH, Vector4d q);

    Vector4d iKinCCD(Matrix4d DH, Vector3d p, Vector4d q0, double tol, int iter, double *w);
    Vector4d iKinDLS(Matrix4d H_leg, Vector3d pref, Vector4d qref, Vector4d q0, Matrix4d Cinv, Matrix4d M, double max_dist, int leg);
    Vector4d iKinIDLS(Matrix4d DH, Vector3d pref, Vector4d qref, Vector4d q0, Matrix4d Cinv, Matrix4d M, double max_dist, int leg, int maxIter, double tol);
    Vector4d iKinNullIDLS(int leg, Vector3d pref, Vector4d qref, Vector4d q0, Vector2d lam, Matrix4d M, 
                            double max_dist, int maxIter, double tol, MatrixXd constr, Vector3d constr_penalty);
    Vector3d steerFun(Vector3d p, Vector2d R, double phiScl);
    void steeringSpineless(double heading_omega);
    void joystickManipulation();
    bool getParameters();
    void runSpine();
    void spineFromGirdleOsc(double dt);
    void spineFromGirdle(double dt);
    void walkingTrajectories();
    void legExtension();
    void stumbleReflex();
    void emergencyReflex();
    void pathPlanner();
    void swimFun();
    Vector3d contactDetection();
    void Kalman();
    void postureControl();
    Vector2d rotEllipse(double a, double b, double rphi, double yoff);
    void runAnimalDataWalking();
    void runAnimalDataSwimming();
    void runAnimalDataAquastepping();
    void runAnimalDataNonperiodic();
    void getObstacles();
    MatrixXd transformation_Ikin_Webots(Matrix<double, 27, 1> joint_angles, int direction, int shifts);
    MatrixXd transformation_Ikin_Robot(Matrix<double, 27, 1> joint_angles, int direction, int shifts);
    MatrixXd transformation_Webots_Robot(Matrix<double, 27, 1> joint_angles, int direction, int shifts);
    void FKIN_posture();
    void swimFunCPG();
    void inverseKinematicsController();
    void VmcController();
    void SnakeController();
    void slippingFromFrictionCone();
    MatrixXd CPGnetwork(Matrix<double, 22, 1> nu, Matrix<double, 22, 1> R, Matrix<double, 22, 1> a, Matrix<double, 22, 22> w, Matrix<double, 22, 22> ph_lag, Matrix<double, 22, 1> r0, Matrix<double, 22, 1> phi0, Matrix<double, 22, 1> fbck, double cpg_offset);
    void simpleTrajectoryFollower();
    void gpsCheatedTrajectoryFollower();
    void headingController(double heading_angle);
    void legPhaseDynamics(double dt);
    void girdleTrajectories(double v, double w);
    void legTrajFromGirdleTraj();
    MatrixXd legSwingTraj(MatrixXd initPoint, MatrixXd currPoint, MatrixXd trPoints, double phase, double trAngles[4], double bezierParam[2]);
    void walkingTrajectories2();
    void getLegJacobians();
    void VMC_bodyPosture();
    
};

#endif