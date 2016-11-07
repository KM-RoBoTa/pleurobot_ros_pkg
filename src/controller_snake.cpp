#include "pleurobot_ros_pkg/controller.h"
#include <vector>
using namespace std;
using namespace Eigen;

extern int IS_SIMULATION, USE_JOYSTICK, JOYSTICK_TYPE, SWIM, SPINE_COMPENSATION_WITH_FEEDBACK, USE_REFLEXES;


/* Snake like movements */
void
Controller :: SnakeController(){

    static double t_snake=0;
    t_snake+=dt;
    // get orientation from compass
    static double snake_compass;
    snake_compass=atan2(compassData[2], compassData[0]);

    // get position of all spine_kin segments
    static MatrixXd snake_theta(12,1), snake_joint_pos(12,2), snake_seg_pos(11,2);
    
    // spine_kin forward kinematics
    snake_theta(0)=snake_compass;
    snake_joint_pos(0,0)=0;
    snake_joint_pos(0,1)=0;
     
    // segment orientations
    for(int i=0; i<11;i++){
	   snake_theta(i+1)=snake_theta(i)+fbck_position(i);
    }
    
    // segment positions
    snake_joint_pos(0,0)=0;
    snake_joint_pos(0,1)=0;
    for(int i=0; i<11; i++) {
    	snake_joint_pos(i+1,0)=spine_kin(i)*cos(snake_theta(i+1)) + snake_joint_pos(i,0);
    	snake_joint_pos(i+1,1)=spine_kin(i)*sin(snake_theta(i+1)) + snake_joint_pos(i,1);

    	snake_seg_pos.block<1,2>(i,0)=(snake_joint_pos.block<1,2>(i,0) + snake_joint_pos.block<1,2>(i+1,0))/2.;
    }
    
    
    // get heading
    static double snake_heading;
    snake_heading=0;
    for(int i=0; i<12; i++){
	   snake_heading+=snake_theta(i);
    }
    snake_heading/=12.0;
    
    // get orientations around the heading
    MatrixXd snake_theta_heading(12,1);
    snake_theta_heading = snake_theta - MatrixXd::Ones(12,1)*snake_heading;
    
  
    // BODY WAVE COMPONENT
    static MatrixXd snake_wave_ref(11,1), snake_theta_ref(12,1);
    //static double wave_head_history[10000]={0};
    static std::vector<double> wave_head_history (10000,0);
    
    
    snake_wave_ref(0)=snake_theta_heading(1) - amp_wave_snake*sin(2*pi*freq_snake * ( t_snake) );
    wave_head_history.pop_back();
    wave_head_history.insert ( wave_head_history.begin() , snake_wave_ref(0) );
    
    for(int i=0; i<11; i++){
    	//snake_wave_ref(i)=snake_theta_heading(1) - amp_wave_snake*sin(2*pi*freq_snake * ( t_snake) - i*snake_phase_lag);
	   snake_wave_ref(i)=amp_wave_snake*sin(2*pi*freq_snake * ( t_snake) - i*snake_phase_lag);
    	//snake_wave_ref(i)=wave_head_history[i*round(snake_phase_lag/(2*pi*freq_snake)/dt)];
    }

    
    // ENVIRONMENT ADAPTATION COMPONENT
    static MatrixXd snake_adapt_ref(11,1), snake_adapt_ref_old=MatrixXd::Zero(11, 1);
    for(int i=1; i<11; i++){
	   snake_adapt_ref(i)=-snake_adapt_gain*(spine_forces(i-1)*cos(snake_theta_heading(i)) - spine_forces(i)*cos(snake_theta_heading(i+1)));
    }
    snake_adapt_ref=pt1_vec(snake_adapt_ref, snake_adapt_ref_old, snake_adapt_filter, dt);
    snake_adapt_ref_old=snake_adapt_ref;
    
    
    // PATH FOLLOWING
    static MatrixXd snake_path_ref=MatrixXd::Zero(11,1);
    static double px, py;
    // get global position
    static MatrixXd snake_seg_pos_global(11,2);
    px=0; py=0;
    for(int i=0; i<11; i++) {
    	snake_seg_pos_global(i,0)=-snake_seg_pos(i,0) + gpsData[0];
    	snake_seg_pos_global(i,1)=-snake_seg_pos(i,1) + gpsData[2];
    	px+=snake_seg_pos_global(i,0)/11.;
    	py+=snake_seg_pos_global(i,1)/11.;
    }

    static int WP_cnt=1;
    static double theta_path_ref;
    static double line_a, line_b, line_c;
    static double cte;
    
    // go through waypoints
    if(WP_cnt<snake_path_WP_number){
    	// get line connecting 2 WPs
    	line_a=snake_path_WP(WP_cnt, 1)-snake_path_WP(WP_cnt-1, 1);
    	line_b=-snake_path_WP(WP_cnt, 0)+snake_path_WP(WP_cnt-1, 0);
    	line_c=-line_a*snake_path_WP(WP_cnt-1, 1) - line_b*snake_path_WP(WP_cnt-1, 0);
    	
    	// get distance of px from that line
    	cte=(line_a*px+line_b*py+line_c)/(sqrt(line_a*line_a+line_b*line_b));
    	
    	theta_path_ref=atan2(cte,snake_path_LAD);
    	snake_path_ref=snake_path_gain*(snake_heading - theta_path_ref)*MatrixXd::Ones(11,1);
    	
    	// check if you're inside acceptance region and if so, switch to the next waypoints
    	static Vector2d wpvec1, wpvec2;
    	wpvec1 << snake_path_WP(WP_cnt-1, 0) - snake_path_WP(WP_cnt, 0), snake_path_WP(WP_cnt-1, 1) - snake_path_WP(WP_cnt, 1);
    	wpvec2 << px - snake_path_WP(WP_cnt, 0), py - snake_path_WP(WP_cnt, 1);
    	if(sqrt((px-snake_path_WP(WP_cnt, 0))*(px-snake_path_WP(WP_cnt, 0)) + (py-snake_path_WP(WP_cnt, 1))*(py-snake_path_WP(WP_cnt, 1)))<snake_path_acceptance_region || (wpvec1(0)*wpvec2(0)+wpvec1(1)*wpvec2(1))<0){
    	    WP_cnt++;
    	    cout<<WP_cnt<<"\t"<<snake_path_WP(WP_cnt, 0)<<"\t"<<snake_path_WP(WP_cnt, 1)<<endl;
    	}
    }
    // limit path correction
    for(int i=0; i<11; i++){
    	if(snake_path_ref(i)>snake_path_limit){
    	    snake_path_ref(i)=snake_path_limit;
    	}
    	else if(snake_path_ref(i)<-snake_path_limit){
    	    snake_path_ref(i)=-snake_path_limit;
    	}
    }

    // TORQUE CONTROL 
    static MatrixXd snake_TS=MatrixXd::Zero(11,1);
    static MatrixXd snake_pos_err(11,1), snake_pos_err_old=MatrixXd::Zero(11,1), snake_pos_err_int=MatrixXd::Zero(11,1);
    
    snake_pos_err = snake_wave_ref + snake_adapt_ref  + snake_path_ref -  fbck_position.block<11,1>(0,0);
    snake_pos_err_int+=snake_pos_err*dt;
    
    
    
    static MatrixXd snake_torque_damping=MatrixXd::Zero(11,1);
    snake_torque_damping=pt1_vec((snake_pos_err - snake_pos_err_old)/dt, snake_torque_damping, snake_torque_damping_filt, dt);
    snake_pos_err_old=snake_pos_err;
    
    snake_TS=pt1_vec(snake_P_gain*snake_pos_err + snake_D_gain*snake_torque_damping + snake_I_gain*snake_pos_err_int, snake_TS, snake_torque_filt, dt);
    
    
    
    
    
    torques.block<11,1>(0,0)=snake_TS;
    angles.block<11,1>(0,0)=snake_wave_ref + snake_adapt_ref + snake_path_ref;
    
    
    
    
    // LOGGING
    static ofstream logSnakeData("data/logSnakeData.txt");
    for(int i=0; i<11; i++){
	   logSnakeData<<fbck_position(i)<<"\t";
    }
    for(int i=0; i<11; i++){
	   logSnakeData<<snake_wave_ref(i)<<"\t";
    }
    for(int i=0; i<11; i++){
	   logSnakeData<<snake_adapt_ref(i)<<"\t";
    }
    for(int i=0; i<11; i++){
	   logSnakeData<<snake_path_ref(i)<<"\t";
    }
    for(int i=0; i<11; i++){
	   logSnakeData<<snake_seg_pos_global(i, 0)<<"\t";
    }
    for(int i=0; i<11; i++){
	   logSnakeData<<snake_seg_pos_global(i, 1)<<"\t";
    }
    
    
    
    
    logSnakeData<<t_snake<<"\t"<<endl;
    
    angles.block<16,1>(11,0)=swim_legs_pos*pi/180.;
    
}

/* Snake like movements */
void
Controller :: SnakeGetForces(double *ts_data){
    for(int i=0; i<12; i++){
	   spine_forces(i)=ts_data[i]; //cout<<spine_forces(i)<<"\t";
    }
    //cout<<endl;
}
























