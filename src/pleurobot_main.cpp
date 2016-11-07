#define OPTIMIZATION
#include <ros/ros.h>
#include <sensor_msgs/JointState.h> 
#include <iostream>
#include <cmath>
#include "Eigen/Dense"
#include "Eigen/Geometry"
#include <ctime>
#include <fstream>
#include <sys/time.h>
#include <vector>
#include <sstream>
#include <string.h>
#include <stdlib.h>
#include "pleurobot_ros_pkg/controller.h"
#include "pleurobot_ros_pkg/publisher.h"

#define pi          3.141592653589793
#define TIME_STEP   4
#define N_SERVOS    27
#define NUM_SERVOS  27
#define N_anTr      10001

using namespace std;
using namespace Eigen;
int joystick_numbers = 1;

// GLOBAL CONFIG
int IS_SIMULATION;
int IS_PLEUROBOT;
int IS_SNAKE;
int IS_OPTIMIZATION=0;
int USE_JOYSTICK;
int JOYSTICK_TYPE;
int SWIM;
int SPINE_COMPENSATION_WITH_FEEDBACK;
int USE_REFLEXES;
int USE_TAIL;
int USE_IMU;
int USED_MOTORS[27], TORQUE_CTRL_MOTORS[27];
int AUTO_RUN, AUTO_RUN_STATE;
int LOG_DATA;
int ANIMAL_DATASET;

#define Num_Joint 27

int main(int argc, char **argv)
{
    ros::init(argc, argv, "FK_Robot");
    ros::NodeHandle nh1;

    if(readGlobalConfig()==0){
        return 0;
    }

    cout<<"MAIN STARTS"<<endl;
    double Trun = 40;    //runtime [s]
    double Td = 0.004, t=0;
	double dt;
    double table_p[NUM_SERVOS], table_t[NUM_SERVOS];

    Controller controller(Td);
    cout<<"CONTROLLER CREATED"<<endl;
	
    SetJointAngleRobot setJointAngleRobot;
    setJointAngleRobot.setPosition(table_p);

    ros::Rate loop_rate(200);
    //=============================  LOOP  =============================
    while(ros::ok())
    {
	dt=Td;
	t=t+dt;
	controller.setTimeStep(dt);

	if(!controller.runStep()){
	        break;
	}

	ROS_INFO("Reading Joint Angles");

	controller.getAngles(table_p);
	// ANGLES
	cout<<"table_p variable:"<<"\n";
	for(int i=0; i<27; i++){
   	    cout<<table_p[i]<<"\t";
	}
	setJointAngleRobot.setPosition(table_p);

	ROS_INFO("The Pose is updated");
	ros::spinOnce();
	loop_rate.sleep();    
	}
   
    return 0;
}
