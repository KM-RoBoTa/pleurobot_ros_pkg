#ifndef PUBLISHER_H
#define PUBLISHER_H

#include <ros/ros.h>        
#include <std_msgs/Float64.h>
#include <sensor_msgs/JointState.h>

#define Num_Joint 27

using namespace std;

class SetJointAngleRobot{
public:
    SetJointAngleRobot();
    void setPosition(double *table_P);
    
private:  
    ros::NodeHandle ph_;
    ros::Publisher joint_pub_[27];
};
#endif
