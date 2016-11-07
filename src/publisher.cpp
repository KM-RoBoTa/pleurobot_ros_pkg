#include "pleurobot_ros_pkg/publisher.h"

SetJointAngleRobot::SetJointAngleRobot()
{        
    for(int k=0;k<Num_Joint;k++)
    { 
        std::stringstream pub_topic;      
        pub_topic << "/pleurobot/j" << k+1 << "_position_controller/command";
        joint_pub_[k]=ph_.advertise<std_msgs::Float64>(pub_topic.str().c_str(),10);		
    }
}

void SetJointAngleRobot::setPosition(double *table_P)
{    
	std_msgs::Float64 msg;
	for(int i=0;i<Num_Joint;i++){
		msg.data=table_P[i];
		joint_pub_[i].publish(msg);
	}
}
