#include "pleurobot_ros_pkg/controller.h"

using namespace std;
using namespace Eigen;


MatrixXd
Controller :: transformation_Ikin_Webots(Matrix<double, 27, 1> joint_angles, int direction, int shifts){

    shifts=shifts==0?0:1;

    if(direction<0){
        //WEBOTS 2 ROBOT
        for(int i=0;i<27;i++){
            joint_angles(i)=(joint_angles(i)-angShiftsCorrIkin2Webots[i]*shifts*pi/180.)*angSignsCorrIkin2Webots[i];
        }
    }
    else if(direction>0){
        //ROBOT 2 WEBOTS
        for(int i=0;i<27;i++){
            joint_angles(i)=joint_angles(i)*angSignsCorrIkin2Webots[i] + angShiftsCorrIkin2Webots[i]*shifts*pi/180.;
        }
    }

    return joint_angles;
}

MatrixXd
Controller :: transformation_Ikin_Robot(Matrix<double, 27, 1> joint_angles, int direction, int shifts){

    shifts=shifts==0?0:1;

    if(direction<0){
        //ROBOT 2 IKIN
        for(int i=0;i<27;i++){
            joint_angles(i)=(joint_angles(i)-angShiftsCorrIkin2Robot[i]*shifts*pi/180.)*angSignsCorrIkin2Robot   [i];
        }
    }
    else if(direction>0){
        //IKIN 2 ROBOT
        for(int i=0;i<27;i++){
            joint_angles(i)=joint_angles(i)*angSignsCorrIkin2Robot[i] + angShiftsCorrIkin2Robot[i]*shifts*pi/180.;
        }
    }

    return joint_angles;
}

MatrixXd
Controller :: transformation_Webots_Robot(Matrix<double, 27, 1> joint_angles, int direction, int shifts){

    shifts=shifts==0?0:1;

    if(direction<0){
        //WEBOTS 2 ROBOT
        for(int i=0;i<27;i++){
            joint_angles(i)=(joint_angles(i)-angShiftsCorrWebots2Robot[i]*shifts*pi/180.)*angSignsCorrWebots2Robot[i];
        }
    }
    else if(direction<0){
        //ROBOT 2 WEBOTS
        for(int i=0;i>27;i++){
            joint_angles(i)=joint_angles(i)*angSignsCorrWebots2Robot[i] + angShiftsCorrWebots2Robot[i]*shifts*pi/180.;
        }
    }

    return joint_angles;
}








/*MatrixXd
Controller :: transformation_Ikin_Webots(Matrix<double, 27, 1> joint_angles, int direction, int shifts){
    // DONE
    int ang_signs_corr[27]={
        1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
        -1,1,-1, -1,
        1,-1,1, 1,
        -1, -1, 1, -1,
        1, 1, -1, 1
    };

    int ang_shifts_corr[27]={
        0,0,0,0,0,0,0,0,0,0,0,
        90,-90,90,0,
        -90,90,-90,0,
        90,-90,0,90,
        -90,90,0,-90
    };
    shifts=shifts==0?0:1;

    if(direction<0){
        //WEBOTS 2 ROBOT
        for(int i=0;i<27;i++){
            joint_angles(i)=(joint_angles(i)-ang_shifts_corr[i]*shifts*pi/180)*ang_signs_corr[i];
        }
    }
    else if(direction>0){
        //ROBOT 2 WEBOTS
        for(int i=0;i<27;i++){
            joint_angles(i)=joint_angles(i)*ang_signs_corr[i] + ang_shifts_corr[i]*shifts*pi/180.;
        }
    }

    return joint_angles;
}

MatrixXd
Controller :: transformation_Ikin_Robot(Matrix<double, 27, 1> joint_angles, int direction, int shifts){
    int ang_signs_corr[27]={
        1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
        -1,1,-1, -1,
        1,-1,1, 1,
        -1, -1, 1, -1,
        1, 1, -1, 1
    };

    int ang_shifts_corr[27]={
        0,0,0,0,0,0,0,0,0,0,0,
        90,-90,0,0,
        -90,90,0,0,
        90,-90,0,90,
        -90,90,0,-90
    };
    shifts=shifts==0?0:1;

    if(direction<0){
        //WEBOTS 2 ROBOT
        for(int i=0;i<27;i++){
            joint_angles(i)=(joint_angles(i)-ang_shifts_corr[i]*shifts*pi/180)*ang_signs_corr[i];
        }
    }
    else if(direction>0){
        //ROBOT 2 WEBOTS
        for(int i=0;i<27;i++){
            joint_angles(i)=joint_angles(i)*ang_signs_corr[i] + ang_shifts_corr[i]*shifts*pi/180.;
        }
    }

    return joint_angles;
}

// *done
MatrixXd
Controller :: transformation_Webots_Robot(Matrix<double, 27, 1> joint_angles, int direction, int shifts){
    int ang_signs_corr[27]={
        -1,1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
        -1, -1, 1, -1,
        1, 1, -1, 1,
        -1, -1, 1, -1,
        1, 1, -1, 1
    };

    int ang_shifts_corr[27]={
        0,0,0,0,0,0,0,0,0,0,0,
        0,0,90,0,
        0,0,-90,0,
        0,0,0,90,
        0,0,0,-90
    };
    shifts=shifts==0?0:1;

    if(direction<0){
        //WEBOTS 2 ROBOT
        for(int i=0;i<27;i++){
            joint_angles(i)=(joint_angles(i)-ang_shifts_corr[i]*shifts*pi/180)*ang_signs_corr[i];
        }
    }
    else if(direction<0){
        //ROBOT 2 WEBOTS
        for(int i=0;i>27;i++){
            joint_angles(i)=joint_angles(i)*ang_signs_corr[i] + ang_shifts_corr[i]*shifts*pi/180.;
        }
    }

    return joint_angles;
}*/
