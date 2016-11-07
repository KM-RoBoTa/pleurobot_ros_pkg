#ifndef PLEUROBOTSIM_HPP
#define PLEUROBOTSIM_HPP


#define OPTIMIZATION
#include "Eigen/Dense"
#include "Eigen/Geometry"
#include <sys/time.h>
#include "pleurobot_ros_pkg/joystick.h"
#include "pleurobot_ros_pkg/utils.h"
#include <fstream>
#include <iostream>
#include <webots/Supervisor.hpp>
#include <webots/Servo.hpp>
#include <webots/Robot.hpp>
#include <webots/TouchSensor.hpp>
#include <webots/Node.hpp>
#include <webots/Field.hpp>
#include "physicsPluginCom.hpp"
#define pi          3.141592653589793

#define N_SERVOS    27
#define N_TOUCH_SENSORS    4
#define N_anTr      1000
#define maxSpeed    50    // mm/s
#define JOYSTICK_DEVNAME "/dev/input/js1"
#define JOYSTICK_DEVNAME2 "/dev/input/js3"



using namespace std;
//using namespace Eigen;
//using namespace Robot;
//using Eigen::Matrix Matrix;



class PleurobotSim : public webots::Supervisor{


  public:
    //================== public variables ===============================================
    webots::Servo *servo[N_SERVOS], *linservo[3];
    webots::Node *servo_node[N_SERVOS];
    webots::Gyro *gyro;
    webots::Accelerometer *acc;
    webots::Compass *compass;
    webots::GPS *gps;
    webots::Camera *camera;
    webots::TouchSensor *touch_sensor[N_TOUCH_SENSORS];
    webots::TouchSensor *touch_sensor_spine[12];
    webots::Node *fgirdle, *FL_marker, *FR_marker, *HL_marker, *HR_marker, *CoM_marker, *pleuroDef, *tsdefFL, *tsdefFR, *tsdefHL, *tsdefHR;
    const double *compassData, *gpsData, *gyroData, *accData, *ts_fl, *ts_fr, *ts_hl, *ts_hr, *rotMat, *posFL, *posFR, *posHL, *posHR;
    webots::Field *pleuroRot, *pleuroPos, *CoM_marker_pos;
    double gamma, tcyc, tcyc_an;
    unsigned char camImg[230400];
    double t_total;

    //================== public functions ===============================================
    PleurobotSim(int TIME_STEP); // constructor
    void posture(double*,int*);
    void torques(double*, int*);
    void ReadSensors(double *d_posture, double *d_torques, double *gyroData, double *accData, double *gpsData);
    void GetPosition(double *gpsData);
    void InitIMU();
    void ReadIMUAttitude(double *data, double *rotmat);
    void ReadTouchSensors(double *ts_data);
    void GetCamera();
    void InitCamera();
    void killSimulation();
    void ColorMarkers(double *logic, double trans, double *col1);
    void setPositionRotation(double *p, double *r);
    void setCoMmarker(double *p);
    void GetCompass(double *data_i);
    void GetFeetGPS(double *FL_feet_gpos, double *FR_feet_gpos, double *HL_feet_gpos, double *HR_feet_gpos);
    void setServoMaxForce(double *force);
    void GetTailForce(double *tailForce);
  private:
    //================== private variables ===============================================




};


#endif
