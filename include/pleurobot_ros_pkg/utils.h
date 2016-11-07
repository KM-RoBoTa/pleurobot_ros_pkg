/* PLEUROBOT UTILITIES */
#ifndef UTILS_HPP
#define UTILS_HPP

#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/types.h>
#include <net/if.h>
#include <ifaddrs.h>
#include <netdb.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <arpa/inet.h>
#include <stdlib.h>
#include "Eigen/Dense"
#include "Eigen/Geometry"
#include <sys/time.h>
#include <stdio.h>
#include <sstream>
#include <fstream>
#include <string>
#include <iostream>

using namespace Eigen;
using namespace std;

#define MX64_CENTER	2047
#define RX28_CENTER 512
#define RPM		0.11

// unit transformation functions

// for position
	// MX64
#define P_S2R64(X) ((X - MX64_CENTER)*3.14159/MX64_CENTER)
#define P_R2S64(X) ((int)((X/3.14159)*MX64_CENTER) + MX64_CENTER)
	//RX28
#define P_S2R28(X) ((X - RX28_CENTER)*3.14159/RX28_CENTER)
#define P_R2S28(X) ((int)((X/3.14159)*RX28_CENTER) + RX28_CENTER)

// for speed
#define S_S2R64(X) ((X > 1023) ? -((X-1024)*RPM*2*3.14159/60) : (X*RPM*2*3.14159/60))

// for torque (only MX64)
#define T2S64(X) ((X >= 0) ?  (int)((1000.0/7.5)*fabs(X)) : (int)((1000.0/7.5)*fabs(X)+1024.0))

// for torque v2 (MX64)
#define T2S64_v2(X) (X >= 1024) ?  -(X-1024) : X

// for current (MX64)
#define C2S64(X) (double) 0.0045*(X-2048);

#define BILLION          1000000000.0
#define MILLION          1000000.0

//for activation signals
#define NUM_ACTIVATION_PARAMS 8
#define NORMALIZE(X,Y)	((X >= Y) ? 1 : (X/Y))

enum {U1_LOCATION,
 	 U1_DURATION,
 	 U1_STEEPNESS,
 	 U2_LOCATION,
 	 U2_DURATION,
 	 U2_STEEPNESS,
  	 U1_AMPLITUDE,
 	 U2_AMPLITUDE};
float compute_pulse_activation(float, float, float, float, float, float);

// for time
double get_real_time();
double get_timestamp();

// for keyboard
int kbhit();
int kbevent(int);

// for sending data to PC
int sendUDP(void *data, int len, const char *IP, int UDP_PORT);

// filtering
double pt1(double u, double y, double T, double Ts);
MatrixXd pt1_vec(MatrixXd u, MatrixXd y, double T, double Ts);

// reading file
void readFileWithLineSkipping(ifstream& inputfile, stringstream& file);

// load global config
int readGlobalConfig();

// basic rotation matrices
Matrix3d rotx(double ang);
Matrix3d roty(double ang);
Matrix3d rotz(double ang);

#endif
