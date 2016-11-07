#include "pleurobot_ros_pkg/utils.h"

extern int IS_SIMULATION, IS_PLEUROBOT, IS_OPTIMIZATION, USE_JOYSTICK, JOYSTICK_TYPE, SWIM, IS_SNAKE;
extern int SPINE_COMPENSATION_WITH_FEEDBACK, USE_REFLEXES, USE_TAIL, USED_MOTORS[27], TORQUE_CTRL_MOTORS[27];
extern int USE_IMU, AUTO_RUN, AUTO_RUN_STATE, LOG_DATA, ANIMAL_DATASET;

/* Read current time */
double
get_real_time()
{
	struct timespec real_time;

	if (clock_gettime(CLOCK_REALTIME, &real_time) == -1 )
	{
		perror("clock gettime");
		exit( EXIT_FAILURE );
	}

	return((double)real_time.tv_sec + (double)(real_time.tv_nsec/BILLION));
}

/* Read current time - this one is being used*/
double
get_timestamp()
{
    struct timeval now;
    gettimeofday (&now, NULL);
    return  (now.tv_usec + (unsigned long long)now.tv_sec * 1000000)/1000000.;
}

/* PT1 filter for scalars */
double
pt1(double u, double y, double T, double Ts)
{
    y=exp(-Ts/T)*y+(1-exp(-Ts/T))*u;
    return y;
}

/* PT1 filter for vectors */
MatrixXd
pt1_vec(MatrixXd u, MatrixXd y, double T, double Ts)
{
    y=exp(-Ts/T)*y+(1-exp(-Ts/T))*u;
    return y;
}

/* Sends an UDP package "128.178.148.59" */
int
sendUDP(void *data, int len, const char *IP, int UDP_PORT)
{

	static int sock = socket(AF_INET, SOCK_DGRAM, 0);

	struct sockaddr_in saddr = {0,};

	saddr.sin_family = AF_INET;
	saddr.sin_addr.s_addr = inet_addr(IP);
	saddr.sin_port = htons(UDP_PORT);


	//double t=0;




	//	double data[27 * 4];
	//	for(int i=0;i<27*4;i++){
	//		data[i]=t;

	//	}
	//	t+=0.001;
       // sendto(sock, (char *)data, len*sizeof(double), 0, (struct sockaddr *)&saddr, sizeof(saddr));
        sendto(sock, (char *)data, len, 0, (struct sockaddr *)&saddr, sizeof(saddr));
     //       double data2[3]={rand(),2,3};
     //      printf("%d\n", sendto(sock, (char *)data2, sizeof(data2), 0, (struct sockaddr *)&saddr, sizeof(saddr)));
       //     usleep(8e3);


 // create socket
/*
    const int port = 8472;
    Socket socket;
    if ( !socket.Open( port ) )
    {
        printf( "failed to create socket!\n" );
        return false;
    }efined reference to `JOYSTICK_TYPE'


    // send a packet

    const char data[] = "hello world!";
    socket.Send( Address(128,178,148,59,port), data, sizeof( data ) );
*/

	return 0;
}

/* Reads file, skipps commented lines starting with "//" and stores the data into an array */
void
readFileWithLineSkipping(ifstream& inputfile, stringstream& file){
    string line;
    file.str(std::string());
    while (!inputfile.eof()){
        getline(inputfile,line);
        //line.erase(line.begin(), find_if(line.begin(), line.end(), not1(ptr_fun<int, int>(isspace))));
        if (line.length() == 0 || (line[0] == '#') || (line[0] == ';')){
            //continue;
        }
        else
            file << line << "\n";
    }
}

// load global config
int readGlobalConfig(){
    stringstream stringstream_file;
    ifstream global_config_file("/home/mparsapo/catkin_ws/src/pleurobot/config/GLOBAL.config");
    if(global_config_file.is_open()){
        readFileWithLineSkipping(global_config_file, stringstream_file);
        stringstream_file >> IS_SIMULATION;
        stringstream_file >> IS_PLEUROBOT;
        stringstream_file >> IS_SNAKE;
        stringstream_file >> IS_OPTIMIZATION;
        stringstream_file >> USE_JOYSTICK;
        stringstream_file >> JOYSTICK_TYPE;
        stringstream_file >> SWIM;
        stringstream_file >> SPINE_COMPENSATION_WITH_FEEDBACK;
        stringstream_file >> USE_REFLEXES;
        stringstream_file >> USE_TAIL;
        stringstream_file >> USE_IMU;
        stringstream_file >> AUTO_RUN;
        stringstream_file >> AUTO_RUN_STATE;
        stringstream_file >> LOG_DATA;

        for(int i=0; i<27; i++){
          stringstream_file >> USED_MOTORS[i];
        }

        for(int i=0; i<27; i++){
          stringstream_file >> TORQUE_CTRL_MOTORS[i];
        }

        stringstream_file >> ANIMAL_DATASET;
        return 1;
    }
        
    else{
        return 0;
    }

}

/* rotation matrix around x axis */
Matrix3d
rotx(double ang){
    Matrix3d mat;
    mat <<  1, 0, 0, 
            0, cos(ang), -sin(ang), 
            0, sin(ang), cos(ang);
    return mat;
}

/* rotation matrix around y axis */
Matrix3d
roty(double ang){
    Matrix3d mat;
    mat <<  cos(ang), 0, sin(ang),
            0, 1, 0,
            -sin(ang), 0, cos(ang);
    return mat;
}

/* rotation matrix around z axis */
Matrix3d
rotz(double ang){
    Matrix3d mat;
    mat <<  cos(ang), -sin(ang), 0,
            sin(ang), cos(ang), 0,
            0,0,1;

    return mat;
}








