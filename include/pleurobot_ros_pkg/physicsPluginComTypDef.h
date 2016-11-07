#include <stdbool.h>

// TYPE DEFINITIONS ---------------------------------------------------
#ifndef N_SERVOS
#define N_SERVOS  27
#endif

// data packet sent from controller to physics plugin
typedef struct
{
  bool status[N_SERVOS];
  float torques[N_SERVOS];
}c2p_packet;

// data packet sent from physics plugin to controller
typedef struct
{
  bool status[N_SERVOS];
  float torques[N_SERVOS];
}p2c_packet;