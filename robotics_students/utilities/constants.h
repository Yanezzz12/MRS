// Defining constants

#define LIMIT_SIM 150  // Maximum number of steps
#define MAG_ADVANCE 0.04 // Advance magnitude in meters 
#define TURN_ANGLE PI/8.0
//#define CNT_GOAL 0.2 // constant to find a threshold to reach the goal 
#define CNT_GOAL 0.6 // constant to find a threshold to reach the goal 
//#define K_GOAL CNT_GOAL*MAG_ADVANCE 
//#define K_INTENSITY 1.00*K_GOAL 
#define THRS_SENSOR 1.5*MAG_ADVANCE // 
#ifndef PI
#define PI 3.1415926535f
#endif
#define RADIO_ROBOT 0.03 // Radio Robot in meters
//#define NUM_MAX_SENSORS 4097
#define STOP 0
#define FORWARD 1
#define BACKWARD 2
#define LEFT 3
#define RIGHT 4
#define RIGHTADVANCE 5
#define LEFTADVANCE 6
#define RIGHTADVANCETWICE 7
#define LEFTADVANCETWICE 8
#define OMNI 1
#define NUM_POLICY 7



