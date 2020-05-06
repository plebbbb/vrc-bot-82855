#include "main.h"
#include "utilityfunctions.hpp"
#include "navigationfunctions.hpp"
#include "pid.hpp"
#include "utilityfunctions.hpp"
using namespace pros;
/*
TO BE DONES:
  1. ORGANIZE EVERYTHING HERE INTO THEIR OWN RESPECTIVE .hpp files,
     I think we will keep initilizations centalized, maybe moved to main.cpp
  2. FINISH ALL CLASS DEFINITIONS
*/

//********************************************************************************//
/*GLOBAL DEFINITIONS*/
//in inches or rads

#define Y_AXIS_TWHEEL_OFFSET 10 //offset from center line of the y axis tracking sheel
#define X_AXIS_TWHEEL_OFFSET 7.5 //not being used currently. we'd need another horz wheel for that
#define STD_WHEEL_RADIUS 1.625 //3.25in wheel for main. To be confirmed
#define STD_TWHEEL_RADIUS 1.25 //2.5in wheel for tracking wheels


//********************************************************************************//
/*GLOBAL VARIABLES*/

//V5 controller
extern Controller ctrl;

//Global angle, rads, positive
extern double angleG;

//Global x coordinate
extern double xG;

//Global y coordinate
extern double yG;

//Array of target x, y, and angle
extern double xyaT[3]; //is making this an array a good idea or not tbh not sure

//Observed heading of robot, rads
extern double heading; //may end up useless

//global slowmode speed multiplier
extern double speedmultiplier;

//array of position sets for auton to reach, to be revised upon design completion
extern double positionsetTEST[][3];

//PID template values
extern double PIDKvals[][3];
extern bool PIDSvals[][3];
extern double PIDLvalues[][2];

//PID template instances
extern PID bPID[];

//global hardware interface layers
extern odometrycontroller odo;
extern motorw kiwimotors[];
extern motorw xdrivemotors[];
extern basecontroller base;
extern coordcontroller mover;
//TBD: add all the other controllers
