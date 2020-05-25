#include "main.h"
using namespace pros;
#pragma once

//********************************************************************************//
/*GLOBAL DEFINITIONS*/
//in inches or rads

#define Y_AXIS_TWHEEL_OFFSET 10 //offset from center line of the y axis tracking sheel
#define X_AXIS_TWHEEL_OFFSET 7.5 //not being used currently. we'd need another horz wheel for that
#define STD_WHEEL_RADIUS 1.625 //3.25in wheel for main. To be confirmed
#define STD_TWHEEL_RADIUS 1.25 //2.5in wheel for tracking wheels


//********************************************************************************//
/*GLOBAL VARIABLES*/
//TBD - use std::vector when it's actually nescessary instead of spamming so many arrays

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

//S curve template values
extern double Scurvevals[][4];

//auton selector values
extern bool confirmedauton;
extern int selectedauton;
