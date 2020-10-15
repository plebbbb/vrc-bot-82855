#include "main.h"
using namespace pros;
#pragma once

//********************************************************************************//
/*GLOBAL DEFINITIONS*/
//in inches or rads

#define Y_AXIS_TWHEEL_OFFSET 6.95 //offset from center line of the y axis tracking wheel
#define X_AXIS_TWHEEL_OFFSET 11.9 //offset from center lie of the x axis tracking wheel
#define STD_WHEEL_RADIUS 1.625 //4in base wheels
#define STD_TWHEEL_RADIUS 1.375 //2.75in wheel for left-right tracking wheels
#define STD_BTWHEEL_RADIUS 2.00 //4in wheel used on back wheel for testing bot
#define BASE_MOTOR_RPM 200 //base motor rpm
#define U_TGT_THRESHOLD 1000 //trigger threshold in mv for intake pot
#define AOM_P_VAL 2 //angle optimization control loop P multiplier
//#define AXIS_COUNT 3; //different motor axises we must interact with, base direction/angle excluded

//********************************************************************************//
/*GLOBAL VARIABLES*/
//note that this is technically bad coding practice in the sense that the interactions to these variables may get convoluted due to the sheer size of the code
//luckily for us, our design targets aren't actually big enough for this to be a significant problem so we can kinda get away with it
//To kinda bypass this problem, all use cases of the more obscure variables will be listed here.


//V5 controller
extern Controller ctrl;

//Global angle, rads, positive
extern double angleG;

//Global x coordinate
extern double xG;

//Global y coordinate
extern double yG;

//Global relative changes from last cycle
extern double xR;
extern double yR;

//Array of target x, y, and angle
extern double xyaT[3]; //is making this an array a good idea or not tbh not sure

//Observed heading of robot, rads
extern double heading; //may end up useless

//Observed velocity of base
extern double estspd;

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


//opcontrol configuration
extern controller_analog_e_t controlscheme[];
extern bool configoptions[];

//Pascal's triangle
extern const std::vector<short> Ptriangle[];

//amount of axises
extern const int AXIS_COUNT;

//global coordcontroller parameters
//technically this is bad practice but at the same time going through all the layers to get to it is a painful thing
extern bool anglemode; //enable/disable angle optimization
extern bool GVT; //global velocity target
