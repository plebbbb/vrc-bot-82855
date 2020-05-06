#include "main.h"
#include "global.hpp"
// This cpp is for the declration of extern objects from global.hpp

/*a tip making motorw arrays:
  it is suggested to not inverse motor direction, and instead figure out the
  correct vector direction for positive movement. This prevents potential issues
  with rotation.
*/


//********************************************************************************//
//Variables:
double speedmultiplier = 1;
double angleG = 0;
double xG = 0;
double yG = 0;
double heading = angleG;
double xyaT[3] = {0,0,angleG};


//********************************************************************************//
//ADIEncoder arrays:
//ADIEncoder format: pin 1, pin2, inversed or not
//Array format: Left, Right, Back
ADIEncoder odencoders[3] = {
  ADIEncoder(0,1,true),
  ADIEncoder(2,3,false),
  ADIEncoder(4,5,false)
};


//********************************************************************************//
//motorw arrays:
//motorw format: pin, inv dir, orientation, wheel size
motorw kiwimotors[] = {
  motorw(1,true,(M_PI*2)/3), //right motor
  motorw(2,true,(M_PI*4)/3), //left motor
  motorw(3,true,0) //Pure translational Motor
};


//********************************************************************************//
//PID Variables

//********************************************************************************//
//PIDKvals format: Pk, Ik, Dk
double PIDKvals[][3] = {
  {1,1,1}
};

//********************************************************************************//
/*PIDKvals format:
S-curve enable/disable,
fractional I enable/disable,
I hardstop at target enable/disable*/
bool PIDSvals[][3] = {
  {false,false,true}
};

//********************************************************************************//
//PIDL values: PID hard limit, I value hard limit
double PIDLvalues[][2] = {
  {100,50}
};

//********************************************************************************//
//PID controllers
//constructor scheme: PIDKvals, PIDSvals, PIDLvals, (OPTIONAL)dualScurve

//PID for base navigation
PID bPID[] = {
  PID(PIDKvals[0],PIDSvals[0],PIDKvals[0]),
  PID(PIDKvals[0],PIDSvals[0],PIDKvals[0])
};

//********************************************************************************//
//actual controllers
Controller ctrl = E_CONTROLLER_MASTER;
odometrycontroller odo(odencoders,Y_AXIS_TWHEEL_OFFSET,X_AXIS_TWHEEL_OFFSET);
basecontroller base(kiwimotors);
coordcontroller mover(base,bPID,xyaT);

//********************************************************************************//
//functions:
double determinebiggest(double a, double b){
  if (a>b) return a;
  else return b;
}

double isposorneg(double input){
  //there is 100% a better solution but this works so imma keep it for now
  //if someone figures it out pls change it
  return input/fabs(input); //its supposted to return 1 or -1, fyi
}

double getrelrad(double crad, double trad){
  //note: untested but probably correct. Pls test
  if (fabs(trad-crad) > M_PI) return (2*M_PI-fabs(trad-crad))*isposorneg(trad-crad);
  return (trad-crad)*isposorneg(trad-crad);
}

//this is pretty stupid
double rottodist(double rad, double radius){
  return rad*radius;
}

//this is solely for convience -
//NGL tho for everyone who already has to use radians, just get used to them
double degtorad(double deg){
  return fmod(deg*(M_PI/180),2*M_PI);
};
