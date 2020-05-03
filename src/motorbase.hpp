#include "main.h"
using namespace pros;
/*motorw: A motor wrapping function which provides the following features:
  - Additional information
    - Motor orientation data
    - Motor wheel size with calculations (INCHES)
  - PID motor control
*/
struct motorw{
  Motor mot;
  double orientation;
  double wheelsize;
  double cosV, sinV;
  motorw(int pin, bool dir):mot(pin,dir){}
  motorw(int pin, bool dir, double o, double ws):mot(pin,dir),orientation(o), wheelsize(ws){
    //precalculated values for basecontroller vector control
    cosV = cos(orientation); sinV = sin(orientation);
  }
};
/*motorbase: A wrapper for motorw which seeks to control base movement:
  - Parametric motor configurations
  - Directional control with speed controls
  - Rotational control with PID stabilization
*/
struct basecontroller{
  motorw* MAP; //sketchy pointer that points to the motorw array so we can use it later
  int arrlen; //even sketchier int that presents the length of the motorw array
  basecontroller(motorw m[]):arrlen(sizeof(*m)){MAP = m;}
/*This version of vectormove is intended for controller usage:
  - x and y magnitudes determine the speed.
*/
  void vectormove(double x, double y, double r){
    for (int i = 0; i < arrlen; i++){
      MAP[i].mot = (-x*MAP[i].sinV + y*MAP[i].cosV + r);
      //calculation for the power of each motor, see discord #design-ideas for formula
      /*note that we dont multiply by 127 b/c this vectormove is for opcontrol,
      and the controller's input is already from -127 to 127, instead of -1 to 1*/
    };
  }
/*This version of vectormove is intended for auton usage
  - x and y magnitudes must be scaled so that the biggest magnitude is 1
*/
  void vectormove(double x, double y, double r, double spd){
    for (int i = 0; i < arrlen; i++){
      MAP[i].mot = spd*127*(-x*MAP[i].sinV + y*MAP[i].cosV + r);
      //calculation for the power of each motor, see discord #design-ideas for formula
    };
  }
};
extern motorw basemotors[];
extern basecontroller base;
