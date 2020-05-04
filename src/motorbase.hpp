#include "main.h"
using namespace pros;
/*ADVANCE DECLARATIONS*/

struct motorw; struct odometrycontroller; struct basecontroller;


/*GLOBAL VARIABLES*/

extern double angleG;
extern double xG;
extern double yG;
extern odometrycontroller odo;
extern motorw basemotors[];
extern basecontroller base;


/*UTLITITY FUNCTIONS*/
//note: making functions here is technically bad practice

//determinebiggest: returns biggest number, not absolute
extern double determinebiggest(double a, double b){
  if (a>b) return a; return b;
};
//isposorneg: returns 1 or -1 depending on if the value is positive or not
extern double isposorneg(double input){
  //there is 100% a better solution but this works so imma keep it for now
  //if someone figures it out pls change it
  return input/fabs(input);
}
//getrelrad: assumes positive radians, whereby 90deg right is 0
extern double getrelrad(double crad, double trad){
  //note: untested but also probably not actually correct. Pls test
  if (fabs(trad-crad) > M_PI) return (2*M_PI-fabs(trad-crad))*isposorneg(trad-crad);
  return (trad-crad)*isposorneg(trad-crad);
}
//rottodist: converts radians into distance based on the radius of rotator
extern double rottodist(double rad, double radius){
  return rad*radius;
}


/*CLASS DECLARATIONS*/

/*motorf: A motor wrapping function for non-base motors which provides the following:
  - PID control system
  - Internal angle scaling system
  - Multiple input control schemes
    - Toggles
    - Double button hold
    - Joystick axis
  - Automated background operation based on sensor inputs
*/
struct motorf{
  Motor mot;

};

/*motorw: A motor wrapping funion for bases which provides the following features:
  - Additional information
    - Motor orientation data
    - Motor wheel size with calculations (INCHES)
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
  basecontroller(motorw m[]){MAP = m;}
  /*vectormove: a universal movement function which takes x and y inputs,
  magnitude is irrelevant in this case, use spd to determine speed*/
  void vectormove(double x, double y, double r, double spd){
    x = x/determinebiggest(fabs(x),fabs(y)); y=y/determinebiggest(fabs(x),fabs(y));
    for (int i = 0; i < sizeof(MAP); i++){
      //calculation for the power of each motor, see discord #design-ideas for formula
      MAP[i].mot = spd*127*(-x*MAP[i].sinV + y*MAP[i].cosV + r);
    };
  }
};

/*odometrycontroller: interface for ADI_Encoder to determine the position of the bot:
  - Rotational and translational data updating
*/
struct odometrycontroller{
  ADIEncoder* left;
  ADIEncoder* right;
  ADIEncoder* back;
  odometrycontroller(ADIEncoder en[]){left = &en[0]; right = &en[1]; back = &en[2];}
  void updateposition(){
    double LD = left->get_value();
    double RD = right->get_value();
    double HD = back->get_value();
  }
};
