#include "main.h"
#include "motorbase.hpp"
// This cpp is for the declration of extern objects from motorbase.hpp

/*a tip making motorw arrays:
  it is suggested to not inverse motor direction, and instead figure out the
  correct vector direction for positive movement. This prevents potential issues
  with rotation.
*/

//Variables:
double speedmultiplier = 1;
double angleG = 0;
double xG = 0;
double yG = 0;

//ADIEncoder arrays:
//ADIEncoder format: pin 1, pin2, inversed or not
ADIEncoder odencoders[] = {
  ADIEncoder(0,1,true),
  ADIEncoder(2,3,false),
  ADIEncoder(4,5,false)
};

//motorw arrays:
//motorw format: pin, inv dir, orientation, wheel size
motorw kiwimotors[] = {
  motorw(1,true,(M_PI*2)/3), //right motor
  motorw(2,true,(M_PI*4)/3), //left motor
  motorw(3,true,0) //Pure translational Motor
};


//controllers:
Controller ctrl = E_CONTROLLER_MASTER;
odometrycontroller odo(odencoders,Y_AXIS_TWHEEL_OFFSET,X_AXIS_TWHEEL_OFFSET);
basecontroller base(kiwimotors);

//functions:
double determinebiggest(double a, double b){
  if (a>b) return a;
  else return b;
}

double isposorneg(double input){
  //there is 100% a better solution but this works so imma keep it for now
  //if someone figures it out pls change it
  return input/fabs(input);
}

double getrelrad(double crad, double trad){
  //note: untested but also probably not actually correct. Pls test
  if (fabs(trad-crad) > M_PI) return (2*M_PI-fabs(trad-crad))*isposorneg(trad-crad);
  return (trad-crad)*isposorneg(trad-crad);
}

double rottodist(double rad, double radius){
  return rad*radius;
}
