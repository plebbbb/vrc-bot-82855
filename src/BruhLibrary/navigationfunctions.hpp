#include "main.h"
#include "utilityfunctions.hpp"
#include "navigationfunctions.hpp"
#include "pid.hpp"
#include "utilityfunctions.hpp"
using namespace pros;


struct odometrycontroller{
  ADIEncoder* left;
  ADIEncoder* right;
  ADIEncoder* back;
  double ds, db; //offsets from center of rotation, not diameter
  odometrycontroller(ADIEncoder en[],double s, double b):ds(s),db(b){left = &en[0]; right = &en[1]; back = &en[2];}
  //updates position, should be triggered once every 10ms or so
  void updateposition(){
    double LD = rottodist(left->get_value(),STD_TWHEEL_RADIUS);
    double RD = rottodist(right->get_value(),STD_TWHEEL_RADIUS);
    double HD = rottodist(back->get_value(),STD_TWHEEL_RADIUS);//for calculating lateral shift we make a perpendicular line on our arc
    double relangle = (RD-LD)/(2*ds); //100% this part works
    double chordlength = 2*(RD/relangle)*sin(relangle/2); //85% this part works
    double xN = chordlength*cos(angleG)+HD*sin(angleG+(relangle/2)); //15% this and below work
    double yN = chordlength*sin(angleG)+HD*cos(angleG+(relangle/2));
    xG+=xN; yG+=yN;
    heading = fmod(atan(yN/xN),(2*M_PI)); //this may need to be mellowed out a bit, a new heading every time can be very noisy
    angleG = fmod((angleG+relangle),(2*M_PI)); //technically sketchy but not really still pls test
    left->reset(); //these resets dont seem to be reliable, so we may have to resort to storing the pre update value
    right->reset();
    back->reset();
  }
  //keys position to a hardcoded target, use for wall allignments
  void key_position(double x, double y, double r){
    left->reset(); //these resets dont seem to be reliable, so we may have to resort to storing the pre update value
    right->reset();
    back->reset();
    xG = x;
    yG = y;
    angleG = r;
  }
};

/*coordcontroller: a wrapper for basecontroller to intepret coordinate grid inputs
    While it 100% is kinda stupid to have this many layers, this is done to allow
    a bit of distinction between each layer of sortware interaction. This way,
    troubleshooting, as well as understanding the code can be a bit easier.
*/
struct coordcontroller{
  basecontroller* mBase;
  PID* axiscontrollers; //the initial plan called for 3 PID controllers to allow for smooth motion curves, but for now we have a direct line approach
  double* tcoords; //we are gonna try a potentially interesting approach, where we dont call coordcontroller but instead change the tgt coords directly
  coordcontroller(basecontroller a, PID b[2], double t[3]){mBase = &a; axiscontrollers = b; tcoords = t;}
  /*returns true when target is reached
    potential camera implementation: overload update with version that replaces r and perp with camera controls
    this overload would input the desired color profile that the camera is looking for.
    note that constructor must be updated for this*/
  bool update(){
    double xD = xG-tcoords[0]; //relative distances to target
    double yD = yG-tcoords[1]; //relative distances to target
    double rD = getrelrad(angleG,tcoords[2]); //VERY janky pls confirm if getrelrad works
    mBase->vectormove(xD,yD,rD,
       //we do fabs because basecontroller already handles backwards vectors, so reversing power is useless
      fabs(axiscontrollers[0].update(sqrt(pow(xD,2)+pow(yD,2))))+
      fabs(axiscontrollers[1].update(rD)));
    return false;
  }
};
