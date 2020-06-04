#include "main.h"
#pragma once
#include "motorfunctions.hpp"
//#include "pid.hpp"
//#include "globalvar.hpp"
//#include "utilityfunctions.hpp"
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
    xR = xN;
    yR = yN;
    //50 - we run a 20ms clock, converts to in/seconds
    estspd = sqrt(xR*xR + yR*yR)*50;
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
    While it 100% is kinda stupid to hsave this many layers, this is done to allow
    a bit of distinction between each layer of sortware interaction. This way,
    troubleshooting, as well as understanding the code can be a bit easier.
*/
struct coordcontroller{
  basecontroller* mBase;
  PID* axiscontrollers; //the initial plan called for 3 PID controllers to allow for smooth motion curves, but for now we have a direct line approach
  double* tcoords; //we are gonna try a potentially stupid approach, where we dont call coordcontroller but instead change the tgt coords directly
  coordcontroller(basecontroller a, PID b[2], double t[3]){mBase = &a; axiscontrollers = b; tcoords = t;}
  /*returns true when target is reached
    potential camera implementation: overload update with version that replaces r and perp with camera controls
    this overload would input the desired color profile that the camera is looking for.
    note that constructor must be updated for this*/
  bool update(){
    double yO = 0;
    //note that it isnt really nescessary, but made to minimize the risk of swaying in circles, it itself is disabled
    //past a certain point for safety's sake, although it is likely isn't gonna do anything weird when we get close to the target
    double xD = (xG-tcoords[0])*sin(angleG)+(yG-tcoords[1])*cos(angleG); //relative distances to target
    double yD = (yG-tcoords[1])*sin(angleG)+(xG-tcoords[0])*cos(angleG); //relative distances to target
    //unsure about recent correction from sin(angleG-pi/2) to cos(angleG), the thing is inversed but my initial math is probably wrong
    double rD = getrelrad(angleG,tcoords[2]); //VERY janky pls confirm if getrelrad works
    if ((sqrt(pow(xD,2)+pow(yD,2))) > 20) yO = axiscontrollers[2].update(getrelrad(heading, atan2(xG-tcoords[0],yG-tcoords[1])));
    //PID offset system if the motors aren't 100% correct orientation wise. May cause potential spinning issues near target
    //Below: Sketchy, and most likely redundent math to account for yO in the local coordinate system
    xD+=yO*sin(atan2(xD,yD));
    yD+=yO*cos(atan2(xD,yD));
    mBase->vectormove(xD,yD,rD,
      //above: unsure about subtracting yO or adding it
      //we do fabs because basecontroller already handles backwards vectors, so reversing power is useless
      fabs(axiscontrollers[0].update(sqrt(pow(xD,2)+pow(yD,2))))+
      fabs(axiscontrollers[1].update(rD))
    );
    if (fabs(axiscontrollers[0].update(sqrt(pow(xD,2)+pow(yD,2))))+fabs(axiscontrollers[1].update(rD)) < 15) return true;

    return false;
  }

  //this variation is for usage with motionpaths, where axiscontrollers merely maintains the speed target given by TSP
  bool update(double TSP){
    double yO = 0;
    //note that it isnt really nescessary, but made to minimize the risk of swaying in circles, it itself is disabled
    //past a certain point for safety's sake, although it is likely isn't gonna do anything weird when we get close to the target
    double xD = (xG-tcoords[0])*sin(angleG)+(yG-tcoords[1])*cos(angleG); //relative distances to target
    double yD = (yG-tcoords[1])*sin(angleG)+(xG-tcoords[0])*cos(angleG); //relative distances to target
    //unsure about recent correction from sin(angleG-pi/2) to cos(angleG), the thing is inversed but my initial math is probably wrong
    double rD = getrelrad(angleG,tcoords[2]); //VERY janky pls confirm if getrelrad works
    if ((sqrt(pow(xD,2)+pow(yD,2))) > 20) yO = axiscontrollers[2].update(getrelrad(heading, atan2(xG-tcoords[0],yG-tcoords[1])));
    //PID offset system if the motors aren't 100% correct orientation wise. May cause potential spinning issues near target
    //Below: Sketchy, and most likely redundent math to account for yO in the local coordinate system
    xD+=yO*sin(atan2(xD,yD));
    yD+=yO*cos(atan2(xD,yD));
    mBase->vectormove(xD,yD,rD,
      //above: unsure about subtracting yO or adding it
      //below: we do fabs because basecontroller already handles backwards vectors, so reversing power is useless
      fabs(axiscontrollers[0].update(mBase->getlimits(TSP)-estspd))+
      fabs(axiscontrollers[1].update(rD))
    );
    if (fabs(axiscontrollers[0].update(mBase->getlimits(TSP)-estspd))+fabs(axiscontrollers[1].update(rD)) < 15) return true;

    return false;
  }
};

/*motionpath:
    The goal of this is to implement movement paths through multiple points without
    stopping

    The current method of using a global trajectory array is kinda sketchy to use
    if we are gonna end up using this approach, but it's probably doable.
    The trajectory array is going to become an imaginary target though.

    The code design for this will have 3 development stages
      1. Converting PID speeds into percentage points for S curves, and making
        the bot not stop after each curve by making the PID percentage be over
        the entire trajectory
          This will be done by either:
            - Making Tcoords into a bait coordinate, and getting axiscontrollers to
            automatically convert to percentage inside coordcontroller by extending
            tcoords percent units ahead in the ideal(line) direction (only good for 1.)
            - Make low key motion profiling and then turn the axis controller from being
            the primary source of movement into something used to maintain the profiling
            speed targets
      2. Adding a speed control when turning corners to minimize shaking from inertia(we might skip this)
      3. Making a curved transision between point to basically eliminate shaking

      Current implementation: step 1 done, low key motion profiling implementation
*/
struct motionpath{
  coordcontroller* controller;
  double** points; //not sure about how to make a pointer for this 2d array case
  dualScurve* tcurve;
  double percentage = 0;
  double oldlen = 0;
  int Tindex = 0;
  bool isfullycomplete = false;
  motionpath(coordcontroller b, double** loc){controller = &b; points = loc;
    oldlen = sqrt(pow(xG-points[Tindex][0],2) + pow(xG-points[Tindex][1],2));
  }
  bool update(){
    double TSP = points[Tindex][3];
    double newlen = sqrt(pow(xG-points[Tindex][0],2) + pow(xG-points[Tindex][1],2));
    percentage = ((newlen/oldlen)*100);
    if (Tindex == sizeof(points)-1 && percentage >= 50) TSP = tcurve->getval(percentage);
    if (Tindex == 0 && percentage <= 50) TSP = tcurve->getval(percentage);
    if (controller->update(TSP)){
      Tindex++;
      if (Tindex == sizeof(points)) isfullycomplete = true;
      oldlen = sqrt(pow(xG-points[Tindex][0],2) + pow(xG-points[Tindex][1],2));
      return true;
    }
    return false;
  }
};
