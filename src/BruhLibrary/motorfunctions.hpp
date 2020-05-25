#include "main.h"
#include "pid.hpp"
#include "globalvar.hpp"
#pragma once
//#include "utilityfunctions.hpp"
using namespace pros;
/*
TO BE DONES:
  1. ORGANIZE EVERYTHING HERE INTO THEIR OWN RESPECTIVE .hpp files,
     I think we will keep initilizations centalized, maybe moved to main.cpp
  2. FINISH ALL CLASS DEFINITIONS
*/

/*motorf: A motor wrapper for non-base motors which provides the following:
  - PID control system
  - Internal angle scaling system
  - Built in encoder based control
  - Multiple input control schemes
    - Toggles - toggle between 2 preset angles currently - this is the only mode with S curves enabled
    - Double button hold - functioning
    - Joystick axis - not to be done unless nescessary
  - Automated background operation based on sensor inputs
*/ //TBD - make troubleshooting tree for motor tuning, also make a damn interface for this already its so messy as is
struct motorf{
  ADIEncoder* linkedencoder;
  Motor* mot; //this might make a mess, but its only pointed to once so it's ok
  double rotratio, tgt;
  double curpos = 0;
  double constraints[2]; //index 0: upper constraint, index 1: lower constraint
  double* toggletargets; // an array of targets for the toggle to switch between
  bool target = 0;
  double uniquespeedscale;
  controller_digital_e_t* button;
  bool toggleorhold = true; //false is toggle, hold is true
  bool islinked = false;
  PID IntPID; //how do I full copy properly? the current method is bloaty
  motorf(double scalers[], bool ms[], double limits[], double rr[], Motor usedmotor, controller_digital_e_t but[]):IntPID(scalers, ms, limits)
  {mot = &usedmotor; button = but; constraints[0] = rr[0]; constraints[1] = rr[1]; rotratio = rr[2]; rr[3] = uniquespeedscale;}
  motorf(double scalers[], bool ms[], double limits[], double rr[], Motor usedmotor, ADIEncoder LE, controller_digital_e_t but[]):IntPID(scalers, ms, limits)
  {mot = &usedmotor; linkedencoder = &LE; button = but; constraints[0] = rr[0]; constraints[1] = rr[1]; rotratio = rr[2]; islinked = true; rr[3] = uniquespeedscale;}
  motorf(double scalers[], bool ms[], double limits[], dualScurve es, double rr[], Motor usedmotor, controller_digital_e_t but):IntPID(scalers, ms, limits, es)
  {mot = &usedmotor; button = &but; constraints[0] = rr[0]; constraints[1] = rr[1]; rotratio = rr[2]; rr[3] = uniquespeedscale;}
  //PID_MOVE_TARGET: sets PID target
  //Note: the current auton system avoids the usage of this system
  void PID_MOVE_TARGET(double tt){
    tgt = tt;
    IntPID.set_tgt_clean(tt);
  }
  //PID_MOVE_CYCLE: One increment PID update system, returns movement completion
  bool PID_MOVE_CYCLE(){
    updateangle();
    mot->move(IntPID.update(curpos));
    if (fabs(tgt-curpos) < 2) return true;
    return false;
  }
  //Dumb 100% throttle movement
  void move(){
    updateangle();
    PID_MOVE_TARGET(curpos);
    if (toggleorhold){
      if (ctrl.get_digital(button[1]) && !ctrl.get_digital(button[1]) && curpos < constraints[0])
        {mot->move(uniquespeedscale*speedmultiplier*127); return;}
      if (!ctrl.get_digital(button[1]) && ctrl.get_digital(button[1]) && curpos > constraints[1])
        {mot->move(uniquespeedscale*-speedmultiplier*127); return;}
      PID_MOVE_CYCLE();
      return;
    }else{
      if (ctrl.get_digital_new_press(*button)) {
        target = !target; PID_MOVE_TARGET(toggletargets[(int)target]);
        //the above cycling for the array is really stupid but it should work
        //besides, I can't think of cases which we will need more than 2 angle presets
      };
      updateangle();
      PID_MOVE_CYCLE();
      //remember that we have S curves on the PID so this should be less shakey than manual movement if done right
    }
  }
  void updateangle(){
    if (islinked) curpos = rotratio*linkedencoder->get_value();
    else curpos = rotratio*mot->get_position();
  }
  void keyangle(){
    curpos = 0;
  }
};

/*motorw: A motor wrapper for bases which provides the following features:
  - Additional information
    - Motor orientation data
    - Motor wheel size with calculations (INCHES)
    NGL kinda useless, but its here for modularity
*/
struct motorw{
  Motor mot;
  double orientation;
  double cosV, sinV;
  motorw(int pin, bool dir, double o):mot(pin,dir),orientation(o){
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
    double rotationalratio = fabs(r)/(determinebiggest(fabs(x),fabs(y))+fabs(r));
    //above: very sketchy power distrubtion formula between rotation and translation
    for (int i = 0; i < sizeof(MAP); i++){
      //calculation for the power of each motor, see discord #design-ideas for formula
      MAP[i].mot = spd*127*((-x*MAP[i].sinV + y*MAP[i].cosV)*(1-rotationalratio) + rotationalratio*r);
      //above: rotationalratio code made even more sketchier, it doesnt even scale correctly I think
    };
  }
};

/*odometrycontroller: interface for ADI_Encoder to determine the position of the bot:
  - Rotational and translational data updating
*/
