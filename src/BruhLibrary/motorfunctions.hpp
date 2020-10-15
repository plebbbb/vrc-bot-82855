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
*/ //TBD - make troubleshooting tree for motor tuning
struct motorf{ //TO BE TESTED
  ADIEncoder* linkedencoder;
  Motor mot; //this might make a mess, but its only pointed to once so it's ok
  double rotratio, tgt;
  double curpos = 0;
  double constraints[2]; //index 0: upper constraint, index 1: lower constraint
  double* toggletargets; // an array of targets for the toggle to switch between
  bool target = false;
  double uniquespeedscale;
  controller_digital_e_t* button;
  bool toggleorhold = false; //false is toggle, hold is true
  bool islinked = false;
  PID IntPID; //how do I full copy properly? the current method is bloaty
  motorf(double scalers[], bool ms[], double limits[], int motpin, controller_digital_e_t but):IntPID(scalers, ms, limits),mot(motpin)
  { button = &but; toggleorhold = false;}
  motorf(double scalers[], bool ms[], double limits[], double rr[], int motpin, controller_digital_e_t but[]):IntPID(scalers, ms, limits),mot(motpin)
  { button = but; constraints[0] = rr[0]; constraints[1] = rr[1]; rotratio = rr[2]; rr[3] = uniquespeedscale;}
  motorf(double scalers[], bool ms[], double limits[], double rr[], int motpin, ADIEncoder LE, controller_digital_e_t but[]):IntPID(scalers, ms, limits),mot(motpin)
  { linkedencoder = &LE; button = but; constraints[0] = rr[0]; constraints[1] = rr[1]; rotratio = rr[2]; islinked = true; rr[3] = uniquespeedscale;}
  motorf(double scalers[], bool ms[], double limits[], dualScurve es, double rr[], int motpin, controller_digital_e_t but):IntPID(scalers, ms, limits, es),mot(motpin)
  { button = &but; constraints[0] = rr[0]; constraints[1] = rr[1]; rotratio = rr[2]; rr[3] = uniquespeedscale;}
  //PID_MOVE_TARGET: sets PID target
  //Note: the current auton system avoids the usage of this system
  void PID_MOVE_TARGET(double tt){
    tgt = tt;
    IntPID.set_tgt_soft(tt);
  }
  //PID_MOVE_CYCLE: One increment PID update system, returns movement completion
  bool PID_MOVE_CYCLE(){
    updateangle();
    mot.move(IntPID.update(curpos));
    if (fabs(tgt-curpos) < 2) return true;
    return false;
  }
  //Dumb 100% throttle movement
  void move(){
    updateangle();
    PID_MOVE_TARGET(curpos);
    if (toggleorhold){
      if (ctrl.get_digital(button[1]) && !ctrl.get_digital(button[1]) && curpos < constraints[0])
        {mot.move(uniquespeedscale*speedmultiplier*127); return;}
      if (!ctrl.get_digital(button[1]) && ctrl.get_digital(button[1]) && curpos > constraints[1])
        {mot.move(uniquespeedscale*-speedmultiplier*127); return;}
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
    else curpos = rotratio*mot.get_position();
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
//also its actually efficient, at least the vector calculation parts. The if statements, idk.
//we dont even call a single cos or sin function, and only do multiplication and adding, so its super fast compared to the other approaches people have been showing
//we make an assumption that there is always a motor which perfectly counters any unwanted forces tho, which works for the most part but also
//means that we will end up relying a lot on heading PIDs for auton as our base isn't actually perfectly symmetrical
struct basecontroller{
  motorw* MAP; //sketchy pointer that points to the motorw array so we can use it later
  //double vals[4];
  double rotationalratio;
  basecontroller(motorw m[]){MAP = m;}
  /*vectormove: a universal movement function which takes x and y inputs,
  magnitude is irrelevant in this case, use spd to determine speed*/
  void vectormove(double x, double y, double r, double spd){
    spd = spd*(BASE_MOTOR_RPM/100); //adjustments to max out performance
    double xyspeedlimit = fabs(x)+fabs(y);
    double biggest = determinebiggest(fabs(x),fabs(y));
    rotationalratio = r/(biggest+fabs(r)); //this should be above x and y scale conversion
    if (x != 0) x = x/xyspeedlimit; //x scale conversion
    if (y != 0) y = y/xyspeedlimit; //y scale conversion
    //rotationalratio = 1;
    //above: very sketchy power distrubtion formula between rotation and translation
    for (int i = 0; i < sizeof(MAP); i++){
      //vals[i] = (spd*(((x*MAP[i].cosV + y*MAP[i].sinV)*(1-rotationalratio)) + rotationalratio));
      //calculation for the power of each motor, see discord #design-ideas for formula
      MAP[i].mot.move_velocity(spd*((x*MAP[i].cosV + y*MAP[i].sinV)*(1-rotationalratio) + rotationalratio));
      //above: rotationalratio code made even more sketchier, it doesnt even scale correctly I think
    //MAP[i].mot = (spd*BASE_MOTOR_RPM*((-x*MAP[i].sinV + y*MAP[i].cosV)*(1-rotationalratio) + rotationalratio*r));
      //also above: haha imagine using DIY position pid when you can use already tuned velocity PID
    };
  }
};

//opcontrolcontroller: wrapper for basecontroller to be used during manual drive.
//we may implement motorF features into it as the situation dictates
struct opcontrolcontroller{
    basecontroller* ssc; //pointer to basecontroller
    controller_analog_e_t* controls; //joystick inputs
    bool* configuration; //config for code
    double tang;
    PID* rot;
    opcontrolcontroller(basecontroller b, controller_analog_e_t* css, PID ro, bool* config){
      ssc = &b; controls = css; ; rot = &ro; configuration = config; tang = angleG;}
    //tbd - deal with interia issues from rotation at high speeds, PID insta targets what happens when analog stick is 0
    void move(){
      double rs = -deadzonecompute(ctrl.get_analog(controls[2]));
      if (configuration[1]) rs = rotationcompute(); //TBD: implement auto 45 degree angle holder here, use the ? notation
      if (configuration[0]) relativemove(rs);
      else absolutemove(rs);
    }
    double determinespeed(double p1, double p2, double p3){
      return (speedmultiplier/127)*determinebiggest( //div by speedmultiplier/127 to scale joystick values to percentage values
       fabs(p1),
       determinebiggest(
         fabs(p2),
         fabs(p3)
       )
     );
    }
    double rotationcompute(){
      if (deadzonecompute(ctrl.get_analog(controls[2]))){tang = angleG; return -deadzonecompute(ctrl.get_analog(controls[2]));} //max rot output
      return rot->update(getrelrad(tang, angleG)); //PID stabilization to hold last input orientaiton
    }
    void relativemove(double rotation){
      ssc->vectormove(
        (double)ctrl.get_analog(controls[0]),
        (double)ctrl.get_analog(controls[1]),
        rotation,
        determinespeed(ctrl.get_analog(controls[0]),ctrl.get_analog(controls[1]),rotation)
      );
    }
    //a reminder, this is hard offset pi/2 counterclockwise to account for angleG = pi/2 for the forwards direction
    //also I have no idea how to do this without an offset now
    void absolutemove(double rotation){
      ssc->vectormove(
        (double)(ctrl.get_analog(controls[0])*cos(getrelrad(angleG-M_PI/2,0))+ctrl.get_analog(controls[1])*cos(getrelrad(angleG,M_PI))),
        (double)(ctrl.get_analog(controls[1])*sin(getrelrad(angleG,M_PI))+ctrl.get_analog(controls[0])*sin(getrelrad(angleG-M_PI/2,0))),
        rotation,
        determinespeed(ctrl.get_analog(controls[0]),ctrl.get_analog(controls[1]),rotation)
      );
    //  lcd::print(6,"local x axis: %f", (ctrl.get_analog(controls[0])*cos(getrelrad(angleG-M_PI/2,0))+ctrl.get_analog(controls[1])*cos(getrelrad(angleG,M_PI))));
    //  lcd::print(7,"local y axis: %f", (ctrl.get_analog(controls[1])*sin(getrelrad(angleG,M_PI))+ctrl.get_analog(controls[0])*sin(getrelrad(angleG-M_PI/2,0))));
    }
    double deadzonecompute(double in){
      if (in > 10) return in;
      return 0;
    }
  };

struct MotorSys{
  double OPT; //filler variable to output sensor results
  Motor* set;
  bool iscomplete = false;
  int c = 0; int t = 0; //c: counter for sensor triggers. Used to dictate how many balls must transfer before shutdown, t: threshold before flagging iscomplete for other
  MotorSys(Motor e[]){
    set = e;
  }
  void update();
  void NC(int count, int TT){
    c = count;
    t = TT;
    iscomplete = false;
  }
  void NR(){ //hard reset when toggle threshold exceeded in motorsysinterface
    c = 0;
    t = 0;
    iscomplete = false;
  }
};

//note: it's likely that we will have to add a motor shutdown delay
struct Intakes: public MotorSys{
  bool PT = false; //change to true if we have preload in the intake;
  ADIAnalogIn g;
  Intakes(Motor e[], int p):MotorSys(e),g(p){};
  void update(){
    if (c) {set[0].move_velocity(127);set[1].move_velocity(127);} else {set[0].move_velocity(0);set[1].move_velocity(0);}
    if (g.get_value() > U_TGT_THRESHOLD && !PT) {c--; PT = true;} //currently the sensor is set up as a potentiometer, we may end up using something else tho
    if (g.get_value() < U_TGT_THRESHOLD && PT) PT = false;
    if (c == t) iscomplete = true;
    OPT = g.get_value();
  }
};

//note: there is a planned linking feature so that we can have MotorSysinterfaces sequentially activate: e.g intakes detect a ball which enables the other motors
//however, such a feature is kinda painful to do and the edge cases which it tackles are rare enough that we can probably cheese through those situations
struct MotorSysInterface{
  MotorSys* sys;
  bool CMPL = false; //completion param which is detached from
  bool P = false; //previously triggered check
  double PA; //activation percentage
  double PD; //deactication percentage
  int Cin; //Count param
  int Tval; //Completion flag control, for the MotorSys
  int TvalL; //Completion flag tied locally to CMPL
  MotorSysInterface(MotorSys A, int count, int toggleval, int AVP, int DVP, int CPT){ //case with synched CMPL and MotorSys completion, this can be desynced if need be
    Cin = count; Tval = toggleval; PA = AVP; PD = DVP; sys = &A; TvalL = Tval;
  }
  bool update(double perc){
    if (!P && PA < perc && PD > perc) {P = true; sys->NC(Cin,Tval);}
    if (sys->c == TvalL) CMPL = true;
    return CMPL;
  }
};
