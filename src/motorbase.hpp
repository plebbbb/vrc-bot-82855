#include "main.h"
using namespace pros;
/*ADVANCE DECLARATIONS*/

struct motorw;
struct odometrycontroller;
struct basecontroller;
class PID;
struct motorf;
struct dualScurve;

/*GLOBAL DEFINITIONS*/
//in inches or rads

#define Y_AXIS_TWHEEL_OFFSET 10 //offset from center line of the y axis tracking sheel
#define X_AXIS_TWHEEL_OFFSET 7.5 //not being used currently. we'd need another horz wheel for that
#define STD_WHEEL_RADIUS 1.875 //3.75in wheel
#define STD_TWHEEL_RADIUS 1.25 //2.5in wheel


/*GLOBAL VARIABLES*/

extern Controller ctrl;
extern double angleG;
extern double xG;
extern double yG;
extern double heading;
extern double speedmultiplier;
extern odometrycontroller odo;
extern motorw kiwimotors[];
extern basecontroller base;


/*UTLITITY FUNCTIONS*/

//for some reason, sort() is broken so i have to diy something,
//This directly sorts the inputted member, no returns
extern void insertionsort(double arr[]);
//determinebiggest: returns biggest number, not absolute
extern double determinebiggest(double a, double b);
//isposorneg: returns 1 or -1 depending on if the value is positive or not
extern double isposorneg(double input);
//getrelrad: assumes positive radians, whereby 90deg right is 0
extern double getrelrad(double crad, double trad);
//rottodist: converts radians into distance based on the radius of rotator
extern double rottodist(double rad, double radius);


/*CLASS DECLARATIONS*/

//curveS: a single S curve
class curveS{
  double* vars;
public:
  curveS(double arr[]){vars = arr;}
  double getval(double pos){
    return vars[0]/(1+pow(M_E,-vars[1]*(pos-vars[2])));
  }
};

//dualScurve: a set of 2 S curves made to aproximate what motion profiling might look like
struct dualScurve{
  curveS* a;
  curveS* b;
  dualScurve(curveS c, curveS d){a=&c; b=&d;}
  double getval(double pos){
    if (pos < 50) return a->getval(pos);
    else return b->getval(pos);
  };
};

/*PID: generic PID system*/
//NOTE: HAS NOT BEEN TESTED PLS TEST
class PID{
private:
  /*Integral mode configurations:
  false: Direct I scaling - 100% of I is added each cycle
  true: Asymptope I - I approaches the max*/
  bool Imode = 0;

  /*Proportional mode configurations
  false: Raw input
  true: External S curve, percent to target based
  */
  bool Pmode = 0;

  //Izerocutoff: turns I to 0 when target reached
  bool Izerocutoff = true;

  //Pk, Ik, Dk, and I fraction array
  double* ratios;

  //selected dualScurve for Pmode = 1
  dualScurve* Scurve;

  //the max limits for the loop
  double maxIlimit;
  double maxlimit;

  //P, I, D, position, and target array
  double PIDa[4] = {0};

  double lasterror = 0;
public:
  PID(double scalers[], bool ms[]){
    ratios = scalers; Pmode = ms[0]; Imode = ms[1]; Izerocutoff = ms[2]; maxlimit = ms[3]; maxIlimit = ms[4];
  }
  //note: if dualScurve is to be used, input percentage to target values
  PID(double scalers[], bool ms[], dualScurve curve){
    ratios = scalers; Scurve = &curve; Pmode = ms[0]; Imode = ms[1]; Izerocutoff = ms[2]; maxlimit = ms[3]; maxIlimit = ms[4];
  }
  //sets a new target for the loop w/o resetting PIDa
  void set_tgt_soft(double tgt){
    PIDa[3] = tgt;
  }
  //sets a new target for the loop and resets PIDa
  void set_tgt_clean(double tgt){
    PIDa[0]=0;PIDa[1]=0;PIDa[2]=0; //this is stupid
    PIDa[3] = tgt;
  }
  double update(double in){
    double err = (PIDa[3]-in);
    if (Pmode) PIDa[0] = isposorneg(err)*Scurve->getval(fabs(err));
    else PIDa[0] = err;
    if (Imode) PIDa[1] += (err-PIDa[1])/ratios[3];
    else PIDa[1] += err;
    if (Izerocutoff && in == 0) PIDa[1] = 0;
    if (fabs(PIDa[1]) > maxIlimit) PIDa[1] = isposorneg(PIDa[1])*maxIlimit;
    PIDa[2] = err-lasterror;
    lasterror = err;
    double final = PIDa[0]*ratios[0] + PIDa[1]*ratios[1] + PIDa[2]*ratios[2];
    return isposorneg(final)*determinebiggest(fabs(final),maxlimit);
  };
};

/*motorf: A motor wrapper for non-base motors which provides the following:
  - PID control system
  - Internal angle scaling system
  - Built in encoder based control
  - Multiple input control schemes
    - Toggles - currently not added in
    - Double button hold - functioning
    - Joystick axis - not to be done unless nescessary
  - Automated background operation based on sensor inputs
*/
struct motorf{
  ADIEncoder* linkedencoder;
  Motor* mot; //this might make a mess, but its only pointed to once so it's ok
  double rotratio, tgt;
  double curpos = 0;
  double constraints[2];
  controller_digital_e_t* button;
  bool toggleorhold = true; //false is toggle, hold is true
  bool islinked = false;
  PID IntPID; //how do I full copy? the current method is bloaty
  motorf(double scalers[], bool ms[], double rr[], Motor usedmotor, controller_digital_e_t but[]):IntPID(scalers, ms)
  {mot = &usedmotor; button = but; constraints[0] = rr[0]; constraints[1] = rr[1]; rotratio = rr[2];}
  motorf(double scalers[], bool ms[], double rr[], Motor usedmotor, ADIEncoder LE, controller_digital_e_t but[]):IntPID(scalers, ms)
  {mot = &usedmotor; linkedencoder = &LE; button = but; constraints[0] = rr[0]; constraints[1] = rr[1]; rotratio = rr[2]; islinked = true;}
  motorf(double scalers[], bool ms[], double rr[], Motor usedmotor, controller_digital_e_t but):IntPID(scalers, ms)
  {mot = &usedmotor; button = &but; constraints[0] = rr[0]; constraints[1] = rr[1]; rotratio = rr[2];}
  //PID_MOVE_TARGET: sets PID target
  void PID_MOVE_TARGET(double tt){
    tgt = tt;
    IntPID.set_tgt_clean(tt);
  }
  //PID_MOVE_CYCLE: One increment PID update system, returns movement completion
  bool PID_MOVE_CYCLE(){
    mot->move(IntPID.update(curpos));
    if (fabs(tgt-curpos) < 2) return true;
    return false;
  }
  //Dumb 100% throttle movement
  void move(){
    updateangle();
    PID_MOVE_TARGET(curpos);
    if (toggleorhold){
      if (ctrl.get_digital(button[1]) && !ctrl.get_digital(button[1])) {mot->move(speedmultiplier*127); return;}
      if (!ctrl.get_digital(button[1]) && ctrl.get_digital(button[1])) {mot->move(-speedmultiplier*127); return;}
      PID_MOVE_CYCLE();
      return;
    }else{
      //TBD: figure out toggle movement
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
    heading = atan(yN/xN); //this may need to be mellowed out a bit, a new heading every time can be very noisy
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
    a bit of distinction between each layer of hardware interaction. This way,
    troubleshooting, as well as understanding the code can be a bit easier.
*/
struct coordcontroller{
  basecontroller* mBase;

};
